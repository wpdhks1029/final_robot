"""Motor transaction/state-machine candidate; NO port opening and NO ROS node.

Only injected nonblocking transports may be used. Default dry-run never touches
the transport. No command-line hardware mode is provided at this stage.
Software stop cannot guarantee stopping on SIGKILL, serial loss or power faults.
"""
from dataclasses import dataclass
from collections import deque
import math
from pathlib import Path
import secrets
import struct
import sys
import time

sys.path.insert(0,str(Path.home()/'robot_ws/test_tools'))
from md200t_pid216_v2 import decode
from md200t_pid216_rx_file import extract_frames


def read_request(driver_id=1):
    data=bytes([0xB7,0xB8,driver_id,4,1,216])
    return data+bytes([(-sum(data))&255])


def velocity_packet(left,right,driver_id=1):
    if not all(isinstance(v,int) and -32768<=v<=32767 for v in (left,right)):
        raise ValueError('RPM must be signed int16')
    # Preserve the user-verified ENABLE=1 at zero RPM; request PID216 response.
    data=bytes([0xB7,0xB8,driver_id,207,7])+struct.pack('<BhBhB',1,left,1,right,1)
    return data+bytes([(-sum(data))&255])


@dataclass(frozen=True)
class Config:
    radius: float=.065
    track: float=.560
    invert_left: bool=True
    invert_right: bool=False
    driver_id: int=1
    max_rpm: int=3  # Trial cap matching the earlier lifted-wheel command, not ground certification.
    command_timeout: float=.3
    feedback_timeout: float=.3
    response_timeout: float=.3
    period: float=.1
    max_feedback_rpm: int=200  # Plausibility trial bound, independent of command cap.
    max_integration_gap: float=.25
    raw_linear_feedback_scale: float=1.0

    def __post_init__(self):
        if not math.isfinite(self.raw_linear_feedback_scale) or not .5<=self.raw_linear_feedback_scale<=1.5:
            raise ValueError('Invalid raw linear feedback scale')
        if not all(math.isfinite(v) and v>0 for v in
                   (self.radius,self.track,self.command_timeout,self.feedback_timeout,
                    self.response_timeout,self.period,self.max_integration_gap)):
            raise ValueError('Geometry/timing must be finite and positive')
        if not (1<=self.driver_id<=254 and isinstance(self.driver_id,int)
                and isinstance(self.max_rpm,int) and 0<self.max_rpm<=32767
                and isinstance(self.max_feedback_rpm,int) and 0<self.max_feedback_rpm<=32767):
            raise ValueError('Invalid ID or RPM limits')


class MotorCore:
    def __init__(self,transport=None,*,dry_run=True,allow_motion=False,
                 config=None,clock=time.monotonic):
        self.config=config or Config()
        self.transport=transport
        self.dry_run=dry_run
        self.allow_motion=allow_motion
        self.clock=clock
        if not dry_run:
            if transport is None or transport.timeout!=0:
                raise ValueError('Hardware adapter must supply one nonblocking transport')
            wt=transport.write_timeout
            if wt is None or not 0<=wt<=.02:
                raise ValueError('Transport write timeout must be bounded to <=20ms')
        self.epoch=secrets.token_hex(8)
        self.armed=False
        self.fault=None
        self.io_failed=False
        self.last_io_error=None
        self.last_response_error=None
        self.last_feedback_error=None
        self.zero_stop_reply_retry_enabled=False
        self.zero_wait_retry_used=False;self.zero_wait_retry_count=0
        self.last_rx_at=None;self.zero_retry_discarded_bytes=0
        self.io_history=deque(maxlen=24)
        self.closed=False
        self.command=None
        self.last_command_stamp=None
        self.armed_at=None
        self.arm_ros_ns=None
        self.last_tick=None
        self.last_feedback=None
        self.feedback=None
        self.buffer=bytearray()
        self.waiting_since=None
        self.next_io=0.
        self.pending_stops=1
        self.last_sent=(0,0)
        self.tx_count=0
        self.valid_count=0
        self.discarded_count=0
        self.odom_x=self.odom_y=self.odom_yaw=0.
        self.odom_v=self.odom_w=0.
        self.odom_sample_at=None
        self.integration_skips=0

    def feedback_fresh(self,now=None):
        now=self.clock() if now is None else now
        return self.last_feedback is not None and 0<=now-self.last_feedback<=self.config.feedback_timeout

    def latch_fault(self,reason):
        if self.fault is None:
            self.fault=reason
            self.pending_stops=max(self.pending_stops,3)
        if self.armed:
            self.epoch=secrets.token_hex(8)
        self.armed=False
        self.command=None

    def arm(self,expected_epoch,ros_now_ns):
        # A successful explicit arm rotates the epoch and clears any queued command.
        if (self.closed or self.io_failed or not self.allow_motion or not self.feedback_fresh()
                or self.feedback is None or self.feedback['m1_rpm']!=0 or self.feedback['m2_rpm']!=0
                or expected_epoch!=self.epoch or ros_now_ns<=0):
            return None
        self.epoch=secrets.token_hex(8)
        self.armed=True;self.fault=None;self.command=None
        self.last_command_stamp=None
        self.armed_at=self.clock();self.arm_ros_ns=ros_now_ns
        return self.epoch

    def disarm(self):
        self.io_history.append(dict(event='disarm',mono=self.clock(),last_sent=list(self.last_sent)))
        self.armed=False;self.command=None
        self.epoch=secrets.token_hex(8)
        self.pending_stops=max(self.pending_stops,3)

    def accept_command(self,v,w,*,frame,stamp_ns,ros_now_ns,epoch):
        if not self.armed or self.closed:
            return False
        valid=(epoch==self.epoch and frame=='follower/base_link'
               and isinstance(stamp_ns,int) and isinstance(ros_now_ns,int)
               and stamp_ns>0 and stamp_ns>=self.arm_ros_ns
               and (self.last_command_stamp is None or stamp_ns>self.last_command_stamp)
               and 0<=(ros_now_ns-stamp_ns)/1e9<=self.config.command_timeout
               and math.isfinite(v) and math.isfinite(w) and v>=0)
        if not valid:
            self.latch_fault('INVALID_COMMAND');return False
        self.command=(float(v),float(w),self.clock(),stamp_ns)
        self.last_command_stamp=stamp_ns
        return True

    def rpm(self,v,w):
        c=self.config
        left=(v-w*c.track/2)*60/(2*math.pi*c.radius)
        right=(v+w*c.track/2)*60/(2*math.pi*c.radius)
        if not math.isfinite(left) or not math.isfinite(right):
            raise ValueError('Nonfinite wheel request')
        scale=max(1.,abs(left)/c.max_rpm,abs(right)/c.max_rpm)
        left,right=int(round(left/scale)),int(round(right/scale))
        return (-left if c.invert_left else left,-right if c.invert_right else right)

    def _record_feedback(self,data,now):
        if any(abs(data[k])>self.config.max_feedback_rpm for k in ('m1_rpm','m2_rpm')):
            self.latch_fault('IMPLAUSIBLE_RPM');return
        self.feedback=data.copy();self.last_feedback=now;self.valid_count+=1
        self.io_history.append(dict(event='feedback',mono=now,
                                    rpm=[data['m1_rpm'],data['m2_rpm']]))
        c=self.config
        left=-data['m1_rpm'] if c.invert_left else data['m1_rpm']
        right=-data['m2_rpm'] if c.invert_right else data['m2_rpm']
        vl,vr=left*2*math.pi*c.radius/60,right*2*math.pi*c.radius/60
        v,w=(vl+vr)/2,(vr-vl)/c.track
        v*=c.raw_linear_feedback_scale
        if self.odom_sample_at is not None:
            dt=now-self.odom_sample_at
            if 0<dt<=c.max_integration_gap:
                mid=self.odom_yaw+w*dt/2
                self.odom_x+=v*math.cos(mid)*dt
                self.odom_y+=v*math.sin(mid)*dt
                self.odom_yaw=math.atan2(math.sin(self.odom_yaw+w*dt),math.cos(self.odom_yaw+w*dt))
            else:
                self.integration_skips+=1
        self.odom_v,self.odom_w=v,w
        self.odom_sample_at=now

    def _receive(self,now):
        count=self.transport.in_waiting
        if count>65536:
            self.latch_fault('RX_OVERFLOW');self.transport.reset_input_buffer();self.buffer.clear();return
        if count:
            chunk=self.transport.read(min(count,4096))
            if chunk:self.last_rx_at=now
            self.io_history.append(dict(event='rx',mono=now,size=len(chunk),
                                        hex=bytes(chunk[:64]).hex(' ')))
            self.buffer.extend(chunk)
        for frame in extract_frames(self.buffer,controller_id=self.config.driver_id):
            data=decode(frame,controller_id=self.config.driver_id,tmid=0xB8)
            if (data is None or self.waiting_since is None
                    or not 0<=now-self.waiting_since<self.config.response_timeout):
                self.discarded_count+=1;continue
            self.waiting_since=None
            self._record_feedback(data,now)
        if len(self.buffer)>8192:
            self.buffer.clear();self.latch_fault('RX_OVERFLOW')

    def _transmit(self,packet,now,*,preempt=False,preserve_response_deadline=False,preserve_input=False):
        if preserve_input and not preserve_response_deadline:raise ValueError('Input preservation requires a zero deadline-preserving retry')
        if preserve_response_deadline and (not preempt or packet!=velocity_packet(0,0,self.config.driver_id)
                or (self.armed and not (self.zero_stop_reply_retry_enabled and self.command is not None
                                       and self.command[:2]==(0.,0.)))):
            raise ValueError('Only disarmed or explicitly enabled zero-command retries may retain a response deadline')
        previous_wait=self.waiting_since
        operation='prepare_write';written=None
        try:
            if preempt and not preserve_input:
                self.waiting_since=None;self.buffer.clear()
                operation='reset_input_before_write'
                self.transport.reset_input_buffer()
            operation='write';written=self.transport.write(packet)
            if written!=len(packet):
                raise RuntimeError('Partial serial write')
            self.waiting_since=(previous_wait if preserve_response_deadline and previous_wait is not None else now)
            if not preserve_response_deadline:self.zero_wait_retry_used=False
            self.next_io=now+self.config.period;self.tx_count+=1
            self.io_history.append(dict(event='tx',mono=self.clock(),request_started=now,
                                        packet_hex=bytes(packet).hex(' '),preempt=preempt,
                                        response_started=self.waiting_since))
            if packet[3]==207:
                self.last_sent=(struct.unpack_from('<h',packet,6)[0],struct.unpack_from('<h',packet,9)[0])
            return True
        except Exception as exc:
            self.last_io_error=dict(operation=operation,type=type(exc).__name__,message=str(exc)[:500],
                                    packet_hex=bytes(packet).hex(' '),expected_bytes=len(packet),written_bytes=written)
            self.io_failed=True;self.latch_fault('SERIAL_WRITE_ERROR')
            self._best_effort_stop()
            return False

    def _best_effort_stop(self):
        # Bounded attempts, never interpreted as measured stopping confirmation.
        for _ in range(3):
            try:self.transport.write(velocity_packet(0,0,self.config.driver_id))
            except Exception:pass

    def tick(self,ros_now_ns):
        if self.closed:return
        now=self.clock()
        tick_gap=None if self.last_tick is None else now-self.last_tick
        if self.last_tick is not None and now<self.last_tick:
            self.latch_fault('MONOTONIC_CLOCK_BACKWARDS')
        self.last_tick=now
        # Never read/write even an injected transport in default dry-run.
        if self.dry_run:return
        if self.io_failed:return
        operation='read_or_decode'
        try:
            # Check expiration before accepting queued RX as current response.
            if self.waiting_since is not None and now-self.waiting_since>=self.config.response_timeout:
                self.last_response_error=dict(
                    mono=now,waiting_s=now-self.waiting_since,
                    tick_gap_s=tick_gap,
                    since_feedback_s=None if self.last_feedback is None else now-self.last_feedback,
                    armed=self.armed,last_sent=list(self.last_sent),pending_stops=self.pending_stops,
                    rx_available_bytes=self.transport.in_waiting,buffered_bytes=len(self.buffer),
                    buffered_hex=bytes(self.buffer[:256]).hex(' '),
                    execution_delay_exceeded_timeout=(tick_gap is not None and tick_gap>=self.config.response_timeout),
                    tx_count=self.tx_count,valid_feedback=self.valid_count,history=list(self.io_history))
                self.waiting_since=None;self.buffer.clear();self.transport.reset_input_buffer()
                self.latch_fault('RESPONSE_TIMEOUT')
            self._receive(now)
        except Exception as exc:
            self.last_io_error=dict(operation=operation,type=type(exc).__name__,message=str(exc)[:500])
            self.io_failed=True;self.latch_fault('SERIAL_READ_ERROR')
            self._best_effort_stop()
            return
        if self.armed:
            if not self.feedback_fresh(now):
                try:available=self.transport.in_waiting
                except Exception:available=None
                self.last_feedback_error=dict(mono=now,tick_gap_s=tick_gap,
                    feedback_age_s=None if self.last_feedback is None else now-self.last_feedback,
                    response_waiting_s=None if self.waiting_since is None else now-self.waiting_since,
                    last_sent=list(self.last_sent),rx_available_bytes=available,buffered_bytes=len(self.buffer),
                    buffered_hex=bytes(self.buffer[:256]).hex(' '),valid_feedback=self.valid_count,tx_count=self.tx_count,
                    zero_retry_used=self.zero_wait_retry_used,zero_retry_count=self.zero_wait_retry_count,
                    zero_retry_discarded_bytes=self.zero_retry_discarded_bytes,
                    last_rx_age_s=None if self.last_rx_at is None else now-self.last_rx_at,history=list(self.io_history))
                self.latch_fault('FEEDBACK_STALE')
            elif self.command is None:
                if now-self.armed_at>self.config.command_timeout:self.latch_fault('COMMAND_TIMEOUT')
            else:
                _,_,rx,stamp=self.command
                if not (0<=now-rx<=self.config.command_timeout
                        and 0<=(ros_now_ns-stamp)/1e9<=self.config.command_timeout):
                    self.latch_fault('COMMAND_TIMEOUT')
        target=(0,0)
        if self.armed and self.fault is None and self.command is not None:
            try:target=self.rpm(*self.command[:2])
            except ValueError:self.latch_fault('INVALID_WHEEL_REQUEST')
        urgent_stop=(target==(0,0) and self.last_sent!=(0,0))
        if urgent_stop:self.pending_stops=max(self.pending_stops,1)
        if urgent_stop or (self.pending_stops and self.waiting_since is not None and self.fault is not None):
            if self._transmit(velocity_packet(0,0,self.config.driver_id),now,preempt=True):self.pending_stops-=1
            return
        # Retry one lost zero response during an explicit STOP only. Never send
        # another moving command or postpone either original safety deadline.
        # Keep actively arriving fragments. A stalled incomplete prefix cannot
        # confirm feedback; discard only that parser prefix before a zero retry.
        if (self.zero_stop_reply_retry_enabled and self.armed and self.fault is None
                and self.command is not None and self.command[:2]==(0.,0.)
                and target==(0,0) and self.last_sent==(0,0) and self.waiting_since is not None
                and not self.zero_wait_retry_used and now>=self.next_io
                and self.last_feedback is not None and now-self.last_feedback>=.18
                and (not self.buffer or (self.last_rx_at is not None and now-self.last_rx_at>=.06))):
            if self.buffer:
                self.io_history.append(dict(event='stalled_partial_zero_retry',mono=now,
                    idle_rx_s=now-self.last_rx_at,discarded_bytes=len(self.buffer),hex=bytes(self.buffer).hex(' ')))
                self.zero_retry_discarded_bytes+=len(self.buffer);self.buffer.clear()
            self.zero_wait_retry_used=True;self.zero_wait_retry_count+=1
            self.io_history.append(dict(event='zero_response_retry',mono=now,feedback_age_s=now-self.last_feedback))
            self._transmit(velocity_packet(0,0,self.config.driver_id),now,preempt=True,
                           preserve_response_deadline=True,preserve_input=True)
            return
        # disarm의 첫 영속도 응답이 잘리거나 소실돼도 남은 영속도 재전송을
        # 그 응답 뒤에 묶어 두지 않는다. 기존 period에 맞춰 재전송하되,
        # 첫 정지 요청의 응답 기한은 연장하지 않는다. 비영속도는 재전송하지 않는다.
        if (not self.armed and self.pending_stops and self.waiting_since is not None
                and self.last_sent==(0,0) and now>=self.next_io):
            if self._transmit(velocity_packet(0,0,self.config.driver_id),now,preempt=True,
                              preserve_response_deadline=True):self.pending_stops-=1
            return
        if self.waiting_since is not None or now<self.next_io:return
        if self.pending_stops:
            if self._transmit(velocity_packet(0,0,self.config.driver_id),now,preempt=True):self.pending_stops-=1
        elif self.armed and self.command is not None:
            self._transmit(velocity_packet(*target,self.config.driver_id),now)
        else:
            self._transmit(read_request(self.config.driver_id),now,preempt=True)

    def shutdown(self):
        if self.closed:return
        self.disarm()
        if not self.dry_run and self.transport is not None:
            self._best_effort_stop()
            try:self.transport.close()
            except Exception:pass
        self.closed=True


if __name__=='__main__':
    print('Motor state-machine candidate only. No ROS node/serial port started.')
    print('Run test_motor_candidate.py for fake-serial verification. This module never opens hardware ports.')
