"""Bounded explicit-arm drive contract. Importing opens no ports or ROS nodes.

Manual atomic envelopes only. No Twist subscription or preview gateway.
각 arm은 기본5초이며 명시적 옵션만10초를 허용합니다. RPM 기본3, 선택5/10.
"""
import math
import struct
import time

from follower_motor_port import ExclusivePort
from follower_motor_candidate import MotorCore,Config,read_request,velocity_packet
from follower_motor_mock import parse_envelope


class BoundedDrivePort(ExclusivePort):
    def __init__(self,device,clock=time.monotonic,*,max_rpm=3):
        if type(max_rpm) is not int or max_rpm not in (3,5,10,15):
            raise ValueError('시험 상한은 3, 5, 10, 15RPM 중 하나여야 합니다.')
        super().__init__(device);self.clock=clock;self.deadline=None;self.max_rpm=max_rpm

    @classmethod
    def open(cls,port,baudrate=19200,*,max_rpm=3):
        if type(max_rpm) is not int or max_rpm not in (3,5,10,15):
            raise ValueError('시험 상한은 3, 5, 10, 15RPM 중 하나여야 합니다.')
        result=super().open(port,baudrate)
        result.max_rpm=max_rpm
        return result

    def open_gate(self,deadline):self.deadline=deadline
    def close_gate(self):self.deadline=None

    def write(self,packet):
        if packet in (read_request(1),velocity_packet(0,0,1)):
            return self.device.write(packet)
        if (len(packet)!=13 or packet[:5]!=b'\xb7\xb8\x01\xcf\x07'
                or packet[5]!=1 or packet[8]!=1 or packet[11]!=1 or sum(packet)&255):
            raise ValueError('Only bounded ID1 PID207/ENABLE1/return216 packets allowed')
        left,right=struct.unpack_from('<h',packet,6)[0],struct.unpack_from('<h',packet,9)[0]
        if self.deadline is None or not self.clock()<self.deadline:
            raise ValueError('Motion gate locked/expired')
        # Raw M1 negative is forward; raw M2 positive is forward.
        if not -self.max_rpm<=left<=0 or not 0<=right<=self.max_rpm:
            raise ValueError(f'전진 방향 및 {self.max_rpm}RPM 상한만 허용합니다.')
        return self.device.write(packet)


class DriveCore(MotorCore):
    def _record_feedback(self,data,now):
        super()._record_feedback(data,now)
        if data['m1_status']!=0 or data['m2_status']!=0:
            if getattr(self,'last_device_status_error',None) is None:
                self.last_device_status_error=dict(mono=now,
                    m1_status=data['m1_status'],m2_status=data['m2_status'],
                    m1_rpm=data['m1_rpm'],m2_rpm=data['m2_rpm'])
            self.latch_fault('DEVICE_STATUS_NONZERO')


class DriveContract:
    def __init__(self,port,*,enabled=False,clock=time.monotonic,max_rpm=3,trial_seconds=5):
        if type(max_rpm) is not int or max_rpm not in (3,5,10,15) or port.max_rpm!=max_rpm:
            raise ValueError('제어기와 포트의 시험 RPM 상한이 같아야 합니다.')
        if type(trial_seconds) is not int or trial_seconds not in (5,10,30,60):
            raise ValueError('시험 시간 상한은 5초, 10초, 30초, 60초 중 하나여야 합니다.')
        self.trial_seconds=trial_seconds
        self.port=port;self.enabled=enabled
        self.core=DriveCore(port,dry_run=False,allow_motion=enabled,config=Config(max_rpm=max_rpm),clock=clock)
        self.deadline=None;self.accepted=0;self.rejected=0
        self.last_command_rejection=None
        self.zero_samples=0

    def arm(self,ros_ns):
        if not self.enabled or self.core.fault is not None or self.zero_samples<5:return None
        epoch=self.core.arm(self.core.epoch,ros_ns)
        if epoch is not None:
            self.deadline=self.core.clock()+self.trial_seconds
            self.port.open_gate(self.deadline)
        return epoch

    def disarm(self):
        self.port.close_gate();self.core.disarm();self.deadline=None

    def command(self,text,ros_ns,*,require_source=False):
        # 정지 후 지연 패킷은 실행하거나 새 오류로 잠그지 않고 폐기한다.
        # 기존 오류는 보존하고, 재arm 후 이전 세션 거부는 아래 검사로 유지한다.
        if not self.core.armed or self.core.closed:
            self.rejected+=1
            return False
        obj=None;expected_epoch=self.core.epoch
        def rejection(reason):
            trace=dict(reason=str(reason)[:200],received_ros_ns=ros_ns,armed_at_rejection=True,
                       arm_ros_ns=self.core.arm_ros_ns)
            if isinstance(obj,dict):
                trace['epoch_matches']=obj.get('epoch')==expected_epoch
                for key in ('stamp_ns','source_stamp_ns'):
                    value=obj.get(key)
                    if type(value) is int:
                        trace[key]=value
                        try:age=(ros_ns-value)/1e9
                        except OverflowError:continue
                        if math.isfinite(age):trace[key+'_age_s']=age
            self.last_command_rejection=trace
        try:
            if require_source:
                from follower_guard_gateway import parse_guard_envelope
                obj=parse_guard_envelope(text)
                source=obj['source_stamp_ns'];arm=self.core.arm_ros_ns
                if (type(arm) is not int or source<arm or source>obj['stamp_ns']
                        or not 0<=(ros_ns-source)/1e9<=.3):
                    raise ValueError('Original measurement stale/future/pre-arm')
            else:obj=parse_envelope(text)
            v,w=obj['v'],obj['w']
            if not 0<=v<=.1 or abs(w)>.2 or v<abs(w)*self.core.config.track/2:
                raise ValueError('Reverse/in-place/excess velocity request')
            ok=self.core.accept_command(v,w,frame=obj['frame'],stamp_ns=obj['stamp_ns'],
                                        ros_now_ns=ros_ns,epoch=obj['epoch'])
            if not ok:rejection(self.core.fault or 'CORE_REJECTED')
        except (ValueError,TypeError,KeyError,OverflowError,RecursionError) as exc:
            rejection(exc)
            self.core.latch_fault('INVALID_DRIVE_ENVELOPE');ok=False
        if ok:self.accepted+=1
        else:self.rejected+=1
        if self.core.fault:self.port.close_gate()
        return ok

    def tick(self,ros_ns):
        if self.core.armed and self.deadline is not None and self.core.clock()>=self.deadline:
            self.core.latch_fault({5:'FIVE_SECOND_TRIAL_EXPIRED',10:'TEN_SECOND_TRIAL_EXPIRED',30:'THIRTY_SECOND_TRIAL_EXPIRED',60:'SIXTY_SECOND_TRIAL_EXPIRED'}[self.trial_seconds])
        if not self.core.armed:self.port.close_gate()
        before=self.core.valid_count
        self.core.tick(ros_ns)
        if self.core.valid_count!=before:
            f=self.core.feedback
            zero=all(f[k]==0 for k in ('m1_rpm','m2_rpm','m1_status','m2_status'))
            self.zero_samples=self.zero_samples+1 if zero else 0
        if self.core.fault:self.port.close_gate()

    def status(self):
        c=self.core
        return dict(armed=c.armed,enabled=self.enabled,fault=c.fault,epoch=c.epoch,arm_ros_ns=c.arm_ros_ns,
                    trial_max_rpm=c.config.max_rpm,
                    trial_max_seconds=self.trial_seconds,
                    feedback_fresh=c.feedback_fresh(),valid_feedback=c.valid_count,
                    measured_rpm=None if c.feedback is None else [c.feedback['m1_rpm'],c.feedback['m2_rpm']],
                    device_status=None if c.feedback is None else
                        dict(m1=c.feedback['m1_status'],m2=c.feedback['m2_status']),
                    last_device_status_error=getattr(c,'last_device_status_error',None),
                    sent_rpm=c.last_sent,io_failed=c.io_failed,
                    last_io_error=c.last_io_error,
                    last_command_rejection=self.last_command_rejection,
                    last_response_error=None if c.last_response_error is None else
                        {k:v for k,v in c.last_response_error.items() if k!='history'},
                    last_feedback_error=None if c.last_feedback_error is None else
                        {k:v for k,v in c.last_feedback_error.items() if k!='history'},
                    zero_response_retries=c.zero_wait_retry_count,
                    zero_retry_discarded_bytes=c.zero_retry_discarded_bytes,
                    feedback_age_s=None if c.last_feedback is None else max(0.,c.clock()-c.last_feedback),
                    trial_remaining_s=0. if not c.armed or self.deadline is None else max(0.,self.deadline-c.clock()),
                    accepted_commands=self.accepted,rejected_commands=self.rejected)

    def shutdown(self):
        self.port.close_gate();self.core.shutdown()
