"""Motor core tests: fake clock and fake byte transport, no ROS or physical I/O."""
from dataclasses import replace
import math
import random
import struct
import unittest

from follower_motor_candidate import MotorCore,Config,read_request,velocity_packet


def frame(left=0,right=0,driver_id=1,length=14):
    data=struct.pack('<hBihBi',left,0,1,right,0,2)+(b'\0\0' if length==16 else b'')
    raw=bytes([0xB8,0xB7,driver_id,216,length])+data
    return raw+bytes([(-sum(raw))&255])


def rpms(packet):
    return struct.unpack_from('<h',packet,6)[0],struct.unpack_from('<h',packet,9)[0]


class Clock:
    def __init__(self):self.now=1.
    def __call__(self):return self.now
    @property
    def ros(self):return 100_000_000_000+int((self.now-1.)*1e9)


class FakeSerial:
    timeout=0
    write_timeout=.01
    def __init__(self):
        self.rx=bytearray();self.sent=[];self.autoreply=True
        self.reply=frame();self.read_error=False;self.write_error=False
        self.partial=False;self.closed=False;self.reads=0
    @property
    def in_waiting(self):
        if self.read_error:raise OSError('fake read failure')
        return len(self.rx)
    def read(self,n):
        self.reads+=1;raw=bytes(self.rx[:n]);del self.rx[:n];return raw
    def reset_input_buffer(self):self.rx.clear()
    def write(self,raw):
        self.sent.append(raw)
        if self.write_error:raise OSError('fake write failure')
        if self.partial:return len(raw)-1
        if self.autoreply:self.rx.extend(self.reply)
        return len(raw)
    def close(self):self.closed=True


class MotorTests(unittest.TestCase):
    def setup_core(self,allow=True):
        clock=Clock();serial=FakeSerial()
        core=MotorCore(serial,dry_run=False,allow_motion=allow,clock=clock)
        core.tick(clock.ros);clock.now+=.02;core.tick(clock.ros)
        return core,serial,clock

    def arm(self,core,clock):
        session=core.arm(core.epoch,clock.ros)
        self.assertIsNotNone(session)
        return session

    def cmd(self,core,clock,v=.02,w=0.,session=None):
        return core.accept_command(v,w,frame='follower/base_link',stamp_ns=clock.ros,
                                   ros_now_ns=clock.ros,epoch=session or core.epoch)

    def pump(self,core,clock,duration,refresh=False):
        end=clock.now+duration
        while clock.now<end-1e-9:
            clock.now+=.02
            if refresh and core.armed:self.cmd(core,clock)
            core.tick(clock.ros)

    def test_default_dry_run_no_io(self):
        s=FakeSerial();c=MotorCore(s)
        c.tick(100);c.shutdown()
        self.assertEqual(s.sent,[]);self.assertEqual(s.reads,0);self.assertFalse(s.closed)

    def test_packets_match_verified_protocol(self):
        self.assertEqual(read_request().hex(),'b7b8010401d8b3')
        self.assertEqual(velocity_packet(0,0).hex(),'b7b801cf0701000001000001b7')
        for left,right in [(0,0),(-3,3),(3,0),(0,3)]:
            raw=velocity_packet(left,right)
            self.assertEqual(len(raw),13);self.assertEqual(sum(raw)%256,0)
            self.assertEqual((raw[5],raw[8]),(1,1));self.assertEqual(rpms(raw),(left,right))

    def test_invalid_config_and_transport_rejected(self):
        for cfg in [dict(radius=0.),dict(track=float('nan')),dict(period=-1.),dict(max_rpm=0),dict(driver_id=0)]:
            with self.assertRaises(ValueError):Config(**cfg)
        for timeout,write_timeout in [(1,.01),(0,None),(0,.5)]:
            s=FakeSerial();s.timeout=timeout;s.write_timeout=write_timeout
            with self.assertRaises(ValueError):MotorCore(s,dry_run=False)

    def test_startup_zero_then_poll_locked(self):
        c,s,k=self.setup_core();self.pump(c,k,.4)
        self.assertEqual(rpms(s.sent[0]),(0,0));self.assertTrue(any(p[3]==4 for p in s.sent))
        self.assertTrue(all(p[3]!=207 or rpms(p)==(0,0) for p in s.sent));self.assertFalse(c.armed)

    def test_allow_motion_false_cannot_arm(self):
        c,s,k=self.setup_core(False);self.assertIsNone(c.arm(c.epoch,k.ros))

    def test_no_feedback_cannot_arm(self):
        c=MotorCore(allow_motion=True);self.assertIsNone(c.arm(c.epoch,100))

    def test_stale_feedback_cannot_arm(self):
        c,s,k=self.setup_core();k.now+=.4;self.assertIsNone(c.arm(c.epoch,k.ros))

    def test_nonzero_measured_rpm_cannot_arm(self):
        c,s,k=self.setup_core();c.feedback['m1_rpm']=1
        self.assertIsNone(c.arm(c.epoch,k.ros))

    def test_arm_rotates_epoch_and_clears_command(self):
        c,s,k=self.setup_core();old=c.epoch;new=self.arm(c,k)
        self.assertNotEqual(old,new);self.assertIsNone(c.command)
        self.assertFalse(self.cmd(c,k,session=old));self.assertEqual(c.fault,'INVALID_COMMAND')

    def test_forward_wheel_signs_and_cap(self):
        c,s,k=self.setup_core();self.arm(c,k);self.assertTrue(self.cmd(c,k))
        self.pump(c,k,.2,refresh=True)
        moving=[rpms(p) for p in s.sent if p[3]==207 and rpms(p)!=(0,0)]
        self.assertTrue(moving);self.assertTrue(all(l<0 and r>0 and max(abs(l),abs(r))<=3 for l,r in moving))
        for v,w in [(1.,0.),(.01,.2),(.01,-.2),(0.,.1)]:
            self.assertLessEqual(max(map(abs,c.rpm(v,w))),3)

    def test_command_timeout_stops_and_latches(self):
        c,s,k=self.setup_core();self.arm(c,k);self.cmd(c,k)
        self.pump(c,k,.5)
        self.assertFalse(c.armed);self.assertEqual(c.fault,'COMMAND_TIMEOUT')
        self.assertEqual(rpms([p for p in s.sent if p[3]==207][-1]),(0,0))

    def test_no_first_command_deadline(self):
        c,s,k=self.setup_core();self.arm(c,k);self.pump(c,k,.4)
        self.assertEqual(c.fault,'COMMAND_TIMEOUT')

    def test_zero_command_preempts_waiting_reply(self):
        c,s,k=self.setup_core();self.arm(c,k);self.cmd(c,k);self.pump(c,k,.11,refresh=True)
        self.assertNotEqual(c.last_sent,(0,0))
        s.autoreply=False;s.rx.clear();k.now+=.02
        self.cmd(c,k,v=0.);c.tick(k.ros)
        self.assertEqual(rpms(s.sent[-1]),(0,0))

    def test_stop_preempts_partial_reply_and_new_feedback_can_recover(self):
        c,s,k=self.setup_core();self.arm(c,k);self.cmd(c,k);self.pump(c,k,.2,refresh=True)
        s.autoreply=False;s.rx.clear()
        # 전진 응답의 일부를 수신한 상태에서 정지를 요청한다.
        c.waiting_since=k.now;s.rx.extend(frame(left=-3,right=3)[:9])
        k.now+=.02;self.cmd(c,k);c.tick(k.ros)
        self.assertEqual(len(c.buffer),9)
        k.now+=.02;c.disarm();c.tick(k.ros)
        self.assertEqual(rpms(s.sent[-1]),(0,0))
        # 이전 응답의 잔여 바이트 뒤에 완전한 정지 피드백을 받는다.
        s.rx.extend(frame(left=-3,right=3)[9:]+frame())
        k.now+=.02;c.tick(k.ros)
        self.assertIsNone(c.fault);self.assertEqual(c.feedback['m1_rpm'],0)
        self.assertFalse(c.armed)
        self.assertTrue(any(e['event']=='rx' and e['size']>0 and e['hex'] for e in c.io_history))

    def test_disarm_truncated_real_reply_retries_zero_without_delaying_first_stop(self):
        c,s,k=self.setup_core();self.arm(c,k);self.cmd(c,k);self.pump(c,k,.2,refresh=True)
        s.autoreply=False;s.rx.clear();count=c.valid_count;old=c.epoch
        c.disarm();k.now+=.02;c.tick(k.ros)
        first=c.waiting_since;first_index=len(s.sent)-1
        self.assertEqual(rpms(s.sent[-1]),(0,0));self.assertFalse(c.armed)
        # 실제 로그의 17바이트 응답을 그대로 사용한다.
        s.rx.extend(bytes.fromhex('b8 b7 01 d8 0e f9 ff 00 e7 fe ff ff 07 00 00 1b 81'))
        k.now+=.02;c.tick(k.ros)
        self.assertEqual(c.valid_count,count);self.assertEqual(len(c.buffer),17)
        s.autoreply=True
        for _ in range(7):k.now+=.02;c.tick(k.ros)
        self.assertIsNone(c.fault);self.assertLess(k.now-first,.3)
        self.assertGreater(c.valid_count,count)
        self.assertEqual((c.feedback['m1_rpm'],c.feedback['m2_rpm']),(0,0))
        self.assertTrue(all(p[3]==207 and rpms(p)==(0,0) for p in s.sent[first_index:]))
        self.assertFalse(c.armed);self.assertNotEqual(c.epoch,old)
        self.assertFalse(self.cmd(c,k,session=old))

    def test_disarm_retry_never_extends_original_response_deadline(self):
        c,s,k=self.setup_core();self.arm(c,k);self.cmd(c,k);self.pump(c,k,.2,refresh=True)
        s.autoreply=False;s.rx.clear();c.disarm();k.now+=.02;c.tick(k.ros)
        first=c.waiting_since;start=len(s.sent)-1
        for _ in range(14):
            k.now+=.02;c.tick(k.ros)
            self.assertEqual(c.waiting_since,first)
            self.assertIsNone(c.fault)
        self.assertEqual(len(s.sent[start:]),3)
        self.assertTrue(all(rpms(p)==(0,0) for p in s.sent[start:]))
        k.now+=.04;c.tick(k.ros)
        self.assertEqual(c.fault,'RESPONSE_TIMEOUT')
        self.assertGreaterEqual(c.last_response_error['waiting_s'],.3)
        self.assertFalse(c.last_response_error['execution_delay_exceeded_timeout'])
        self.assertFalse(c.armed)

    def test_deadline_preserving_retry_rejects_nonzero_packet(self):
        c,s,k=self.setup_core()
        before=len(s.sent)
        with self.assertRaises(ValueError):
            c._transmit(velocity_packet(-3,3),k.now,preempt=True,preserve_response_deadline=True)
        self.assertEqual(len(s.sent),before)

    def test_feedback_loss_stops_even_with_fresh_commands(self):
        c,s,k=self.setup_core();self.arm(c,k);self.cmd(c,k);self.pump(c,k,.2,refresh=True)
        s.autoreply=False;s.rx.clear();self.pump(c,k,.5,refresh=True)
        self.assertFalse(c.armed);self.assertIn(c.fault,('RESPONSE_TIMEOUT','FEEDBACK_STALE'))
        self.assertEqual(rpms([p for p in s.sent if p[3]==207][-1]),(0,0))

    def test_fault_does_not_auto_resume(self):
        c,s,k=self.setup_core();session=self.arm(c,k);self.cmd(c,k);self.pump(c,k,.5)
        self.assertFalse(self.cmd(c,k,session=session))
        self.pump(c,k,.3)
        self.assertFalse(c.armed);self.assertIsNotNone(c.fault)
        new=self.arm(c,k);self.assertNotEqual(new,session);self.assertIsNone(c.command)

    def test_invalid_commands_latch(self):
        for v,w,frame_name in [(float('nan'),0.,'follower/base_link'),(.02,float('inf'),'follower/base_link'),
                                (-.01,0.,'follower/base_link'),(.02,0.,'base_link')]:
            c,s,k=self.setup_core();self.arm(c,k)
            ok=c.accept_command(v,w,frame=frame_name,stamp_ns=k.ros,ros_now_ns=k.ros,epoch=c.epoch)
            self.assertFalse(ok);self.assertEqual(c.fault,'INVALID_COMMAND')

    def test_invalid_command_timestamps(self):
        for delta in [-1_000_000_000,1, None]:
            c,s,k=self.setup_core();self.arm(c,k)
            stamp=0 if delta is None else k.ros+delta
            self.assertFalse(c.accept_command(.02,0.,frame='follower/base_link',stamp_ns=stamp,
                                             ros_now_ns=k.ros,epoch=c.epoch))
            self.assertFalse(c.armed)

    def test_replayed_command_rejected(self):
        c,s,k=self.setup_core();self.arm(c,k);self.assertTrue(self.cmd(c,k))
        self.assertFalse(self.cmd(c,k));self.assertEqual(c.fault,'INVALID_COMMAND')

    def test_disarm_invalidates_old_commands(self):
        c,s,k=self.setup_core();session=self.arm(c,k);self.cmd(c,k);c.disarm()
        self.assertFalse(self.cmd(c,k,session=session));self.assertNotEqual(c.epoch,session)

    def test_partial_frames(self):
        s=FakeSerial();s.autoreply=False;k=Clock();c=MotorCore(s,dry_run=False,clock=k)
        c.tick(k.ros);raw=frame();s.rx.extend(raw[:9]);k.now+=.02;c.tick(k.ros)
        self.assertEqual(c.valid_count,0)
        s.rx.extend(raw[9:]);k.now+=.02;c.tick(k.ros)
        self.assertEqual(c.valid_count,1)

    def test_noise_bad_checksum_then_valid(self):
        s=FakeSerial();s.autoreply=False;k=Clock();c=MotorCore(s,dry_run=False,clock=k)
        c.tick(k.ros);bad=bytearray(frame());bad[6]^=1
        s.rx.extend(b'noise'+bad+frame());k.now+=.02;c.tick(k.ros)
        self.assertEqual(c.valid_count,1)

    def test_duplicate_reply_does_not_refresh_twice(self):
        s=FakeSerial();s.autoreply=False;k=Clock();c=MotorCore(s,dry_run=False,clock=k)
        c.tick(k.ros);s.rx.extend(frame()+frame());k.now+=.02;c.tick(k.ros)
        self.assertEqual(c.valid_count,1);self.assertEqual(c.discarded_count,1)

    def test_timeout_checked_before_queued_reply(self):
        s=FakeSerial();s.autoreply=False;k=Clock();c=MotorCore(s,dry_run=False,clock=k)
        c.tick(k.ros);s.rx.extend(frame());k.now+=.31;c.tick(k.ros)
        self.assertEqual(c.valid_count,0);self.assertEqual(c.fault,'RESPONSE_TIMEOUT')
        self.assertTrue(c.last_response_error['execution_delay_exceeded_timeout'])

    def test_partial_response_timeout_keeps_raw_evidence(self):
        s=FakeSerial();s.autoreply=False;k=Clock();c=MotorCore(s,dry_run=False,clock=k)
        c.tick(k.ros);partial=frame()[:-1];s.rx.extend(partial)
        k.now+=.02;c.tick(k.ros)
        for _ in range(16):k.now+=.02;c.tick(k.ros)
        self.assertEqual(c.fault,'RESPONSE_TIMEOUT')
        self.assertEqual(c.last_response_error['buffered_hex'],partial.hex(' '))
        self.assertFalse(c.last_response_error['execution_delay_exceeded_timeout'])
        self.assertEqual(c.valid_count,0)

    def test_unsolicited_reply_not_current_feedback(self):
        c,s,k=self.setup_core();count=c.valid_count;s.rx.extend(frame());k.now+=.02;c.tick(k.ros)
        self.assertEqual(c.valid_count,count);self.assertGreater(c.discarded_count,0)

    def test_nondefault_id_and_22byte_reply(self):
        k=Clock();s=FakeSerial();s.reply=frame(driver_id=2,length=16)
        c=MotorCore(s,dry_run=False,config=replace(Config(),driver_id=2),clock=k)
        c.tick(k.ros);k.now+=.02;c.tick(k.ros)
        self.assertEqual(c.valid_count,1);self.assertEqual(s.sent[0][2],2)

    def test_wrong_id_does_not_refresh(self):
        s=FakeSerial();s.reply=frame(driver_id=2);k=Clock();c=MotorCore(s,dry_run=False,clock=k)
        c.tick(k.ros);k.now+=.02;c.tick(k.ros)
        self.assertIsNone(c.last_feedback)

    def test_implausible_rpm_does_not_refresh(self):
        s=FakeSerial();s.reply=frame(left=1000);k=Clock();c=MotorCore(s,dry_run=False,clock=k)
        c.tick(k.ros);k.now+=.02;c.tick(k.ros)
        self.assertIsNone(c.last_feedback);self.assertEqual(c.fault,'IMPLAUSIBLE_RPM')

    def test_partial_write_fault_bounded_stop_attempts(self):
        k=Clock();s=FakeSerial();s.partial=True;c=MotorCore(s,dry_run=False,clock=k)
        c.tick(k.ros);self.assertTrue(c.io_failed);self.assertEqual(len(s.sent),4)
        self.assertEqual(c.fault,'SERIAL_WRITE_ERROR')
        c.tick(k.ros);self.assertEqual(len(s.sent),4)

    def test_read_exception_attempts_zero_without_fake_feedback(self):
        c,s,k=self.setup_core();s.reply=frame(left=-3,right=3);self.pump(c,k,.2)
        previous=c.feedback.copy();self.assertNotEqual(previous['m1_rpm'],0);s.read_error=True
        k.now+=.02;c.tick(k.ros)
        self.assertEqual(c.fault,'SERIAL_READ_ERROR');self.assertEqual(c.feedback,previous)
        self.assertTrue(all(rpms(p)==(0,0) for p in s.sent[-3:]))

    def test_write_exception_latches(self):
        s=FakeSerial();s.write_error=True;k=Clock();c=MotorCore(s,dry_run=False,clock=k)
        c.tick(k.ros);self.assertEqual(c.fault,'SERIAL_WRITE_ERROR');self.assertFalse(c.armed)

    def test_measured_rpm_drives_odom_with_left_sign(self):
        c,s,k=self.setup_core();s.reply=frame(left=-3,right=3)
        self.pump(c,k,.3)
        self.assertGreater(c.odom_x,0.);self.assertAlmostEqual(c.odom_w,0.)
        self.assertEqual(c.feedback['m1_rpm'],-3)

    def test_long_feedback_gap_not_integrated(self):
        c,s,k=self.setup_core();s.reply=frame(left=-3,right=3)
        x=c.odom_x;k.now+=.5;c.tick(k.ros);k.now+=.02;c.tick(k.ros)
        self.assertEqual(c.odom_x,x);self.assertGreater(c.integration_skips,0)

    def test_shutdown_zero_attempts_and_close_once(self):
        c,s,k=self.setup_core();count=len(s.sent);c.shutdown()
        self.assertEqual(len(s.sent)-count,3);self.assertTrue(s.closed)
        self.assertTrue(all(rpms(p)==(0,0) for p in s.sent[-3:]))
        count=len(s.sent);c.shutdown();c.tick(k.ros);self.assertEqual(len(s.sent),count)

    def test_poll_does_not_exceed_rate(self):
        c,s,k=self.setup_core();self.pump(c,k,1.)
        self.assertLessEqual(len(s.sent),11)

    def test_waiting_request_not_overlapped(self):
        s=FakeSerial();s.autoreply=False;k=Clock();c=MotorCore(s,dry_run=False,clock=k)
        c.tick(k.ros);self.pump(c,k,.2)
        self.assertEqual(len(s.sent),1)

    def test_monotonic_clock_jump_disarms(self):
        c,s,k=self.setup_core();self.arm(c,k);k.now-=.1;c.tick(k.ros)
        self.assertFalse(c.armed);self.assertEqual(c.fault,'MONOTONIC_CLOCK_BACKWARDS')

    def test_ros_clock_jump_expires_old_command(self):
        c,s,k=self.setup_core();self.arm(c,k);self.cmd(c,k);self.pump(c,k,.11,refresh=True)
        c.tick(k.ros+2_000_000_000)
        self.assertFalse(c.armed);self.assertEqual(c.fault,'COMMAND_TIMEOUT')
        self.assertEqual(rpms(s.sent[-1]),(0,0))

    def test_no_new_feedback_never_fakes_zero_rpm(self):
        c,s,k=self.setup_core();s.reply=frame(left=-3,right=3);self.pump(c,k,.2)
        previous=c.feedback.copy();s.autoreply=False;s.rx.clear();self.pump(c,k,.5)
        self.assertEqual(c.feedback,previous);self.assertFalse(c.feedback_fresh())

    def test_randomized_lock_fault_and_command_cap_invariants(self):
        rng=random.Random(20261028)
        c,s,k=self.setup_core()
        for _ in range(500):
            k.now+=.02
            action=rng.randrange(7)
            if action==0:c.arm(c.epoch,k.ros)
            elif action==1:c.disarm()
            elif action in (2,3):self.cmd(c,k,v=rng.uniform(0,.1),w=rng.uniform(-.2,.2))
            elif action==4:c.latch_fault('INJECTED_FAULT')
            elif action==5:s.autoreply=not s.autoreply
            begin=len(s.sent);c.tick(k.ros)
            for packet in s.sent[begin:]:
                self.assertEqual(sum(packet)%256,0)
                if packet[3]!=207:continue
                wheels=rpms(packet);self.assertLessEqual(max(map(abs,wheels)),3)
                if wheels!=(0,0):
                    self.assertTrue(c.armed);self.assertIsNone(c.fault)
                    self.assertTrue(c.feedback_fresh());self.assertIsNotNone(c.command)


if __name__=='__main__':unittest.main(verbosity=2)
