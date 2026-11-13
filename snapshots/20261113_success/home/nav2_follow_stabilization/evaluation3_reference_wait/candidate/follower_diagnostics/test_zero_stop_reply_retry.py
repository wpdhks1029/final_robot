"""Clock/byte transport tests only; no serial hardware or ROS."""
import struct,unittest
from follower_motor_candidate import MotorCore,velocity_packet
from follower_motor_mock import MemoryTransport
from test_motor_candidate import Clock,frame

class DroppedZeroReply(MemoryTransport):
    def __init__(self):super().__init__();self.drop=0
    def write(self,p):
        n=super().write(p)
        if len(p)==13 and p[3]==207 and struct.unpack_from('<h',p,6)[0]==struct.unpack_from('<h',p,9)[0]==0 and self.drop:
            self.drop-=1;self.rx.clear()
        return n

class Tests(unittest.TestCase):
    def setup(self,enabled=True):
        clock=Clock();port=DroppedZeroReply();core=MotorCore(port,dry_run=False,allow_motion=True,clock=clock)
        core.zero_stop_reply_retry_enabled=enabled
        for _ in range(30):core.tick(clock.ros);clock.now+=.02
        self.assertIsNotNone(core.arm(core.epoch,clock.ros))
        for _ in range(12):self.command(core,clock,.02);core.tick(clock.ros);clock.now+=.02
        return core,port,clock
    def command(self,core,clock,v=0.):
        return core.accept_command(v,0.,frame='follower/base_link',stamp_ns=clock.ros,ros_now_ns=clock.ros,epoch=core.epoch)
    def stop(self,core,port,clock,drop):
        port.drop=drop;self.assertTrue(self.command(core,clock));core.tick(clock.ros)
        self.assertEqual(core.last_sent,(0,0));return core.waiting_since
    def test_one_lost_zero_reply_is_retried_once_without_deadline_extension(self):
        c,p,k=self.setup();first=self.stop(c,p,k,1);seen=False
        for _ in range(20):
            k.now+=.02;self.command(c,k);c.tick(k.ros)
            if c.zero_wait_retry_used and c.zero_wait_retry_count==1 and c.waiting_since==first:seen=True
        self.assertTrue(seen);self.assertTrue(c.armed);self.assertIsNone(c.fault)
        self.assertEqual(c.zero_wait_retry_count,1);self.assertEqual(p.rpm,(0,0));self.assertTrue(c.feedback_fresh())
    def test_persistent_loss_still_faults_at_original_deadline_and_never_rearms(self):
        c,p,k=self.setup();first=self.stop(c,p,k,100)
        while c.armed:
            k.now+=.02;self.command(c,k);c.tick(k.ros)
        self.assertLessEqual(k.now-first,.34);self.assertEqual(c.zero_wait_retry_count,1)
        self.assertIn(c.fault,('RESPONSE_TIMEOUT','FEEDBACK_STALE'))
        oldfault=c.fault;p.drop=0
        for _ in range(30):k.now+=.02;c.tick(k.ros)
        self.assertFalse(c.armed);self.assertEqual(c.fault,oldfault);self.assertEqual(p.rpm,(0,0))
    def test_partial_reply_is_not_flushed_for_an_armed_retry(self):
        c,p,k=self.setup();first=self.stop(c,p,k,100);raw=frame()
        for _ in range(7):k.now+=.02;self.command(c,k);c.tick(k.ros)
        p.rx.extend(raw[:10]);k.now+=.02;self.command(c,k);c.tick(k.ros)
        for _ in range(2):k.now+=.02;self.command(c,k);c.tick(k.ros)
        self.assertEqual(c.zero_wait_retry_count,0);self.assertEqual(bytes(c.buffer),raw[:10])
        p.rx.extend(raw[10:]);k.now+=.02;self.command(c,k);c.tick(k.ros)
        self.assertTrue(c.feedback_fresh());self.assertIsNone(c.fault)
    def test_actual_stalled_19_byte_prefix_is_not_feedback_and_recovers_on_zero_retry(self):
        c,p,k=self.setup();first=self.stop(c,p,k,1);c.last_feedback=first-.08;count=c.valid_count
        prefix=bytes.fromhex('b8 b7 01 d8 0e fa ff 00 f9 00 00 00 04 00 00 f2 02 00 00')
        p.rx.extend(prefix)
        for _ in range(3):
            k.now+=.02;self.command(c,k);c.tick(k.ros);self.assertEqual(c.valid_count,count)
        for _ in range(12):k.now+=.02;self.command(c,k);c.tick(k.ros)
        self.assertTrue(c.armed);self.assertIsNone(c.fault);self.assertEqual(c.zero_wait_retry_count,1)
        self.assertEqual(c.zero_retry_discarded_bytes,19);self.assertEqual(p.rpm,(0,0))
    def test_stalled_prefix_without_new_reply_still_faults_and_is_never_accepted(self):
        c,p,k=self.setup();first=self.stop(c,p,k,100);count=c.valid_count
        p.rx.extend(bytes.fromhex('b8 b7 01 d8 0e fa ff 00 f9 00 00 00 04 00 00 f2 02 00 00'))
        while c.armed:k.now+=.02;self.command(c,k);c.tick(k.ros)
        self.assertEqual(c.valid_count,count);self.assertEqual(c.zero_retry_discarded_bytes,19)
        self.assertEqual(c.zero_wait_retry_count,1);self.assertLessEqual(k.now-first,.34)
    def test_nonzero_commands_and_default_profile_never_enable_retry(self):
        c,p,k=self.setup();p.feedback_enabled=False;p.rx.clear()
        for _ in range(20):k.now+=.02;self.command(c,k,.02);c.tick(k.ros)
        self.assertFalse(c.armed);self.assertEqual(c.zero_wait_retry_count,0)
        c,p,k=self.setup(enabled=False);self.stop(c,p,k,100)
        for _ in range(20):k.now+=.02;self.command(c,k);c.tick(k.ros)
        self.assertFalse(c.armed);self.assertEqual(c.zero_wait_retry_count,0)
    def test_armed_nonzero_context_cannot_use_zero_deadline_preservation(self):
        c,p,k=self.setup();self.command(c,k,.02)
        with self.assertRaises(ValueError):c._transmit(velocity_packet(0,0),k.now,preempt=True,preserve_response_deadline=True,preserve_input=True)
    def test_feedback_timeout_before_response_timeout_has_its_own_trace(self):
        c,p,k=self.setup();p.rx.clear();c.last_feedback=k.now-.11
        first=self.stop(c,p,k,100)
        while c.armed:k.now+=.02;self.command(c,k);c.tick(k.ros)
        self.assertEqual(c.fault,'FEEDBACK_STALE');self.assertIsNone(c.last_response_error)
        e=c.last_feedback_error
        self.assertGreater(e['feedback_age_s'],.3);self.assertLess(e['response_waiting_s'],.3)
        self.assertLess(e['tick_gap_s'],.03);self.assertEqual(e['last_sent'],[0,0])
        self.assertEqual(e['zero_retry_count'],1);self.assertTrue(e['history'])
    def test_two_zero_reply_frames_do_not_count_as_two_fresh_transactions(self):
        c,p,k=self.setup();p.rx.clear();c.last_feedback=k.now-.11;self.stop(c,p,k,100)
        while not c.zero_wait_retry_used:k.now+=.02;self.command(c,k);c.tick(k.ros)
        before=c.valid_count;p.rx.extend(frame()+frame());k.now+=.02;self.command(c,k);c.tick(k.ros)
        self.assertEqual(c.valid_count,before+1);self.assertTrue(c.feedback_fresh())
    def test_local_phase_contract_recovers_and_still_requires_measured_zero(self):
        import json
        from local_avoidance_drive import LocalAvoidanceContract,PivotDrivePort,PROFILE
        k=Clock();p=DroppedZeroReply();port=PivotDrivePort(p,k,max_rpm=10)
        contract=LocalAvoidanceContract(port,enabled=True,clock=k,max_rpm=10,trial_seconds=30)
        for _ in range(50):contract.tick(k.ros);k.now+=.02
        self.assertIsNotNone(contract.arm(k.ros))
        def command(motion,v,w):
            return contract.command(json.dumps(dict(version=3,controller=PROFILE,epoch=contract.core.epoch,
                frame='follower/base_link',stamp_ns=k.ros,source_stamp_ns=k.ros,scan_stamp_ns=k.ros,
                motion=motion,v=v,w=w,clearance_m=.5,swept_clearance_m=.5)),k.ros)
        for _ in range(15):
            self.assertTrue(command('TURN',0.,.12));contract.tick(k.ros);k.now+=.02
        p.drop=1
        for _ in range(35):
            self.assertTrue(command('STOP',0.,0.));contract.tick(k.ros);k.now+=.02
        self.assertIsNone(contract.core.fault);self.assertEqual(contract.core.zero_wait_retry_count,1)
        self.assertGreaterEqual(contract.zero_samples,3);self.assertEqual(p.rpm,(0,0))
        self.assertTrue(command('DRIVE',.04,0.));self.assertEqual(contract.zero_samples,0)

if __name__=='__main__':unittest.main()
