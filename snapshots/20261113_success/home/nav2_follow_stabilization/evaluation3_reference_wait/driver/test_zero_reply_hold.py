import json,unittest
from test_zero_stop_reply_retry import DroppedZeroReply
from local_avoidance_drive import LocalAvoidanceContract,PivotDrivePort,PROFILE
from test_motor_candidate import Clock

class Tests(unittest.TestCase):
    def setup(self,sign=1):
        k=Clock();p=DroppedZeroReply();c=LocalAvoidanceContract(PivotDrivePort(p,k,max_rpm=10),enabled=True,clock=k,max_rpm=10,trial_seconds=30)
        for _ in range(40):c.tick(k.ros);k.now+=.02
        self.assertIsNotNone(c.arm(k.ros))
        for _ in range(30):k.now+=.02;self.command(c,k,.12*sign);c.tick(k.ros)
        return c,p,k
    def command(self,c,k,w):
        obj=dict(version=3,controller=PROFILE,epoch=c.core.epoch,frame='follower/base_link',stamp_ns=k.ros,source_stamp_ns=k.ros,scan_stamp_ns=k.ros,motion='TURN' if w else 'STOP',v=0.,w=w,clearance_m=.5,swept_clearance_m=.4)
        return c.command(json.dumps(obj),k.ros)
    def test_same_direction_intent_cannot_disable_zero_retry_for_nineteen_byte_reply(self):
        for sign in (-1,1):
            c,p,k=self.setup(sign);p.drop=1;k.now+=.02;self.command(c,k,0.);c.tick(k.ros)
            self.assertEqual(c.core.last_sent,(0,0));deadline=c.core.waiting_since;count=c.core.valid_count
            p.rx.extend(bytes.fromhex('b8 b7 01 d8 0e ff ff 00 7b fd ff ff fe ff 00 47 03 00 00'))
            held=False;retry=False
            for _ in range(20):
                k.now+=.02
                self.assertTrue(self.command(c,k,.12*sign),c.last_command_rejection)
                if c.zero_reply_hold_active:
                    held=True;self.assertEqual(c.core.command[:2],(0.,0.))
                c.tick(k.ros)
                if c.core.valid_count==count:self.assertEqual(p.rpm,(0,0))
                if c.core.zero_wait_retry_used and c.core.waiting_since is not None:
                    retry=True;self.assertEqual(c.core.waiting_since,deadline)
                self.assertIsNone(c.core.fault,c.status())
            self.assertTrue(held);self.assertTrue(retry);self.assertEqual(c.core.zero_wait_retry_count,1)
            self.assertEqual(c.core.zero_retry_discarded_bytes,19);self.assertTrue(c.core.feedback_fresh())
    def test_continued_partial_loss_still_faults_without_rearming(self):
        c,p,k=self.setup();p.drop=100;k.now+=.02;self.command(c,k,0.);c.tick(k.ros)
        begin=k.now;count=c.core.valid_count
        p.rx.extend(bytes.fromhex('b8 b7 01 d8 0e ff ff 00 7b fd ff ff fe ff 00 47 03 00 00'))
        while c.core.armed:
            k.now+=.02;self.command(c,k,.12);c.tick(k.ros)
        self.assertLessEqual(k.now-begin,.34);self.assertEqual(c.core.valid_count,count)
        self.assertIn(c.core.fault,('FEEDBACK_STALE','RESPONSE_TIMEOUT'));self.assertEqual(p.rpm,(0,0))
        p.drop=0
        for _ in range(20):k.now+=.02;c.tick(k.ros)
        self.assertFalse(c.core.armed)

if __name__=='__main__':unittest.main()
