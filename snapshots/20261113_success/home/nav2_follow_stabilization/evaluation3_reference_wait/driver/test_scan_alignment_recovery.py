import unittest
from scan_alignment_recovery import ScanAlignmentRecovery

class Tests(unittest.TestCase):
    def motor(self,n=3,rpm=None):return dict(armed=True,feedback_fresh=True,measured_rpm=rpm or [0,0],zero_feedback_samples=n)
    def test_recorded_half_second_braking_is_not_a_data_timeout(self):
        r=ScanAlignmentRecovery();r.missing(0.)
        self.assertFalse(r.matched(.09,self.motor(0,[-3,4])))
        self.assertFalse(r.matched(.19,self.motor(0,[-2,0])))
        self.assertFalse(r.matched(.309,self.motor(1)))
        self.assertFalse(r.matched(.428,self.motor(2)))
        self.assertTrue(r.matched(.548,self.motor(3)))
    def test_missing_data_still_times_out_at_point_four(self):
        r=ScanAlignmentRecovery();r.missing(0.)
        with self.assertRaisesRegex(ValueError,'ALIGNMENT_TIMEOUT'):r.missing(.4)
    def test_braking_has_its_own_bound(self):
        r=ScanAlignmentRecovery();r.missing(0.);self.assertFalse(r.matched(.1,self.motor(0)))
        with self.assertRaisesRegex(ValueError,'BRAKE_CONFIRM_TIMEOUT'):r.matched(2.11,self.motor(0))
    def test_alternating_data_gaps_cannot_extend_total_wait_forever(self):
        r=ScanAlignmentRecovery();r.missing(0.)
        for t in (.3,.6,.9,1.2,1.5,1.8,2.1):
            self.assertFalse(r.matched(t,self.motor(0)));r.missing(t+.01)
        with self.assertRaises(ValueError):r.missing(2.41)
    def test_stale_feedback_cannot_resume(self):
        r=ScanAlignmentRecovery();r.missing(0.);m=self.motor();m['feedback_fresh']=False
        self.assertFalse(r.matched(.1,m))

if __name__=='__main__':unittest.main()
