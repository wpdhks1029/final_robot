import unittest
from local_near_return_pause import NearReturnPause

def row(stamp,reason='LOCAL_AVOIDANCE_READY',clearance=.4):
    return dict(reason=reason,v=.02,w=0.,motion='DRIVE',scan_stamp_ns=stamp,
                clearance_m=clearance,swept_clearance_m=clearance-.01)

ZERO=dict(measured_rpm=[0,0],zero_feedback_samples=3)

class Tests(unittest.TestCase):
    def trigger(self,p):return p.apply(row(100,'STOP_FOOTPRINT_CLEARANCE',.05),0.,ZERO)
    def test_immediate_zero_without_altering_measured_clearance(self):
        p=NearReturnPause();r=self.trigger(p)
        self.assertEqual((r['v'],r['w'],r['motion']),(0.,0.,'STOP'))
        self.assertEqual(r['clearance_m'],.05);self.assertTrue(p.active)
    def test_repeated_same_scan_cannot_count_as_five_clear_scans(self):
        p=NearReturnPause();self.trigger(p)
        for i in range(20):r=p.apply(row(101),.01+i*.02,ZERO)
        self.assertEqual(r['near_return_pause']['clear_scans'],1);self.assertEqual(r['v'],0.)
    def test_five_unique_safe_scans_and_measured_zero_are_required(self):
        p=NearReturnPause();self.trigger(p)
        for i in range(1,6):r=p.apply(row(100+i),i*.1,dict(measured_rpm=[1,1],zero_feedback_samples=0))
        self.assertEqual(r['v'],0.)
        r=p.apply(row(106),.6,ZERO)
        self.assertEqual(r['near_return_pause']['event'],'resumed');self.assertEqual(r['v'],.02)
    def test_new_near_return_resets_clear_streak(self):
        p=NearReturnPause();self.trigger(p)
        for i in range(1,4):p.apply(row(100+i),i*.1,ZERO)
        r=p.apply(row(104,'STOP_FOOTPRINT_CLEARANCE',.05),.4,ZERO)
        self.assertEqual(r['near_return_pause']['clear_scans'],0)
        for i in range(1,5):r=p.apply(row(104+i),.4+i*.1,ZERO)
        self.assertEqual(r['v'],0.)
        self.assertEqual(p.apply(row(109),.9,ZERO)['near_return_pause']['event'],'resumed')
    def test_timeout_and_unrelated_faults_cannot_resume(self):
        p=NearReturnPause();self.trigger(p)
        self.assertEqual(p.apply(row(101,'STOP_RAW_ODOM_JUMP'),.1,ZERO)['reason'],'STOP_RAW_ODOM_JUMP')
        r=p.apply(row(102,'STOP_FOOTPRINT_CLEARANCE',.05),2.,ZERO)
        self.assertEqual(r['reason'],'STOP_NEAR_RETURN_WAIT_TIMEOUT');self.assertEqual(r['v'],0.)
    def test_clearance_below_resume_margin_does_not_accumulate(self):
        p=NearReturnPause();self.trigger(p)
        for i in range(1,10):r=p.apply(row(100+i,clearance=.34),i*.1,ZERO)
        self.assertEqual(r['near_return_pause']['clear_scans'],0);self.assertEqual(r['v'],0.)

if __name__=='__main__':unittest.main()
