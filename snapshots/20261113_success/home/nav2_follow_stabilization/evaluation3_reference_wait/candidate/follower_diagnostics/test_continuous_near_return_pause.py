import unittest
from continuous_near_return_pause import ContinuousNearReturnPause

class Tests(unittest.TestCase):
    def row(self,stamp=1,reason='CONTINUOUS_READY',gap=.25):
        return dict(reason=reason,scan_stamp_ns=stamp,v=.025,w=.02,motion='DRIVE',clearance_m=gap,swept_clearance_m=gap)
    def motor(self):return dict(armed=True,feedback_fresh=True,measured_rpm=[0,0],zero_feedback_samples=3)
    def start(self,p):
        r=p.apply(self.row(reason='STOP_FOOTPRINT_CLEARANCE',gap=.06),0.,self.motor())
        self.assertEqual((r['v'],r['w'],r['motion']),(0.,0.,'STOP'));return r
    def test_five_new_safe_scans_after_stationary_then_next_tick_recheck(self):
        p=ContinuousNearReturnPause();self.start(p)
        for i in range(2,7):
            r=p.apply(self.row(i),i*.1,self.motor());self.assertEqual(r['motion'],'STOP')
        self.assertEqual(r['reason'],'CONTINUOUS_RECHECK_COMPLETE');self.assertFalse(p.active)
        self.assertEqual(p.apply(self.row(7),.7,self.motor())['reason'],'CONTINUOUS_READY')
    def test_follow_wait_while_leader_stopped_can_finish_zero_reobservation(self):
        p=ContinuousNearReturnPause();self.start(p)
        for i in range(2,7):r=p.apply(self.row(i,'FOLLOW_WAITING_LEADER'),i*.1,self.motor())
        self.assertEqual(r['reason'],'CONTINUOUS_RECHECK_COMPLETE');self.assertEqual(r['v'],0.)

    def test_duplicate_scans_or_moving_wheels_cannot_resume(self):
        p=ContinuousNearReturnPause();self.start(p)
        for i in range(8):p.apply(self.row(2),.1+i*.1,self.motor())
        self.assertTrue(p.active);self.assertEqual(p.clear_scans,1)
        m=self.motor();m['measured_rpm']=[1,1]
        for i in range(3,10):p.apply(self.row(i),1.,m)
        self.assertTrue(p.active);self.assertEqual(p.clear_scans,0)
    def test_persistent_point_times_out_and_unrelated_stop_is_preserved(self):
        p=ContinuousNearReturnPause();self.start(p)
        r=p.apply(self.row(2,'STOP_FOOTPRINT_CLEARANCE',.06),2.,self.motor())
        self.assertEqual(r['reason'],'STOP_NEAR_RETURN_WAIT_TIMEOUT')
        r=p.apply(self.row(3,'STOP_APF_WORKSPACE_BOUND'),2.1,self.motor())
        self.assertEqual(r['reason'],'STOP_APF_WORKSPACE_BOUND')
    def test_more_than_two_episodes_aborts(self):
        p=ContinuousNearReturnPause()
        for episode in range(2):
            self.start(p)
            for i in range(2,7):p.apply(self.row(i),i*.1,self.motor())
        self.assertEqual(p.apply(self.row(8,'STOP_FOOTPRINT_CLEARANCE',.06),0.,self.motor())['reason'],'STOP_NEAR_RETURN_REPEAT_LIMIT')
    def test_unsafe_sweep_resets_clear_scans(self):
        p=ContinuousNearReturnPause();self.start(p)
        for i in range(2,5):p.apply(self.row(i),i*.1,self.motor())
        r=self.row(5);r['swept_clearance_m']=.09
        p.apply(r,.5,self.motor());self.assertEqual(p.clear_scans,0)

if __name__=='__main__':unittest.main()
