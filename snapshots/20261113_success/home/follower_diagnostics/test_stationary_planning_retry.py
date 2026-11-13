import unittest
from stationary_planning_retry import StationaryPlanningRetry

class Tests(unittest.TestCase):
    def motor(self):return dict(armed=True,feedback_fresh=True,measured_rpm=[0,0],zero_feedback_samples=3)
    def scans(self,g,t=2.,start=1,motor=None,leader=0.):
        result=False
        for i in range(start,start+5):result=g.ready(t,i,motor or self.motor(),leader)
        return result
    def test_wait_then_five_distinct_stopped_scans(self):
        g=StationaryPlanningRetry(0.)
        self.assertFalse(self.scans(g,t=1.));self.assertTrue(g.ready(1.5,6,self.motor(),0.))
    def test_both_robots_must_be_measured_stopped(self):
        g=StationaryPlanningRetry(0.)
        self.assertFalse(self.scans(g,leader=.02))
        m=self.motor();m['measured_rpm']=[1,0]
        self.assertFalse(self.scans(g,start=10,motor=m))
        m=self.motor();m['feedback_fresh']=False
        self.assertFalse(self.scans(g,start=20,motor=m))
        self.assertTrue(self.scans(g,start=30))
    def test_duplicates_cannot_satisfy_new_snapshot_requirement(self):
        g=StationaryPlanningRetry(0.)
        for _ in range(20):self.assertFalse(g.ready(2.,1,self.motor(),0.))
    def test_three_attempt_limit_and_fresh_scan_after_failure(self):
        g=StationaryPlanningRetry(0.)
        for attempt in range(3):
            self.assertTrue(self.scans(g,t=attempt*2+2,start=attempt*10+1));g.submitted()
            retry=g.retry('NO_VALIDATED_CONTINUOUS_ROUTE',attempt*2+2,attempt*10+5)
            self.assertEqual(retry,attempt<2)
            if retry:self.assertFalse(g.ready(attempt*2+3,attempt*10+5,self.motor(),0.))
    def test_clear_front_requires_five_distinct_scans_and_resets_on_blocker(self):
        g=StationaryPlanningRetry(0.)
        for stamp in range(1,6):g.ready(2.,stamp,self.motor(),0.,3)
        self.assertEqual(g.clear_scans,5)
        g.ready(2.,6,self.motor(),0.,9);self.assertEqual(g.clear_scans,0)
        g.submitted();self.assertTrue(g.retry('NO_OBSTACLE_CLUSTER',2.,6))

    def test_other_failures_cannot_be_retried(self):
        for reason in ('FILTERED_STALE','CONTINUOUS_PLANNING_BUDGET_EXCEEDED','MOTOR_FAULT'):
            self.assertFalse(StationaryPlanningRetry(0.).retry(reason,2.,1))

if __name__=='__main__':unittest.main()
