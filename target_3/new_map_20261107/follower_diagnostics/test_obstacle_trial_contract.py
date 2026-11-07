import copy
import unittest
from run_joint_follow_trial import obstacle_stop_evidence

class Tests(unittest.TestCase):
    def sample(self):
        return dict(version=1,stamp_ns=100_050_000_000,source_stamp_ns=99_900_000_000,
                    scan_source_stamp_ns=99_950_000_000,stamped_reason='STOP_FOOTPRINT_CLEARANCE',
                    clearance_zone='stop',clearance_m=.25,output_v=0.,output_w=0.,
                    nearest_points=[dict(base_x_m=.52,base_y_m=.1,clearance_m=.25)])
    def evidence(self,d,**kw):
        args=dict(rx_mono=10.02,failure_ros=100_000_000_000,failure_mono=10.,
                  guard=dict(frame='follower/base_link',source_ns=99_900_000_000,zero=True))
        args.update(kw)
        return obstacle_stop_evidence(d,**args)
    def test_fresh_same_source_front_stop_is_evidence(self):
        self.assertEqual(self.evidence(self.sample())['clearance_m'],.25)
    def test_side_or_inside_footprint_is_not_front_stop_pass(self):
        for x,y,d,zone in [(.1,.55,.25,'stop'),(.1,.1,0.,'inside')]:
            m=self.sample();m.update(clearance_m=d,clearance_zone=zone)
            m['nearest_points']=[dict(base_x_m=x,base_y_m=y,clearance_m=d)]
            self.assertIsNone(self.evidence(m))
    def test_stale_scan_source_or_diagnostic_is_rejected(self):
        for field,value in [('scan_source_stamp_ns',99_000_000_000),('source_stamp_ns',100_100_000_000),
                            ('stamp_ns',99_000_000_000),('source_stamp_ns',0)]:
            m=self.sample();m[field]=value;self.assertIsNone(self.evidence(m))
        self.assertIsNone(self.evidence(self.sample(),rx_mono=11.))
    def test_upstream_or_timing_stop_cannot_pass_as_obstacle(self):
        for reason in ('STOP_UPSTREAM','STOP_SOURCE_STALE','STOP_SCAN_STALE_OR_INVALID'):
            m=self.sample();m['stamped_reason']=reason;self.assertIsNone(self.evidence(m))
    def test_nonzero_or_wrong_guard_is_rejected(self):
        for field,value in [('zero',False),('frame','other'),('source_ns',99_800_000_000)]:
            guard=dict(frame='follower/base_link',source_ns=99_900_000_000,zero=True);guard[field]=value
            self.assertIsNone(self.evidence(self.sample(),guard=guard))
        m=self.sample();m['output_v']=.01;self.assertIsNone(self.evidence(m))

if __name__=='__main__':unittest.main()
