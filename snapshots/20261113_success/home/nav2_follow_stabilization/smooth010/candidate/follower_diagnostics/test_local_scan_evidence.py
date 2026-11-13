import math,unittest
from types import SimpleNamespace as S
from local_scan_evidence import scan_evidence

def message(ns):
    return S(header=S(stamp=S(sec=ns,nanosec=0),frame_id='follower/laser'),angle_min=-math.pi,
             angle_max=math.pi,angle_increment=.1,time_increment=.001,scan_time=.1,
             range_min=.05,range_max=12.,ranges=[.327,.345,float('inf')],intensities=[3.,4.,0.])

class Tests(unittest.TestCase):
    def test_preserves_near_pair_neighbors_timing_and_before_after(self):
        a,b,c=message(1),message(2),message(3)
        output=scan_evidence([(a,11),(b,12)],[(b,13),(c,14)],[(1,1.,(0,0,0,0,0))])
        self.assertEqual(len(output['frames']),3)
        self.assertEqual(output['frames'][1]['rx_ros_ns'],12)
        self.assertEqual(output['frames'][1]['ranges'],[.327,.345,None])
        self.assertEqual(output['frames'][1]['intensities'],[3.,4.,0.])
        self.assertFalse(output['control_input_changed']);self.assertTrue(output['raw_odometry_history'])
        self.assertTrue(math.isinf(b.ranges[2]))
    def test_filtered_history_is_labelled_separately_from_raw_monitor(self):
        filtered=[(1,1.,(0,0,.5,0,0))];raw=[(1,1.,(0,0,.4,0,0))]
        d=scan_evidence([],[],filtered,pose_source='filtered',raw_monitor_history=raw)
        self.assertEqual(d['pose_odometry_history'],filtered);self.assertEqual(d['raw_odometry_history'],raw)

if __name__=='__main__':unittest.main()
