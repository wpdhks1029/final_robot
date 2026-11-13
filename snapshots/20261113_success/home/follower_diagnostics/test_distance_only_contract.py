import json,unittest
from test_local_avoidance_drive import Tests as Setup
from local_avoidance_drive import LocalAvoidanceContract,PivotDrivePort
from test_motor_candidate import Clock
from follower_motor_mock import MemoryTransport

class Tests(unittest.TestCase):
    def setup(self):
        c=Clock();p=PivotDrivePort(MemoryTransport(),c,max_rpm=10)
        k=LocalAvoidanceContract(p,enabled=True,clock=c,max_rpm=10,trial_seconds=60,distance_test_only=True)
        for _ in range(75):k.tick(c.ros);c.now+=.01
        self.assertIsNotNone(k.arm(c.ros));c.now+=.01
        return c,k
    def test_actual_low_clearance_accepted_only_in_dedicated_mode(self):
        for special in (False,True):
            c,k=self.setup();k.distance_test_only=special
            o=Setup().envelope(c,k,'DRIVE',.04,0.);o.update(clearance_m=.24,swept_clearance_m=.22)
            self.assertEqual(k.command(json.dumps(o),c.ros),special)
    def test_yaw_reverse_and_excess_speed_still_fault(self):
        for motion,v,w in [('TURN',0.,.1),('DRIVE',-.02,0.),('DRIVE',.05,0.),('DRIVE',.03,.01)]:
            c,k=self.setup();o=Setup().envelope(c,k,motion,v,w)
            self.assertFalse(k.command(json.dumps(o),c.ros));self.assertIsNotNone(k.core.fault)
    def test_independent_distance_and_time_caps(self):
        c,k=self.setup();self.assertLessEqual(k.deadline-c.now,45.)
        k.core.odom_x=1.21;k.tick(c.ros)
        self.assertEqual(k.core.fault,'DISTANCE_TEST_1_2M_LIMIT')

if __name__=='__main__':unittest.main()
