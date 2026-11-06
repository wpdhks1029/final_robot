import ast,math,unittest
from pathlib import Path
from run_leader_yaw_trial import request

class Tests(unittest.TestCase):
    def test_blocking_scan_index_transform_and_all_raw_ranges_are_retained(self):
        from types import SimpleNamespace as S
        from run_leader_yaw_trial import scan_evidence,pack_scan
        m=S(header=S(stamp=S(sec=1,nanosec=0),frame_id='laser'),ranges=[1.,.4,float('inf'),0.],
            angle_min=0.,angle_increment=math.pi/2,range_min=.1,range_max=10.)
        tf=S(transform=S(translation=S(x=.2,y=0.,z=.2),rotation=S(x=0.,y=0.,z=0.,w=1.)))
        d=scan_evidence(m,tf);self.assertEqual(d['nearest_points'][0]['index'],1)
        self.assertAlmostEqual(d['nearest_points'][0]['base_y_m'],.4)
        self.assertEqual(d['inside_count'],1)
        self.assertEqual(pack_scan(m)['ranges'],[1.,.4,None,0.])
    def test_both_ninety_degree_directions_stop_on_imu(self):
        for sign in (-1,1):
            w,r=request(sign*math.radians(75),sign*math.pi/2,sign*math.pi/2,15.,.5,'imu')
            self.assertEqual((w,r),(0.,'TARGET_REACHED'))
    def test_raw_reference_is_explicit_and_independent(self):
        w,r=request(math.pi/2,math.radians(80),math.pi/2,15.,.5,'raw')
        self.assertEqual((w,r),(0.,'TARGET_REACHED'))
    def test_bounded_low_speed_and_final_slowdown(self):
        self.assertEqual(request(0,0,math.pi/2,0.,.5,'imu')[0],.12)
        self.assertEqual(request(1.4,math.radians(87),math.pi/2,15.,.5,'imu')[0],.06)
    def test_independent_angle_and_time_limits(self):
        self.assertEqual(request(0,math.radians(111),math.pi/2,1.,.5,'raw')[1],'ANGLE_110_DEG_LIMIT')
        self.assertEqual(request(0,0,math.pi/2,30.,.5,'imu')[1],'TURN_TIME_LIMIT')
    def test_obstacle_and_opposite_motion_stop(self):
        self.assertEqual(request(0,0,math.pi/2,1.,.29,'imu')[0],0.)
        self.assertEqual(request(-.2,0,math.pi/2,1.,.5,'imu')[1],'OPPOSITE_ROTATION')
    def test_nonfinite_never_commands(self):
        self.assertEqual(request(float('nan'),0,math.pi/2,1.,.5,'imu')[0],0.)
    def test_no_follower_arm_and_only_leader_cmd_publisher(self):
        tree=ast.parse(Path(__file__).with_name('run_leader_yaw_trial.py').read_text())
        calls=[n for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute)]
        self.assertFalse(any(n.func.attr=='create_client' for n in calls))
        pubs=[n for n in calls if n.func.attr=='create_publisher']
        self.assertEqual(len(pubs),1);self.assertEqual(pubs[0].args[1].value,'/cmd_vel')

if __name__=='__main__':unittest.main()
