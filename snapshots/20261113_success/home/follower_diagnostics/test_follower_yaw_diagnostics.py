import math,unittest
from types import SimpleNamespace as S
from follower_yaw_diagnostics import rpy,synchronized_summary,pivot_request,rotation_circle_diagnostic

class Tests(unittest.TestCase):
    def streams(self):
        return {k:[dict(stamp_ns=100_000_000_000+i*100_000_000,yaw=math.radians(base+rate*i/10),roll=0.,pitch=0.)
                   for i in range(11)] for k,base,rate in [('raw',170,20),('imu',-90,28)]}
    def test_common_window_compares_changes_not_absolute_heading_offsets(self):
        d=synchronized_summary(self.streams())
        self.assertAlmostEqual(d['sources']['raw']['delta_deg'],20)
        self.assertAlmostEqual(d['sources']['imu']['delta_deg'],28)
        self.assertAlmostEqual(d['imu_over_raw_ratio'],1.4)
    def test_missing_initial_or_final_data_uses_intersection(self):
        s=self.streams();s['imu']=s['imu'][1:-1];d=synchronized_summary(s)
        self.assertAlmostEqual(d['duration_s'],.8)
        self.assertAlmostEqual(d['sources']['raw']['delta_deg'],16)
        self.assertAlmostEqual(d['imu_over_raw_ratio'],1.4)
    def test_large_message_gap_is_not_silently_interpolated(self):
        s=self.streams();s['imu']=s['imu'][:2]+s['imu'][8:]
        self.assertEqual(synchronized_summary(s)['reason'],'INCOMPLETE_SOURCE_WINDOW')
    def test_invalid_quaternion_rejected(self):
        with self.assertRaises(ValueError):rpy(S(x=0.,y=0.,z=0.,w=0.))
    def test_pivot_has_independent_clearance_time_imu_and_direction_stops(self):
        target=math.radians(20)
        for args,reason in [((0.,0.,target,0.,.3),'STOP_ROTATION_CIRCLE_CLEARANCE'),
                            ((0.,math.radians(45),target,0.,.6),'STOP_IMU_45_DEG_LIMIT'),
                            ((0.,0.,target,12.,.6),'STOP_12_SECOND_LIMIT'),
                            ((0.,math.radians(-6),target,0.,.6),'STOP_OPPOSITE_ROTATION')]:
            w,r=pivot_request(*args);self.assertEqual(w,0.);self.assertEqual(r,reason)
    def test_every_nonzero_request_is_executable_at_no_more_than_five_rpm(self):
        for target in (-30,-20,20,30):
            for clearance in (.301,.35,.6,1.):
                w,r=pivot_request(0.,0.,math.radians(target),0.,clearance)
                rpm=round(abs(w)*.28/.065*60/(2*math.pi))
                self.assertGreaterEqual(rpm,1);self.assertLessEqual(rpm,5)
                self.assertEqual(math.copysign(1,w),math.copysign(1,target))
    def test_circle_diagnostic_distinguishes_side_clearance_and_rotation_reserve(self):
        d=rotation_circle_diagnostic([(-.65,0.),(2.,0.)])
        self.assertGreater(d['nearest_points'][0]['footprint_clearance_m'],.30)
        self.assertLess(d['circle_clearance_m'],.30)
        self.assertAlmostEqual(d['required_center_distance_m'],math.hypot(.27,.3)+.32)
    def test_near_return_is_preserved_with_actual_laser_direction(self):
        points=[(.26,-.334),(2.,0.)];d=rotation_circle_diagnostic(points)
        self.assertAlmostEqual(d['nearest_points'][0]['laser_range_m'],.334)
        self.assertAlmostEqual(d['nearest_points'][0]['laser_angle_deg'],-90.)
        self.assertEqual(d['points_inside_required_circle'],1);self.assertEqual(len(points),2)
    def test_imu_target_does_not_wait_for_raw_angle_and_raw_cap_remains(self):
        w,r=pivot_request(math.radians(24),math.radians(30),math.radians(30),3.,.6,reference='imu')
        self.assertEqual((w,r),(0.,'IMU_TARGET_REACHED'))
        w,r=pivot_request(math.radians(40),math.radians(20),math.radians(30),3.,.6,reference='imu')
        self.assertEqual((w,r),(0.,'STOP_RAW_40_DEG_LIMIT'))

if __name__=='__main__':unittest.main()
