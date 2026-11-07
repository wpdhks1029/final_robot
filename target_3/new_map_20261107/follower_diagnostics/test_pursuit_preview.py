"""Behavior tests for isolated preview, using fake messages and clocks only."""
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path.home() / 'robot_ws/test_tools'))
import test_follower_input_safety as fake
from follower_pursuit_preview import PreviewCore, validate_output_topic, TEST_TOPIC


class PreviewTests(unittest.TestCase):
    def make(self, points=((0.,0.), (3.,0.))):
        core = PreviewCore()
        leader = fake.odom()
        leader.pose.pose.position.x = 3.
        leader.twist.twist.linear.x = .1
        follower = fake.odom('follower/base_link')
        path = fake.S(header=fake.header(), poses=[
            fake.S(header=fake.header(), pose=fake.S(position=fake.S(x=x, y=y)))
            for x,y in points])
        core.path_callback(path, 100_100_000_000, 10.)
        core.odom_callback('leader',leader,100_100_000_000,10.)
        core.odom_callback('follower',follower,100_100_000_000,10.)
        return core,path,leader,follower

    def test_rejection_trace_preserves_future_time_and_stop(self):
        core,path,leader,follower=self.make()
        leader.header.stamp.sec=100;leader.header.stamp.nanosec=160_000_000
        core.odom_callback('leader',leader,100_100_000_000,10.)
        self.assertEqual(core.last_input_rejection['reason'],'INVALID_SOURCE_TIME')
        self.assertAlmostEqual(core.last_input_rejection['age_s'],-.06)
        self.assertNotIn('leader',core.inputs)
        self.assertEqual(self.command(core)[:2],(0.,0.))

    def test_rejection_trace_reports_short_path(self):
        core,path,leader,follower=self.make()
        path.poses=path.poses[:1];core.path_callback(path,100_100_000_000,10.)
        self.assertEqual(core.last_input_rejection['path_points'],1)
        self.assertEqual(core.last_input_rejection['reason'],'INVALID_GEOMETRY_OR_FRAME')
        self.assertNotIn('path',core.inputs)

    def command(self, core):
        return core.command(100_100_000_000,10.)

    def assert_stop(self, core, reason=None):
        v,w,why=self.command(core)
        self.assertEqual((v,w),(0.,0.))
        if reason:
            self.assertEqual(why,reason)

    def test_straight_forward(self):
        c,*_=self.make();v,w,r=self.command(c)
        self.assertEqual((v,w,r),(.1,0.,'PREVIEW_TRACK'))

    def test_left_right(self):
        for y in (1.,-1.):
            c,*_=self.make(((0.,0.),(3.,y)))
            v,w,_=self.command(c)
            self.assertGreater(v,0.);self.assertGreater(w*y,0.)
            self.assertLessEqual(abs(w),c.max_angular)

    def test_rotated_coordinates(self):
        c,_,_,f=self.make(((0.,0.),(0.,3.)))
        f.pose.pose.orientation.z=math.sin(math.pi/4)
        f.pose.pose.orientation.w=math.cos(math.pi/4)
        c.odom_callback('follower',f,100_100_000_000,10.)
        v,w,_=self.command(c)
        self.assertGreater(v,0.);self.assertAlmostEqual(w,0.)

    def test_behind_target_regression(self):
        c,*_=self.make(((-3.,0.),(-2.,0.)))
        self.assert_stop(c,'STOP_TARGET_BEHIND')

    def test_large_angle_stops(self):
        c,*_=self.make(((0.,0.),(1.,3.)))
        self.assert_stop(c,'STOP_ALIGN_REQUIRED')

    def test_stopped_reverse_and_rotating_leader(self):
        for vx,wz,reason in [(0.,0.,'STOP_LEADER_STOPPED'),(-.1,0.,'STOP_LEADER_REVERSE'),
                             (0.,.1,'STOP_LEADER_ROTATING')]:
            c,_,l,_=self.make();l.twist.twist.linear.x=vx;l.twist.twist.angular.z=wz
            c.odom_callback('leader',l,100_100_000_000,10.)
            self.assert_stop(c,reason)

    def test_observed_forward_arc_is_not_in_place_rotation(self):
        for wz in (.0676183274,-.0676183274):
            c,_,l,_=self.make()
            l.twist.twist.linear.x=.0472652548;l.twist.twist.angular.z=wz
            c.odom_callback('leader',l,100_100_000_000,10.)
            self.assertGreater(self.command(c)[0],0.)
            l.twist.twist.linear.x=.009
            c.odom_callback('leader',l,100_100_000_000,10.)
            self.assert_stop(c,'STOP_LEADER_ROTATING')
            # 제자리 회전 후에는 낮은 전진 속도만으로 자동 복귀하지 않는다.
            l.twist.twist.linear.x=.026
            c.odom_callback('leader',l,100_100_000_000,10.)
            self.assert_stop(c,'STOP_LEADER_STOPPED')

    def test_low_speed_jitter_after_start_does_not_look_stopped(self):
        c,_,l,_=self.make()
        for v in (.04,.026,.031,.025,.03):
            l.twist.twist.linear.x=v
            c.odom_callback('leader',l,100_100_000_000,10.)
            self.assertGreater(self.command(c)[0],0.)

    def test_low_speed_cannot_start_or_restart_after_true_stop(self):
        c,_,l,_=self.make()
        for v in (0.,.026,.03):
            l.twist.twist.linear.x=v
            c.odom_callback('leader',l,100_100_000_000,10.)
            self.assert_stop(c,'STOP_LEADER_STOPPED')
        l.twist.twist.linear.x=.04;c.odom_callback('leader',l,100_100_000_000,10.)
        self.assertGreater(self.command(c)[0],0.)
        l.twist.twist.linear.x=.009;c.odom_callback('leader',l,100_100_000_000,10.)
        self.assert_stop(c,'STOP_LEADER_STOPPED')
        l.twist.twist.linear.x=.026;c.odom_callback('leader',l,100_100_000_000,10.)
        self.assert_stop(c,'STOP_LEADER_STOPPED')

    def test_stale_leader_does_not_preserve_motion_latch(self):
        c,_,l,_=self.make()
        c.command(100_700_000_000,10.6)
        self.assertFalse(c.leader_moving)
        l.header.stamp.sec=100;l.header.stamp.nanosec=700_000_000
        l.twist.twist.linear.x=.026
        c.odom_callback('leader',l,100_700_000_000,10.6)
        self.assertFalse(c.leader_moving)

    def test_near_leader_overrides_rotation(self):
        c,_,l,_=self.make();l.pose.pose.position.x=.5;l.twist.twist.angular.z=.1
        c.odom_callback('leader',l,100_100_000_000,10.)
        self.assert_stop(c,'STOP_NEAR_LEADER')

    def test_follow_distance(self):
        c,_,l,_=self.make();l.pose.pose.position.x=1.4
        c.odom_callback('leader',l,100_100_000_000,10.)
        self.assert_stop(c,'STOP_FOLLOW_DISTANCE')

    def test_each_input_expired_or_missing(self):
        for key in ('path','leader','follower'):
            for expired in (False,True):
                c,*_=self.make()
                if expired:
                    stamp,_,data=c.inputs[key];c.inputs[key]=(stamp,9.,data)
                else:
                    c.inputs.pop(key)
                self.assert_stop(c)

    def test_measurement_time_expiry(self):
        c,*_=self.make();self.assertEqual(c.command(100_600_000_001,10.)[:2],(0.,0.))

    def test_small_remote_clock_skew_accepts_but_not_large_skew(self):
        for lead_ns,allowed in [(20_000_000,True),(50_000_000,True),(50_000_001,False)]:
            c,p,l,_=self.make()
            for m in (p,l):
                ns=100_100_000_000+lead_ns
                m.header.stamp.sec=ns//10**9;m.header.stamp.nanosec=ns%10**9
            c.path_callback(p,100_100_000_000,10.)
            c.odom_callback('leader',l,100_100_000_000,10.)
            self.assertEqual(self.command(c)[0]>0,allowed)

    def test_local_future_timestamp_still_rejected(self):
        c,_,_,f=self.make()
        f.header.stamp.sec=100;f.header.stamp.nanosec=100_000_001
        c.odom_callback('follower',f,100_100_000_000,10.)
        self.assert_stop(c,'STOP_MISSING_OR_INVALID_FOLLOWER')

    def test_single_point_invalidates_previous_path(self):
        c,p,_,_=self.make();p.poses=p.poses[:1]
        c.path_callback(p,100_100_000_000,10.)
        self.assert_stop(c)

    def test_bad_path_frames_times_and_values(self):
        for mode in ('frame','old','future','zero','nan'):
            c,p,_,_=self.make()
            if mode=='frame':p.header.frame_id='odom'
            elif mode=='nan':p.poses[-1].pose.position.x=float('nan')
            else:p.header.stamp.sec={'old':98,'future':101,'zero':0}[mode]
            c.path_callback(p,100_100_000_000,10.)
            self.assert_stop(c)

    def test_invalid_odom_and_recovery(self):
        for key in ('leader','follower'):
            c,_,l,f=self.make();msg=l if key=='leader' else f
            old=msg.header.frame_id;msg.header.frame_id='wrong'
            c.odom_callback(key,msg,100_100_000_000,10.);self.assert_stop(c)
            msg.header.frame_id=old
            c.odom_callback(key,msg,100_100_000_000,10.)
            self.assertGreater(self.command(c)[0],0.)

    def test_output_topic_restriction(self):
        validate_output_topic(TEST_TOPIC)
        for topic in ('/cmd_vel','/follower/cmd_vel','/follower/cmd_vel_raw'):
            with self.assertRaises(RuntimeError):
                validate_output_topic(topic)

    def test_overflow_geometry_stops(self):
        c,p,l,f=self.make(((0.,0.),(1e308,1e308)))
        l.pose.pose.position.x=1e308
        f.pose.pose.position.x=-1e308
        c.odom_callback('leader',l,100_100_000_000,10.)
        c.odom_callback('follower',f,100_100_000_000,10.)
        self.assert_stop(c,'STOP_INVALID_GEOMETRY')


if __name__=='__main__':
    unittest.main(verbosity=2)
