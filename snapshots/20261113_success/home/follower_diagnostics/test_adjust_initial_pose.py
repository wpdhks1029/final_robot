import math
import unittest
from types import SimpleNamespace as S
from unittest.mock import patch
from adjust_initial_pose import Pose,plan,apply_ready,fill_pose,main


def odom(frame,child):
    vector=lambda:S(x=0.,y=0.,z=0.)
    return S(header=S(frame_id=frame,stamp=S(sec=10,nanosec=0)),child_frame_id=child,
             twist=S(twist=S(linear=vector(),angular=vector())))


class Tests(unittest.TestCase):
    def ready(self):
        motor=(dict(armed=False,fault=None,io_failed=False,feedback_fresh=True,measured_rpm=[0,0]),1.)
        leader=(odom('odom','base_link'),1.)
        follower=(odom('follower/odom','follower/base_link'),1.)
        return motor,leader,follower

    def test_base_offset_rotates_into_map(self):
        p=plan(Pose(2.,3.,math.pi/2),dx_cm=1,dy_cm=2,dyaw_deg=1)
        self.assertAlmostEqual(p.x,1.98);self.assertAlmostEqual(p.y,3.01)
        self.assertAlmostEqual(math.degrees(p.yaw),91.)

    def test_map_offset_does_not_rotate(self):
        p=plan(Pose(2.,3.,math.pi/2),dx_cm=1,dy_cm=2,offset_frame='map')
        self.assertAlmostEqual(p.x,2.01);self.assertAlmostEqual(p.y,3.02)

    def test_absolute_pose_without_existing_map_tf(self):
        p=plan(None,x=2.,y=.1,yaw_deg=181)
        self.assertAlmostEqual(math.degrees(p.yaw),-179.)

    def test_absolute_heading_keeps_fresh_current_position(self):
        p=plan(Pose(1.923,.01996,math.radians(14.64)),set_yaw_deg=13.72)
        self.assertEqual((p.x,p.y),(1.923,.01996))
        self.assertAlmostEqual(math.degrees(p.yaw),13.72)
        with self.assertRaises(ValueError):plan(None,set_yaw_deg=13.72)
        with self.assertRaises(ValueError):plan(Pose(1.,2.,0.),set_yaw_deg=13.72,dyaw_deg=1.)

    def test_incomplete_mixed_large_or_nonfinite_rejected(self):
        for kw in [dict(x=2),dict(x=1,y=2,yaw_deg=3,dx_cm=1),dict(dx_cm=21),
                   dict(dyaw_deg=11),dict(dyaw_deg=float('nan'))]:
            with self.assertRaises(ValueError):plan(Pose(0.,0.,0.),**kw)

    def test_healthy_stationary_inputs_allowed(self):
        self.assertTrue(apply_ready(*self.ready(),10_100_000_000,1.1))

    def test_armed_or_feedback_fault_rejected(self):
        for key,value in [('armed',True),('fault','COMMAND_TIMEOUT'),('feedback_fresh',False),('measured_rpm',[1,0])]:
            m,l,f=self.ready();m[0][key]=value
            self.assertFalse(apply_ready(m,l,f,10_100_000_000,1.1))

    def test_either_robot_motion_or_stale_data_rejected(self):
        for name in ('leader','follower'):
            m,l,f=self.ready();sample=l if name=='leader' else f
            sample[0].twist.twist.angular.z=.02
            self.assertFalse(apply_ready(m,l,f,10_100_000_000,1.1))
        self.assertFalse(apply_ready(*self.ready(),10_400_000_000,1.4))

    def test_published_frame_quaternion_and_covariance(self):
        p=plan(None,x=2.,y=.1,yaw_deg=90.)
        m=S(header=S(),pose=S(pose=S(position=S(),orientation=S())))
        fill_pose(m,p,S(sec=10,nanosec=0),.5,15)
        self.assertEqual(m.header.frame_id,'map')
        self.assertAlmostEqual(m.pose.pose.orientation.z,math.sqrt(.5))
        self.assertAlmostEqual(m.pose.pose.orientation.w,math.sqrt(.5))
        self.assertEqual(m.pose.covariance[0],.25)
        self.assertAlmostEqual(m.pose.covariance[35],math.radians(15)**2)

    def test_apply_requires_ack_before_ros(self):
        with patch('sys.argv',['adjust','--apply','--dyaw-deg','1']):
            with self.assertRaises(SystemExit):main()


if __name__=='__main__':unittest.main()
