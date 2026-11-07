"""Odom observer tests use fake ROS objects only."""
import unittest
from types import SimpleNamespace as S
from unittest.mock import Mock
from follower_pulse_odom import Observer,fill_odom


def message():
    return S(header=S(stamp=None,frame_id=''),child_frame_id='',
             pose=S(pose=S(position=S(x=0.,y=0.),orientation=S(z=0.,w=0.)),covariance=[0.]*36),
             twist=S(twist=S(linear=S(x=0.),angular=S(z=0.)),covariance=[0.]*36))


class Tests(unittest.TestCase):
    def core(self):return S(odom_x=.007,odom_y=-.001,odom_yaw=0.,odom_v=.013,odom_w=.02,valid_count=1)

    def test_measured_values_frames_covariances(self):
        m=fill_odom(message(),self.core(),'stamp')
        self.assertEqual(m.header.frame_id,'follower/odom')
        self.assertEqual(m.child_frame_id,'follower/base_link')
        self.assertEqual(m.pose.pose.position.x,.007)
        self.assertEqual(m.twist.twist.linear.x,.013)
        self.assertEqual(m.pose.pose.orientation.w,1.)
        self.assertEqual(m.header.stamp,'stamp')
        self.assertGreater(m.twist.covariance[0],0)

    def test_no_new_feedback_no_publish(self):
        o=Observer.__new__(Observer);o.last_count=1;o.pub=Mock()
        self.assertFalse(o.publish(self.core()));o.pub.publish.assert_not_called()

    def test_new_feedback_once(self):
        o=Observer.__new__(Observer);o.last_count=0;o.pub=Mock();o.node=Mock();o.message_type=message
        self.assertTrue(o.publish(self.core()));self.assertFalse(o.publish(self.core()))
        o.pub.publish.assert_called_once()


if __name__=='__main__':unittest.main()
