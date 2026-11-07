import unittest
from types import SimpleNamespace as S
from unittest.mock import Mock,patch
from follower_ekf_comparison_observer import EkfComparisonObserver,ekf_command,OUTPUT


class Tests(unittest.TestCase):
    def test_readiness_requires_gyro_and_ekf_data(self):
        o=EkfComparisonObserver.__new__(EkfComparisonObserver);o.child=Mock();o.child.poll.return_value=None
        o.imu_count=0;o.rows={OUTPUT:[]};self.assertFalse(o.comparison_ready())
        o.imu_count=10;o.rows[OUTPUT]=[None]*10;self.assertTrue(o.comparison_ready())
        o.child.poll.return_value=1;self.assertFalse(o.comparison_ready())

    def test_child_namespace_and_output(self):
        cmd=ekf_command()
        self.assertIn('__ns:=/follower_position_ekf_test',cmd)
        self.assertIn('odometry/filtered:='+OUTPUT,cmd)
        self.assertNotIn('/follower/odometry/filtered',cmd)

    def test_relay_preserves_original_and_measurement_stamp(self):
        o=EkfComparisonObserver.__new__(EkfComparisonObserver);o.imu_pub=Mock();o.imu_count=0
        m=S(header=S(frame_id='follower/imu_link',stamp=123),angular_velocity=S(z=.02))
        o.relay(m);sent=o.imu_pub.publish.call_args.args[0]
        self.assertEqual(m.header.frame_id,'follower/imu_link')
        self.assertEqual(sent.header.frame_id,'follower_position_preview/imu_link')
        self.assertEqual(sent.header.stamp,123);self.assertEqual(sent.angular_velocity.z,.02)

    def test_child_failure_propagates_before_publish(self):
        o=EkfComparisonObserver.__new__(EkfComparisonObserver);o.child=Mock();o.child.poll.return_value=1
        with patch('follower_comparison_observer.ComparisonObserver.publish') as publish:
            with self.assertRaises(RuntimeError):o.publish(object())
            publish.assert_not_called()

    def test_close_child_and_log(self):
        o=EkfComparisonObserver.__new__(EkfComparisonObserver)
        o.child=Mock();o.child.poll.return_value=None;o.child_log=Mock();log=o.child_log
        with patch('follower_comparison_observer.ComparisonObserver.close') as close:
            o.close();o.child.terminate.assert_called_once();log.close.assert_called_once();close.assert_called_once()


if __name__=='__main__':unittest.main()
