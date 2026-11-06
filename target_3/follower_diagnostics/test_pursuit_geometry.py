"""Offline geometry checks using existing AST fake ROS harness; no device/ROS context."""
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path.home() / 'robot_ws/test_tools'))
from test_follower_input_safety import SafetyTests, S, header


class GeometryChecks(unittest.TestCase):
    def scenario(self, points):
        n, path, leader, follower = SafetyTests().pursuit()
        path.poses = [S(header=header(), pose=S(position=S(x=x, y=y)))
                      for x, y in points]
        n.path_cb(path)
        return n, path, leader, follower

    def command(self, n):
        n.control_loop()
        return n.cmd_pub.publish.call_args.args[0]

    def test_straight_forward(self):
        n, *_ = self.scenario([(0.,0.), (3.,0.)])
        cmd = self.command(n)
        self.assertGreater(cmd.linear.x, 0.)
        self.assertEqual(cmd.angular.z, 0.)

    def test_left_right_turn_sign(self):
        for y in (1., -1.):
            n, *_ = self.scenario([(0.,0.), (3.,y)])
            cmd = self.command(n)
            self.assertGreater(cmd.linear.x, 0.)
            self.assertGreater(cmd.angular.z*y, 0.)
            self.assertLessEqual(abs(cmd.angular.z), n.max_angular)

    def test_rotated_follower_coordinates(self):
        n, _, _, follower = self.scenario([(0.,0.), (0.,3.)])
        q = follower.pose.pose.orientation
        q.z, q.w = math.sin(math.pi/4), math.cos(math.pi/4)
        n.follower_cb(follower)
        cmd = self.command(n)
        self.assertGreater(cmd.linear.x, 0.)
        self.assertAlmostEqual(cmd.angular.z, 0., places=10)

    def test_path_reset_to_one_point_stops(self):
        n, path, *_ = self.scenario([(0.,0.), (3.,0.)])
        self.assertGreater(self.command(n).linear.x, 0.)
        path.poses = path.poses[:1]
        n.path_cb(path)
        cmd = self.command(n)
        self.assertEqual((cmd.linear.x, cmd.angular.z), (0.,0.))

    def test_each_missing_input_stops(self):
        for attr in ('last_path_rx','last_leader_rx','last_follower_rx'):
            n, *_ = self.scenario([(0.,0.), (3.,0.)])
            setattr(n, attr, 9.)
            cmd = self.command(n)
            self.assertEqual((cmd.linear.x,cmd.angular.z),(0.,0.))

    def test_behind_target_must_not_drive_forward(self):
        # Valid historical path can lie behind the follower after a turn/reset.
        n, *_ = self.scenario([(-3.,0.), (-2.,0.)])
        cmd = self.command(n)
        self.assertLessEqual(cmd.linear.x, 0., 'Target behind base +X must not command forward motion')


if __name__ == '__main__':
    unittest.main(verbosity=2)
