import unittest

from nav2_convoy_launch_core import remappings_for_nav2_node


class Nav2ConvoyLaunchTests(unittest.TestCase):
    def test_controller_and_behavior_feed_the_smoother_input(self):
        for name in ('controller_server', 'behavior_server'):
            rules = dict(remappings_for_nav2_node(name))
            self.assertEqual(rules['cmd_vel'], '/leader/nav2_cmd_vel')
            self.assertNotIn('cmd_vel_smoothed', rules)

    def test_smoother_input_and_output_are_distinct(self):
        rules = dict(remappings_for_nav2_node('velocity_smoother'))
        self.assertEqual(rules['cmd_vel'], '/leader/nav2_cmd_vel')
        self.assertEqual(rules['cmd_vel_smoothed'], '/cmd_vel_smoothed')
        self.assertNotEqual(rules['cmd_vel'], rules['cmd_vel_smoothed'])

    def test_other_nodes_keep_only_tf_remaps(self):
        self.assertEqual(
            dict(remappings_for_nav2_node('planner_server')),
            {'/tf': 'tf', '/tf_static': 'tf_static'},
        )


if __name__ == '__main__':
    unittest.main()
