import importlib.util
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from launch.actions import ExecuteProcess, GroupAction
from launch.launch_context import LaunchContext
from launch_ros.actions import Node


LAUNCH_PATH = Path(__file__).with_name('leader_nav2_convoy.launch.py')
SPEC = importlib.util.spec_from_file_location('leader_nav2_convoy_launch', LAUNCH_PATH)
LAUNCH_MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(LAUNCH_MODULE)


class ConvoyLaunchTests(unittest.TestCase):
    def test_nested_nav2_nodes_get_normalized_non_looping_remaps(self):
        nodes = [
            # Match Humble navigation_launch.py: controller node name is inferred
            # from its executable and starts with cmd_vel -> cmd_vel_nav.
            Node(package='nav2_controller', executable='controller_server',
                 remappings=[('cmd_vel', 'cmd_vel_nav')]),
            Node(package='nav2_behaviors', executable='behavior_server',
                 name='behavior_server'),
            Node(package='nav2_velocity_smoother', executable='velocity_smoother',
                 name='velocity_smoother',
                 remappings=[('cmd_vel', 'cmd_vel_nav'),
                             ('cmd_vel_smoothed', 'cmd_vel')]),
            Node(package='nav2_planner', executable='planner_server',
                 name='planner_server'),
        ]
        LAUNCH_MODULE._route_nodes([GroupAction(actions=nodes)])

        context = LaunchContext()
        actual = {}
        for node in nodes:
            node._perform_substitutions(context)
            name = node._Node__node_name or node._Node__node_executable
            actual[name] = dict(node.expanded_remapping_rules or [])

        self.assertEqual(actual['controller_server']['cmd_vel'], '/leader/nav2_cmd_vel')
        self.assertEqual(actual['behavior_server']['cmd_vel'], '/leader/nav2_cmd_vel')
        smoother = actual['velocity_smoother']
        self.assertEqual(smoother['cmd_vel'], '/leader/nav2_cmd_vel')
        self.assertEqual(smoother['cmd_vel_smoothed'], '/cmd_vel_smoothed')
        self.assertNotEqual(smoother['cmd_vel'], smoother['cmd_vel_smoothed'])
        self.assertNotIn('cmd_vel', actual['planner_server'])

    def test_wrapper_generates_fake_humble_description_and_gate(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            share = Path(temp_dir) / 'nav2_bringup'
            launch_dir = share / 'launch'
            launch_dir.mkdir(parents=True)
            (launch_dir / 'navigation_launch.py').write_text('''
from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node
def generate_launch_description():
    return LaunchDescription([GroupAction(actions=[
        Node(package='nav2_controller', executable='controller_server',
             remappings=[('cmd_vel', 'cmd_vel_nav')]),
        Node(package='nav2_behaviors', executable='behavior_server',
             name='behavior_server'),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother',
             name='velocity_smoother',
             remappings=[('cmd_vel', 'cmd_vel_nav'),
                         ('cmd_vel_smoothed', 'cmd_vel')]),
        Node(package='nav2_planner', executable='planner_server',
             name='planner_server'),
    ])])
''')
            with patch.object(
                LAUNCH_MODULE,
                'get_package_share_directory',
                return_value=str(share),
            ):
                description = LAUNCH_MODULE.generate_launch_description()

        self.assertEqual(len(description.entities), 6)
        self.assertIsInstance(description.entities[-1], ExecuteProcess)
        nodes = description.entities[0]._GroupAction__actions
        actual = {}
        context = LaunchContext()
        for node in nodes:
            node._perform_substitutions(context)
            name = node._Node__node_name or node._Node__node_executable
            actual[name] = dict(node.expanded_remapping_rules or [])
        self.assertEqual(actual['controller_server']['cmd_vel'], '/leader/nav2_cmd_vel')
        self.assertEqual(actual['behavior_server']['cmd_vel'], '/leader/nav2_cmd_vel')
        self.assertEqual(actual['velocity_smoother']['cmd_vel'], '/leader/nav2_cmd_vel')
        self.assertEqual(actual['velocity_smoother']['cmd_vel_smoothed'], '/cmd_vel_smoothed')
        self.assertNotIn('cmd_vel', actual['planner_server'])


if __name__ == '__main__':
    unittest.main()
