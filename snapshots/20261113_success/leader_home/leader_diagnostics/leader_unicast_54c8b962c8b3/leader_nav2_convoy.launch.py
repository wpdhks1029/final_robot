"""Humble Nav2 launch with a separately routed, fail-closed motor gate.

Load the installed Nav2 launch description and change only command-producing
nodes' remappings. A group-wide cmd_vel remap also captures the smoother's
output and can create a feedback loop, so it must not be used here.
"""
import importlib.util
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.utilities import normalize_remap_rules

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from nav2_convoy_launch_core import remappings_for_nav2_node


def _route_nodes(actions):
    """Apply only the controller, behavior, and smoother command routes."""
    for action in actions:
        if isinstance(action, Node):
            # Humble's navigation_launch.py leaves controller_server's node
            # name unspecified, so launch_ros derives it from the executable.
            name = (getattr(action, '_Node__node_name', None)
                    or getattr(action, '_Node__node_executable', None))
            if name in ('controller_server', 'behavior_server', 'velocity_smoother'):
                action._Node__remappings = list(
                    normalize_remap_rules(remappings_for_nav2_node(name)))
            continue
        children = getattr(action, '_GroupAction__actions', None)
        if children is not None:
            _route_nodes(children)


def generate_launch_description():
    nav2_share = get_package_share_directory('nav2_bringup')
    source = os.path.join(nav2_share, 'launch', 'navigation_launch.py')
    spec = importlib.util.spec_from_file_location('nav2_humble_navigation_launch', source)
    if spec is None or spec.loader is None:
        raise RuntimeError(f'Cannot load installed Nav2 launch file: {source}')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    nav2 = module.generate_launch_description()
    _route_nodes(nav2.entities)
    nav2.add_action(DeclareLaunchArgument('convoy_max_linear', default_value='0.04',
                                        choices=['0.04', '0.05']))
    nav2.add_action(DeclareLaunchArgument('convoy_motor_status_topic',default_value='/follower/motor_candidate/status'))
    nav2.add_action(DeclareLaunchArgument('convoy_follow_status_topic',default_value='/follower/local_avoidance/status'))
    nav2.add_action(DeclareLaunchArgument('convoy_follower_pose_topic',default_value='/follower/odom_map'))
    gate = ExecuteProcess(
        cmd=['python3', os.path.join(_HERE, 'nav2_convoy_gate.py'),
             '--ros-args', '-p', ['max_linear:=', LaunchConfiguration('convoy_max_linear')],
             '-r', ['/follower/motor_candidate/status:=',LaunchConfiguration('convoy_motor_status_topic')],
             '-r', ['/follower/local_avoidance/status:=',LaunchConfiguration('convoy_follow_status_topic')],
             '-r', ['/follower/odom_map:=',LaunchConfiguration('convoy_follower_pose_topic')]],
        output='screen',
    )
    nav2.add_action(gate)
    return nav2
