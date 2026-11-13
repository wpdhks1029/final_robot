"""Success-backup TF topology for follower sensors, AMCL and pose bridge.

No motor driver, command publisher, arming or automatic initial pose is included.
Stop the existing follower sensors/localization/bridge before starting this stack.
"""
from pathlib import Path
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    root=Path.home()/'follower_diagnostics';here=Path(__file__).resolve().parent
    sensor=root/'follower_position_fixed_lidar.launch.py'
    localization=root/'follower_localization.launch.py'
    bridge=here/'real_follower_bridge_tf_candidate.py'
    for p in (sensor,localization,bridge):
        if not p.is_file():raise RuntimeError('Missing verified source: '+str(p))
    return LaunchDescription([
        DeclareLaunchArgument('map',default_value=str(Path.home()/'robot_ws/maps/leader_mapping_20261107_143101_9R24Wc/leader_map.yaml')),
        DeclareLaunchArgument('ekf_frequency',default_value='20.0'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(sensor)),
            launch_arguments={'enable_position_profile':'true','ekf_frequency':LaunchConfiguration('ekf_frequency')}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(localization)),
            launch_arguments={'enable_localization':'true','map':LaunchConfiguration('map')}.items()),
        Node(executable=sys.executable,arguments=['-B',str(bridge)],name='real_follower_bridge',output='screen'),
    ])
