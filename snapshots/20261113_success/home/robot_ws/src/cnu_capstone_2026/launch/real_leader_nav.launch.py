#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    convoy_share = get_package_share_directory('convoy_robot')
    leader_share = get_package_share_directory('leader_robot_control')
    nav2_share = get_package_share_directory('nav2_bringup')

    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    hardware_ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                leader_share,
                'launch',
                'leader_robot_ekf.launch.py',
            )
        )
    )

    nav2 = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        nav2_share,
                        'launch',
                        'bringup_launch.py',
                    )
                ),
                launch_arguments={
                    'map': map_file,
                    'params_file': params_file,
                    'use_sim_time': use_sim_time,
                    'autostart': 'true',
                }.items(),
            )
        ],
    )

    leader_bridge = Node(
        package='convoy_robot',
        executable='real_leader_bridge_node.py',
        name='real_leader_bridge',
        output='screen',
        parameters=[
            os.path.join(
                convoy_share,
                'config',
                'real_leader_bridge.yaml',
            )
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=os.path.expanduser(
                '~/robot_ws/maps/leader_map.yaml'
            ),
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                leader_share,
                'config',
                'nav2_params.yaml',
            ),
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
        ),
        hardware_ekf,
        nav2,
        leader_bridge,
    ])
