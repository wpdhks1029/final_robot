#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_dir = get_package_share_directory('convoy_robot')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    map_file = LaunchConfiguration(
        'map',
        default=os.path.join(pkg_dir, 'maps', 'convoy_map.yaml')
    )
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('map',
                              default_value=os.path.join(pkg_dir, 'maps', 'convoy_map.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        nav2,
    ])
