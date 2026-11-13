"""Sensor/TF/EKF check alongside an already running motion-blocked MD200T.

Does not start a motor controller or publish velocity commands. The caller must
keep the existing MD200T node at allow_motion=false. Stop previous standalone
sensor/EKF launches before running this file to avoid duplicate nodes/TF.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    sensor_launch = os.path.join(
        os.path.dirname(__file__), 'leader_sensors_check.launch.py',
    )
    ekf_config = os.path.join(
        get_package_share_directory('leader_robot_control'), 'config', 'ekf.yaml',
    )
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(sensor_launch)),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config, {'use_sim_time': False}],
        ),
    ])
