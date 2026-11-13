"""Lifted-wheel ROS check; motion enabled, RPM limited to 5.

Use only with wheels secured above the ground and all previous ROS launches
stopped. Includes sensors/TF/EKF and exactly one MD200T node. No Nav2/AMCL/bridge.
"""
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            os.path.dirname(__file__), 'leader_stationary_ekf_check.launch.py',
        ))),
        Node(
            package='leader_robot_control',
            executable='md200t_controller',
            name='md200t_controller',
            output='screen',
            parameters=[{
                'dry_run': False,
                'allow_motion': True,
                'serial_port': (
                    '/dev/serial/by-id/'
                    'usb-FTDI_FT232R_USB_UART_AP2472IE-if00-port0'
                ),
                'baudrate': 57600,
                'controller_id': 1,
                'wheel_radius': 0.085,
                'wheel_track': 0.466,
                'max_rpm': 5,
                'cmd_vel_timeout': 0.5,
                'invert_left': False,
                'invert_right': False,
            }],
        ),
    ])
