"""Diagnostic actual RPM polling + sensors/EKF, motion blocked by default."""
import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    root = os.path.dirname(__file__)
    return LaunchDescription([
        DeclareLaunchArgument('allow_motion', default_value='false'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            root, 'leader_stationary_ekf_check.launch.py',
        ))),
        Node(
            executable=sys.executable,
            arguments=[os.path.join(root, 'md200t_polling_controller.py')],
            name='md200t_controller',
            output='screen',
            parameters=[{
                'dry_run': False,
                'allow_motion': ParameterValue(
                    LaunchConfiguration('allow_motion'), value_type=bool),
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
                'feedback_poll_rate': 10.0,
            }],
        ),
    ])
