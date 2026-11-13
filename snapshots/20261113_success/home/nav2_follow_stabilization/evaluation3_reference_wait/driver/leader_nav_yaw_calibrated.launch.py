"""Leader yaw feedback x1.18 from repeated IMU/physical angle comparisons.

Default motion gate is false. Uses a diagnostic EKF YAML, not the success YAML.
Yaw feedback1.18; forward feedback1.173 from physical102cm / odom100.56cm.
Physical radius, track and outgoing command conversion remain unchanged.
"""
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
            root, 'leader_sensors_check.launch.py',
        ))),
        Node(
            package='robot_localization', executable='ekf_node',
            name='ekf_filter_node', output='screen',
            parameters=[os.path.join(root, 'leader_ekf_wheel_yaw_candidate.yaml'),
                        {'use_sim_time': False}],
        ),
        Node(
            executable=sys.executable,
            arguments=[os.path.join(root, 'leader_md200t_polling_split_scale_candidate.py')],
            name='md200t_controller', output='screen',
            parameters=[{
                'dry_run': False,
                'allow_motion': ParameterValue(
                    LaunchConfiguration('allow_motion'), value_type=bool),
                'serial_port': (
                    '/dev/serial/by-id/'
                    'usb-FTDI_FT232R_USB_UART_AP2472IE-if00-port0'
                ),
                'baudrate': 57600, 'controller_id': 1,
                'wheel_radius': 0.085, 'wheel_track': 0.560,
                'max_rpm': 25, 'cmd_vel_timeout': 0.5,
                'invert_left': False, 'invert_right': False,
                'feedback_poll_rate': 10.0,
                'feedback_speed_scale': 1.173,
                'feedback_yaw_scale': 1.18,
                'wheel_yaw_rate_variance': 0.0009,
                'constrain_lateral_velocity': True,
                'lateral_velocity_variance': 0.0004,
            }],
        ),
    ])
