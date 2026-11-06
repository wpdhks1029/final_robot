"""Separate opt-in native-frame position EKF observation profile; motor stays locked.

Start one locked owner with --publish-position-candidate first. Stop old sensor/
observation launch first. This launch opens sensors, never a motor port.
"""
from pathlib import Path
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription,LogInfo,OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

ROOT=Path(__file__).resolve().parent


def start(context):
    enabled=LaunchConfiguration('enable_position_profile').perform(context).lower()
    if enabled not in ('true','false'):raise RuntimeError('enable_position_profile must be true or false')
    if enabled=='false':return [LogInfo(msg='Position profile disabled; no sensor/EKF processes started.')]
    if str(ROOT) not in sys.path:sys.path.insert(0,str(ROOT))
    from position_profile_contract import preflight
    preflight()
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(str(ROOT/'follower_observation.launch.py')),
        launch_arguments={'enable_sensor_tf':'true','enable_lidar':'true','enable_imu':'true','enable_ekf':'true',
                          'enable_feedback':'false','external_feedback_confirmed':'true',
                          'odom_profile':'position_candidate',
                          'ekf_frequency':LaunchConfiguration('ekf_frequency').perform(context)}.items())]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('enable_position_profile',default_value='false'),
                              DeclareLaunchArgument('ekf_frequency',default_value='0.0'),
                              OpaqueFunction(function=start)])
