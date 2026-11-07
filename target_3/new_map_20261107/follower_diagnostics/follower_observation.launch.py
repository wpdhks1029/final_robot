"""Opt-in follower observation with operator-confirmed sensor mounting TF."""
from pathlib import Path
import math
import sys
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


ROOT = Path(__file__).resolve().parent
WORKSPACE = Path.home() / 'robot_ws'
HARDWARE = ROOT / 'follower_hardware.yaml'
PID4 = WORKSPACE / 'test_tools/follower_md200t_pid4_node.py'
EKF = WORKSPACE / 'src/cnu_capstone_2026/config/follower_ekf_A_selected.yaml'


def enabled(context, name):
    value = LaunchConfiguration(name).perform(context).lower()
    if value not in ('true', 'false'):
        raise RuntimeError(f'{name} must be true or false')
    return value == 'true'


def observation(context):
    feedback = enabled(context, 'enable_feedback')
    external_feedback = enabled(context, 'external_feedback_confirmed')
    imu = enabled(context, 'enable_imu')
    ekf = enabled(context, 'enable_ekf')
    tf_confirmed = enabled(context, 'imu_tf_confirmed')
    sensor_tf = enabled(context, 'enable_sensor_tf')
    lidar = enabled(context, 'enable_lidar')
    profile=LaunchConfiguration('odom_profile').perform(context)
    if profile not in ('rpm','position_candidate'):
        raise RuntimeError('odom_profile must be rpm or position_candidate')
    ekf_path=EKF if profile=='rpm' else ROOT/'follower_ekf_position_candidate.yaml'
    ekf_frequency=float(LaunchConfiguration('ekf_frequency').perform(context))
    if not math.isfinite(ekf_frequency) or (ekf_frequency!=0 and not 10<=ekf_frequency<=30):
        raise RuntimeError('ekf_frequency must be 0 (YAML default) or 10..30 Hz')
    if profile=='position_candidate' and (feedback or not external_feedback):
        raise RuntimeError('Position candidate requires one external position source; internal RPM polling must be disabled')
    laser_flip = enabled(context, 'laser_flip_180')
    laser_yaw_offset = float(LaunchConfiguration('laser_yaw_offset_deg').perform(context))
    if not math.isfinite(laser_yaw_offset) or abs(laser_yaw_offset) > 30.0:
        raise RuntimeError('laser_yaw_offset_deg must be finite and within +/-30 degrees')

    # Validate the complete selection before returning any process action.
    if feedback and external_feedback:
        raise RuntimeError('Internal and external feedback owners cannot be selected together')
    if ekf and not ((feedback or external_feedback) and imu and (sensor_tf or tf_confirmed)):
        raise RuntimeError(
            'EKF requires enable_feedback=true OR external_feedback_confirmed=true, enable_imu=true and '
            'enable_sensor_tf=true (confirmed mounting values), or an external '
            'IMU mounting TF with imu_tf_confirmed=true.'
        )
    required = ([HARDWARE] if feedback or imu or sensor_tf or lidar else [])
    required += [PID4] if feedback else []
    required += [ekf_path] if ekf else []
    for path in required:
        if not path.is_file():
            raise RuntimeError(f'Required file missing: {path}')

    actions = []
    if sensor_tf:
        with HARDWARE.open() as stream:
            mounts = yaml.safe_load(stream)['follower_sensor_tf']['ros__parameters']
        for sensor, child in [('imu', 'follower/imu_link'),
                              ('laser', 'follower/laser')]:
            xyz = mounts[f'{sensor}_xyz']
            rpy = list(mounts[f'{sensor}_rpy'])
            if sensor == 'laser':
                rpy[2] += math.radians(laser_yaw_offset)
                if laser_flip:
                    rpy[2] += math.pi
            if (len(xyz) != 3 or len(rpy) != 3
                    or not all(isinstance(v, (int, float)) and math.isfinite(v)
                               for v in xyz + rpy)):
                raise RuntimeError(f'Invalid {sensor} mounting xyz/rpy')
            arguments = []
            for flag, value in zip(
                    ['--x', '--y', '--z', '--roll', '--pitch', '--yaw'], xyz + rpy):
                arguments.extend([flag, str(value)])
            arguments += ['--frame-id', 'follower/base_link', '--child-frame-id', child]
            actions.append(Node(
                package='tf2_ros', executable='static_transform_publisher',
                name=f'follower_{sensor}_mount_tf', output='screen',
                arguments=arguments,
            ))
    if feedback:
        actions.append(ExecuteProcess(
            cmd=[sys.executable, '-B', str(PID4), '--ros-args',
                 '--params-file', str(HARDWARE),
                 '-p', 'enable_hardware_polling:=true'],
            output='screen',
            additional_env={'PYTHONDONTWRITEBYTECODE': '1'},
        ))
    if imu:
        actions.append(Node(
            package='iahrs_ros2', executable='iahrs_node',
            name='follower_imu', output='screen',
            parameters=[str(HARDWARE)],
            remappings=[('/imu/data', '/follower/imu/data')],
            additional_env={'PYTHONDONTWRITEBYTECODE': '1'},
        ))
    if lidar:
        actions.append(Node(
            package='ydlidar_ros2_driver', executable='ydlidar_ros2_driver_node',
            name='follower_lidar', output='screen',
            parameters=[str(HARDWARE)],
            remappings=[('scan', '/follower/scan'),
                        ('point_cloud', '/follower/point_cloud'),
                        ('start_scan', '/follower/start_scan'),
                        ('stop_scan', '/follower/stop_scan')],
        ))
    if ekf:
        actions.append(Node(
            package='robot_localization', executable='ekf_node',
            name='follower_ekf_filter_node', output='screen',
            parameters=[str(ekf_path), dict({'use_sim_time': False},
                        **({'frequency':ekf_frequency} if ekf_frequency else {}))],
            remappings=[('odometry/filtered', '/follower/odometry/filtered'),
                        ('/set_pose', '/follower/set_pose'),
                        ('/enable', '/follower/enable'),
                        ('/toggle', '/follower/toggle')],
        ))
    if not actions:
        actions.append(LogInfo(msg='All device nodes disabled. No hardware processes started.'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('enable_feedback', default_value='false',
                              description='Open MD200T and send PID4 requests; writes RX log.'),
        DeclareLaunchArgument('odom_profile',default_value='rpm',
                              description='rpm preserves original settings; position_candidate explicitly selects separate YAML.'),
        DeclareLaunchArgument('external_feedback_confirmed', default_value='false',
                              description='Operator confirms one external /follower/odom_raw provider; opens no motor port here.'),
        DeclareLaunchArgument('enable_imu', default_value='false',
                              description='Open IMU and send g/a/e sensor queries.'),
        DeclareLaunchArgument('enable_ekf', default_value='false'),
        DeclareLaunchArgument('ekf_frequency',default_value='0.0',
                             description='0 preserves YAML frequency; 10..30 explicitly overrides EKF Hz.'),
        DeclareLaunchArgument('enable_lidar', default_value='false',
                              description='Open G2B and start laser scanning; no wheel commands.'),
        DeclareLaunchArgument('laser_yaw_offset_deg', default_value='0.0',
                              description='Temporary laser TF yaw correction; positive is CCW, +/-30 deg.'),
        DeclareLaunchArgument('laser_flip_180', default_value='false',
                              description='Temporary 180-degree laser TF flip for front/back comparison.'),
        DeclareLaunchArgument('enable_sensor_tf', default_value='false',
                              description='Publish confirmed follower sensor mounting TFs.'),
        DeclareLaunchArgument('imu_tf_confirmed', default_value='false',
                              description='Operator confirms measured IMU mounting TF is available.'),
        OpaqueFunction(function=observation),
    ])
