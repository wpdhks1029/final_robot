"""재시작용 위치 EKF+고정8비트 라이다 후보 구성. 기존 센서 launch와 동시 실행 금지."""
from pathlib import Path
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription,LogInfo,OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ROOT=Path(__file__).resolve().parent


def start(context):
    enabled=LaunchConfiguration('enable_position_profile').perform(context).lower()
    if enabled not in ('true','false'):raise RuntimeError('enable_position_profile must be true or false')
    if enabled=='false':return [LogInfo(msg='위치 센서 구성 비활성. 프로세스를 시작하지 않습니다.')]
    binary=ROOT/'lidar_sdk_trace/build/lidar_sdk_trace_node'
    for path in (binary,ROOT/'follower_hardware.yaml',ROOT/'follower_observation.launch.py'):
        if not path.is_file():raise RuntimeError('필수 파일 없음: '+str(path))
    if str(ROOT) not in sys.path:sys.path.insert(0,str(ROOT))
    from position_profile_contract import preflight
    preflight()
    sensors=IncludeLaunchDescription(PythonLaunchDescriptionSource(str(ROOT/'follower_observation.launch.py')),
        launch_arguments=dict(enable_sensor_tf='true',enable_lidar='false',enable_imu='true',enable_ekf='true',
                              enable_feedback='false',external_feedback_confirmed='true',odom_profile='position_candidate',
                              ekf_frequency=LaunchConfiguration('ekf_frequency').perform(context)).items())
    lidar=Node(executable=str(binary),name='follower_lidar',output='screen',
               parameters=[str(ROOT/'follower_hardware.yaml'),{'auto_intensity':False}],
               remappings=[('scan','/follower/scan'),('point_cloud','/follower/point_cloud'),
                           ('start_scan','/follower/start_scan'),('stop_scan','/follower/stop_scan')])
    return [sensors,lidar]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('enable_position_profile',default_value='false'),
                              DeclareLaunchArgument('ekf_frequency',default_value='0.0'),
                              OpaqueFunction(function=start)])
