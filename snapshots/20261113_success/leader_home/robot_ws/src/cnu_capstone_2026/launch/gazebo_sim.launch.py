import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_dir = get_package_share_directory('convoy_robot')
    leader_urdf   = os.path.join(pkg_dir, 'urdf', 'leader_robot.urdf.xacro')
    follower_urdf = os.path.join(pkg_dir, 'urdf', 'follower_robot.urdf.xacro')
    world_file    = os.path.join(pkg_dir, 'worlds', 'warehouse.world')

    leader_desc   = ParameterValue(Command(['xacro ', leader_urdf]), value_type=str)
    follower_desc = ParameterValue(Command(['xacro ', follower_urdf]), value_type=str)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch', 'gazebo.launch.py'
            )
        ]),
        launch_arguments={'world': world_file, 'verbose': 'false'}.items(),
    )

    # 선두 로봇: 네임스페이스 없음 (SLAM/Nav2 단일로봇처럼)
    leader_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='leader_rsp',
        parameters=[{'robot_description': leader_desc, 'use_sim_time': True}],
    )

    # 후발 로봇: follower 네임스페이스
    follower_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='follower_rsp',
        namespace='follower',
        parameters=[{'robot_description': follower_desc, 'use_sim_time': True}],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
    )

    spawn_leader = TimerAction(
        period=5.0,
        actions=[Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic', '/robot_description',
                '-entity', 'leader_robot',
                '-x', '0.0', '-y', '0.0', '-z', '0.1',
            ],
            output='screen',
        )]
    )

    spawn_follower = TimerAction(
        period=8.0,
        actions=[Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic', '/follower/robot_description',
                '-entity', 'follower_robot',
                '-x', '-1.5', '-y', '0.0', '-z', '0.1',
            ],
            output='screen',
        )]
    )

    return LaunchDescription([
        gazebo,
        leader_rsp,
        follower_rsp,
        spawn_leader,
        spawn_follower,
    ])
