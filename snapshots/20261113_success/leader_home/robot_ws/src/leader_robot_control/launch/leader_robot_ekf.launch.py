import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    leader_share = get_package_share_directory(
        'leader_robot_control'
    )

    bringup_launch = os.path.join(
        leader_share,
        'launch',
        'leader_robot_bringup.launch.py'
    )

    ekf_params = os.path.join(
        leader_share,
        'config',
        'ekf.yaml'
    )

    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                bringup_launch
            )
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                ekf_params
            ],
            remappings=[
                ('odometry/filtered', '/odometry/filtered'),
            ],
        ),

    ])
