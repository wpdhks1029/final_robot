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

    ekf_launch = os.path.join(
        leader_share,
        'launch',
        'leader_robot_ekf.launch.py'
    )

    slam_params = os.path.join(
        leader_share,
        'config',
        'slam_toolbox.yaml'
    )

    return LaunchDescription([

        # ==========================================
        # Robot bringup + EKF
        #
        # Includes:
        # - robot_state_publisher
        # - SLLIDAR
        # - iAHRS
        # - MD200T
        # - robot_localization EKF
        # ==========================================

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                ekf_launch
            )
        ),

        # ==========================================
        # SLAM Toolbox
        # ==========================================

        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',

            parameters=[
                slam_params
            ],
        ),
    ])
