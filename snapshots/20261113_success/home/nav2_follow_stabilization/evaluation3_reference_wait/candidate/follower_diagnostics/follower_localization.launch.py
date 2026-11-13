"""Opt-in follower map localization. No sensor or velocity-control processes."""
import hashlib
from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


WORKSPACE = Path.home() / 'robot_ws'
MAP = WORKSPACE / 'maps/leader_map.yaml'
MAP_SHA = '0ece345abf4149f98955ba8b92fd892ced548dac03f82dde542c63d3e9e9b8f6'
IMAGE_SHA = '1afef54bf00023fd64cb41babc6410459f3eeda7f467e848d8374174a9e0f927'
AMCL_CONFIG = WORKSPACE / 'src/cnu_capstone_2026/config/follower_amcl_candidate.yaml'


def localization(context):
    value = LaunchConfiguration('enable_localization').perform(context).lower()
    if value not in ('true', 'false'):
        raise RuntimeError('enable_localization must be true or false')
    if value == 'false':
        return [LogInfo(msg='Follower localization disabled; no processes started.')]

    # Default retains the original success map's hash checks. New maps require
    # an explicit path, never replacement of that original file.
    import sys
    directory = str(Path(__file__).parent)
    if directory not in sys.path:sys.path.insert(0, directory)
    from follower_map_selection import selected_map
    selected, yaml_sha, image_sha = selected_map(
        LaunchConfiguration('map').perform(context), MAP, MAP_SHA, IMAGE_SHA)
    amcl = yaml.safe_load(AMCL_CONFIG.read_text())['follower_amcl']['ros__parameters']
    amcl = dict(amcl)
    # Wire the map through a topic remap rather than relying on map_topic support.
    amcl.pop('map_topic', None)
    amcl.update({
        'use_sim_time': False,
        'base_frame_id': 'follower/base_link',
        'odom_frame_id': 'follower/odom',
        'global_frame_id': 'map',
        'scan_topic': '/follower/scan',
        'set_initial_pose': False,
        'tf_broadcast': True,
    })
    return [
        LogInfo(msg=f'Follower map: {selected}; YAML_SHA256={yaml_sha}; IMAGE_SHA256={image_sha}'),
        Node(
            package='nav2_map_server', executable='map_server',
            namespace='follower', name='map_server', output='screen',
            parameters=[{'use_sim_time': False, 'yaml_filename': str(selected),
                         'topic_name': '/follower/localization_map', 'frame_id': 'map'}],
        ),
        Node(
            package='nav2_amcl', executable='amcl',
            namespace='follower', name='amcl', output='screen',
            parameters=[amcl],
            remappings=[('map', '/follower/localization_map'),
                        ('initialpose', '/follower/initialpose'),
                        ('amcl_pose', '/follower/amcl_pose'),
                        ('particle_cloud', '/follower/particle_cloud'),
                        ('tf', '/tf'), ('tf_static', '/tf_static')],
        ),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            namespace='follower', name='localization_manager', output='screen',
            parameters=[{'use_sim_time': False, 'autostart': True,
                         # Humble bond IDs use get_name(), not the fully qualified name.
                         # Relative service names resolve inside namespace /follower.
                         'node_names': ['map_server', 'amcl']}],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=str(MAP),
                              description='Map YAML; use an explicit new path for a new shared map.'),
        DeclareLaunchArgument('enable_localization', default_value='false',
                              description='Start follower map server, AMCL and lifecycle manager.'),
        OpaqueFunction(function=localization),
    ])
