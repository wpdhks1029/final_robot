"""Opt-in pursuit + obstacle preview. No sensor/motor/real velocity processes."""
from pathlib import Path
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,ExecuteProcess,LogInfo,OpaqueFunction,RegisterEventHandler,EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration

ROOT=Path(__file__).resolve().parent


def preflight():
    import rclpy
    from rclpy.context import Context
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    if str(ROOT) not in sys.path:sys.path.insert(0,str(ROOT))
    from convoy_preview_preflight import wait_for_inputs
    context=Context();rclpy.init(args=[],context=context)
    node=Node('convoy_preview_preflight',context=context)
    executor=SingleThreadedExecutor(context=context);executor.add_node(node)
    try:
        wait_for_inputs(node,lambda:executor.spin_once(timeout_sec=.1))
    finally:
        executor.remove_node(node);executor.shutdown();node.destroy_node();rclpy.shutdown(context=context)


def start(context):
    enabled=LaunchConfiguration('enable_preview').perform(context).lower()
    if enabled not in ('true','false'):raise RuntimeError('enable_preview must be true or false')
    if enabled=='false':return [LogInfo(msg='Preview disabled. No processes started.')]
    for name in ['follower_pursuit_preview.py','follower_obstacle_preview.py']:
        if not (ROOT/name).is_file():raise RuntimeError('Required preview script missing: '+name)
    preflight()
    pursuit=ExecuteProcess(cmd=[sys.executable,'-B',str(ROOT/'follower_pursuit_preview.py')],output='screen')
    obstacle=ExecuteProcess(cmd=[sys.executable,'-B',str(ROOT/'follower_obstacle_preview.py')],output='screen')
    return [LogInfo(msg='PREVIEW ONLY: cmd_vel_test -> cmd_vel_guard_test. No motor connection.'),
            RegisterEventHandler(OnProcessExit(target_action=pursuit,
                on_exit=[EmitEvent(event=Shutdown(reason='Pursuit preview exited'))])),
            RegisterEventHandler(OnProcessExit(target_action=obstacle,
                on_exit=[EmitEvent(event=Shutdown(reason='Obstacle preview exited'))])),
            pursuit,obstacle]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('enable_preview',default_value='false'),
                              OpaqueFunction(function=start)])
