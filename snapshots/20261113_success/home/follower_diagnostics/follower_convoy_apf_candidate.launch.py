"""APF 초기 조향 연결 후보. 통과/재합류 단계는 정지. 기본 비활성."""
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
    visible=LaunchConfiguration('confirm_obstacle_visible').perform(context).lower()
    center=LaunchConfiguration('obstacle_center_y').perform(context)
    if visible!='true':raise RuntimeError('Raised obstacle visibility confirmation required')
    from math import isfinite
    if not isfinite(float(center)):raise RuntimeError('Invalid obstacle offset')
    rejoin=LaunchConfiguration('enable_rejoin').perform(context).lower()
    if rejoin not in ('true','false'):raise RuntimeError('Invalid enable_rejoin')
    extra=['--enable-apf-rejoin'] if rejoin=='true' else []
    preflight()
    pursuit=ExecuteProcess(cmd=[sys.executable,'-B',str(ROOT/'follower_pursuit_preview.py')],output='screen')
    obstacle=ExecuteProcess(cmd=[sys.executable,'-B',str(ROOT/'follower_obstacle_preview.py'),'--enable-apf-candidate','--confirm-obstacle-visible','--obstacle-center-y',center]+extra,output='screen')
    return [LogInfo(msg='APF 초기 조향 후보: Gateway가 실행 중이면 제어 체인에 연결됩니다. 비무장 상태로 시작합니다. 재합류 모드: '+rejoin+''),
            RegisterEventHandler(OnProcessExit(target_action=pursuit,
                on_exit=[EmitEvent(event=Shutdown(reason='Pursuit preview exited'))])),
            RegisterEventHandler(OnProcessExit(target_action=obstacle,
                on_exit=[EmitEvent(event=Shutdown(reason='Obstacle preview exited'))])),
            pursuit,obstacle]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('enable_preview',default_value='false'),
                              DeclareLaunchArgument('confirm_obstacle_visible',default_value='false'),
                              DeclareLaunchArgument('obstacle_center_y',default_value='0.15'),
                              DeclareLaunchArgument('enable_rejoin',default_value='false'),
                              OpaqueFunction(function=start)])
