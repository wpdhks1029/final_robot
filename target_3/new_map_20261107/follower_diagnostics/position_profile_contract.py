"""Read-only readiness checks for the opt-in position profile; no motor APIs."""
import math


def validate(source_count,filtered_count,frame,child,age):
    if source_count!=1:raise RuntimeError('Require exactly one /follower/odom_position_candidate publisher')
    if filtered_count!=0:raise RuntimeError('Existing follower filtered odom publisher; stop old observation/EKF first')
    if frame!='follower/odom' or child!='follower/base_link':raise RuntimeError('Wrong candidate frames')
    if not math.isfinite(age) or not 0<=age<=.3:raise RuntimeError('Candidate measurement missing/stale/future')


def preflight():
    import time
    import rclpy
    from rclpy.context import Context
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from nav_msgs.msg import Odometry
    context=Context();rclpy.init(args=[],context=context)
    node=Node('position_profile_preflight',context=context)
    executor=SingleThreadedExecutor(context=context);executor.add_node(node)
    latest=[]
    subscription=node.create_subscription(Odometry,'/follower/odom_position_candidate',lambda m:latest.append(m),10)
    try:
        until=time.monotonic()+3.
        while time.monotonic()<until:executor.spin_once(timeout_sec=.1)
        if not latest:raise RuntimeError('No position candidate messages received within 3s; check follower drive status, last_position_error and /follower/odom_position_candidate. follower_drive_node.py publishes this automatically; do not add --publish-position-candidate.')
        m=latest[-1];stamp=m.header.stamp.sec*10**9+m.header.stamp.nanosec
        age=(node.get_clock().now().nanoseconds-stamp)/1e9
        validate(len(node.get_publishers_info_by_topic('/follower/odom_position_candidate')),
                 len(node.get_publishers_info_by_topic('/follower/odometry/filtered')),
                 m.header.frame_id,m.child_frame_id,age)
    finally:
        executor.remove_node(node);executor.shutdown();node.destroy_node();rclpy.shutdown(context=context)
