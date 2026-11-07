#!/usr/bin/env python3
"""Explicitly authorized hardware LOCKED mode: zero-RPM/PID4 only; no arm/input API.

Default invocation starts nothing. Not a velocity controller or physical watchdog.
Do not use beside the existing feedback node. Sensor/EKF relaunch is a separate step.
"""
import argparse
import json
import math
from pathlib import Path
import sys

from follower_motor_candidate import MotorCore,Config


def check_locked_feedback(core):
    if core.feedback is not None and (core.feedback['m1_rpm']!=0 or core.feedback['m2_rpm']!=0):
        core.latch_fault('UNEXPECTED_MOTION_WHILE_LOCKED')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-hardware-locked',action='store_true')
    parser.add_argument('--ack-zero-rpm-tx',action='store_true',
                        help='Acknowledge physical port opening and zero-RPM ENABLE=1/PID4 transmission')
    position_choice=parser.add_mutually_exclusive_group()
    position_choice.add_argument('--publish-position-test',action='store_true',
                        help='Additional TEST-only position odom; motor remains locked')
    position_choice.add_argument('--publish-position-candidate',action='store_true',
                        help='Candidate odom in native follower frames; remains locked, RPM raw output preserved')
    args=parser.parse_args()
    if not args.run_hardware_locked:parser.print_help();return 0
    if not args.ack_zero_rpm_tx:parser.error('Explicit --ack-zero-rpm-tx is required')

    import yaml
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import ExternalShutdownException
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from follower_motor_port import ExclusivePort

    params=yaml.safe_load(Path(__file__).with_name('follower_hardware.yaml').read_text())['follower_md200t_pid4']['ros__parameters']
    # Freeze the operator-confirmed geometry/device contract for this first adapter trial.
    if (params['baudrate']!=19200 or params['driver_id']!=1
            or params['wheel_radius']!=.065 or params['wheel_track']!=.560
            or params['invert_left'] is not True or params['invert_right'] is not False
            or not params['port'].endswith('usb-0:1.4:1.0-port0')):
        raise RuntimeError('Configuration differs from confirmed follower setup; re-review before hardware use')
    rclpy.init(args=[])
    node=Node('follower_motor_locked')
    core=None;transport=None
    try:
        for topic in ['/follower/odom_raw','/follower/motor_locked/status']:
            if node.resolve_topic_name(topic)!=topic:raise RuntimeError('Locked hardware topic remap rejected')
        # Discover a conflicting ROS publisher before opening the physical port.
        import time
        until=time.monotonic()+1.5
        while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.1)
        if node.get_publishers_info_by_topic('/follower/odom_raw'):
            raise RuntimeError('Existing /follower/odom_raw publisher; stop it first, do not run duplicate drivers')
        position_test=None
        if args.publish_position_test or args.publish_position_candidate:
            from follower_position_preview import PositionPublisher,PositionCandidatePublisher
            position_test=(PositionCandidatePublisher if args.publish_position_candidate else PositionPublisher)(node)
        transport=ExclusivePort.open(params['port'],params['baudrate'])
        core=MotorCore(transport,dry_run=False,allow_motion=False,config=Config())
        pub=node.create_publisher(Odometry,'/follower/odom_raw',20)
        status_pub=node.create_publisher(String,'/follower/motor_locked/status',10)
        last_status=0.

        def tick():
            nonlocal last_status
            now=node.get_clock().now();before=core.valid_count
            core.tick(now.nanoseconds)
            if core.valid_count!=before:
                check_locked_feedback(core)
                odom=Odometry();odom.header.stamp=now.to_msg()
                odom.header.frame_id='follower/odom';odom.child_frame_id='follower/base_link'
                odom.pose.pose.position.x=core.odom_x;odom.pose.pose.position.y=core.odom_y
                odom.pose.pose.orientation.z=math.sin(core.odom_yaw/2)
                odom.pose.pose.orientation.w=math.cos(core.odom_yaw/2)
                odom.twist.twist.linear.x=core.odom_v;odom.twist.twist.angular.z=core.odom_w
                odom.pose.covariance[0]=odom.pose.covariance[7]=.05;odom.pose.covariance[35]=.1
                odom.twist.covariance[0]=odom.twist.covariance[7]=.02;odom.twist.covariance[35]=.05
                pub.publish(odom)
                if position_test is not None:position_test.publish(core,now.to_msg())
            if core.clock()-last_status>=1.:
                last_status=core.clock()
                state={'mode':'HARDWARE_LOCKED','armed':core.armed,'fault':core.fault,
                       'feedback_fresh':core.feedback_fresh(),'valid_feedback':core.valid_count,
                       'sent_rpm':core.last_sent,'io_failed':core.io_failed,
                       'measured_rpm':None if core.feedback is None else
                           [core.feedback['m1_rpm'],core.feedback['m2_rpm']]}
                status_pub.publish(String(data=json.dumps(state,allow_nan=False)))
                if core.fault:node.get_logger().error(json.dumps(state))
                else:node.get_logger().info(json.dumps(state))

        timer=node.create_timer(.02,tick)
        node.get_logger().info('HARDWARE LOCKED: zero-RPM/PID4 only; no command subscriptions, arm services or TF')
        rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    except Exception as exc:
        print(f'Locked hardware adapter stopped: {exc}',file=sys.stderr)
        return 1
    finally:
        if core is not None:core.shutdown()
        elif transport is not None:transport.close()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    return 0


if __name__=='__main__':sys.exit(main())
