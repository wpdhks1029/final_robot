#!/usr/bin/env python3
"""Opt-in bounded motor candidate. Default starts nothing.

Memory mode is Domain217 localhost-only. Hardware mode requires separate explicit
acknowledgment and keeps motion disabled unless --enable-motion-input is selected.
Even then, manual /arm and fresh JSON envelope are required. NO Twist gateway.
"""
import argparse
from collections import deque
import json
import os
from pathlib import Path

PREFIX='/follower/motor_candidate'


def publish_position_guarded(position,contract,stamp):
    """위치 입력 오류는 무시하지 않고 정지·잠금 후 진단으로 보존한다."""
    try:
        return position.publish(contract.core,stamp)
    except (ValueError,KeyError,TypeError,OverflowError) as exc:
        contract.core.latch_fault('POSITION_FEEDBACK_INVALID')
        contract.disarm()
        contract.last_position_error=dict(reason=str(exc),feedback=dict(contract.core.feedback or {}),
                                          valid_feedback=contract.core.valid_count)
        return False


def main():
    p=argparse.ArgumentParser(description=__doc__)
    modes=p.add_mutually_exclusive_group()
    modes.add_argument('--run-memory',action='store_true')
    modes.add_argument('--run-hardware',action='store_true')
    p.add_argument('--ack-hardware-zero-rpm-tx',action='store_true')
    p.add_argument('--enable-motion-input',action='store_true')
    p.add_argument('--input-kind',choices=['manual','guard','local_avoidance'],default='manual')
    p.add_argument('--trial-max-rpm',type=int,choices=[3,5,10,15],default=3)
    p.add_argument('--trial-seconds',type=int,choices=[0,5,10,30,60,360],default=5,help='0: no session limit; standard local_avoidance only')
    p.add_argument('--ack-bounded-pivot-candidate',action='store_true')
    p.add_argument('--ack-distance-test-ignore-clearance',action='store_true')
    p.add_argument('--ack-continuous20-candidate',action='store_true')
    p.add_argument('--disable-clearance-stop',action='store_true',help='Explicitly disable follower LiDAR clearance rejection; source/command/feedback/RPM limits remain')
    p.add_argument('--fault-snapshot-dir',help='Optional background file writer for the first motor fault; no continuous tracing')
    p.add_argument('--ack-extended-follow-speed',action='store_true',help='Opt-in standard local follow profile, 15RPM cap')
    a=p.parse_args()
    if a.ack_extended_follow_speed and (a.input_kind!='local_avoidance' or a.trial_max_rpm!=15 or a.ack_continuous20_candidate or a.ack_distance_test_ignore_clearance):
        p.error('Extended follow requires standard local_avoidance, 15RPM and no distance/continuous profile')
    if a.trial_max_rpm==15 and not a.ack_extended_follow_speed:
        p.error('15RPM requires --ack-extended-follow-speed')
    if a.disable_clearance_stop and (a.input_kind!='local_avoidance' or a.ack_continuous20_candidate or a.ack_distance_test_ignore_clearance):
        p.error('--disable-clearance-stop requires standard local_avoidance mode')
    if a.ack_continuous20_candidate and (a.input_kind!='local_avoidance' or a.ack_distance_test_ignore_clearance):p.error('Continuous20 requires exclusive local avoidance mode')
    if a.ack_distance_test_ignore_clearance and a.input_kind!='local_avoidance':p.error('Distance-test option requires local_avoidance driver')
    if a.input_kind=='local_avoidance':
        if not a.ack_bounded_pivot_candidate or a.trial_max_rpm!=(15 if a.ack_extended_follow_speed else 10) or a.trial_seconds not in (0,30,60,360):
            p.error('Local avoidance requires explicit pivot acknowledgement, matching 10/15RPM profile and 30/60/360s')
    elif a.trial_seconds in (0,360) or a.ack_bounded_pivot_candidate:
        p.error('Pivot acknowledgement and 360s are exclusive to local_avoidance mode')
    if a.trial_seconds==0 and (a.ack_continuous20_candidate or a.ack_distance_test_ignore_clearance):
        p.error('Unlimited sessions require standard local follow mode')
    if not a.run_memory and not a.run_hardware:p.print_help();return 0
    if a.run_hardware and not a.ack_hardware_zero_rpm_tx:p.error('Physical zero-RPM/PID4 acknowledgment required')
    if a.run_memory:
        os.environ['ROS_DOMAIN_ID']='217';os.environ['ROS_LOCALHOST_ONLY']='1'
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import ExternalShutdownException
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from nav_msgs.msg import Odometry
    from follower_drive_candidate import DriveContract,BoundedDrivePort
    if a.input_kind=='local_avoidance':
        from local_avoidance_drive import PivotDrivePort as BoundedDrivePort,LocalAvoidanceContract as DriveContract,COMMAND_TOPIC
        if a.ack_continuous20_candidate:COMMAND_TOPIC='/follower/motor_candidate/continuous20_command'
        if a.ack_distance_test_ignore_clearance:COMMAND_TOPIC='/follower/motor_candidate/distance_test_command'
    from follower_motor_mock import MemoryTransport
    from follower_position_preview import PositionCandidatePublisher
    from follower_pulse_odom import fill_odom

    rclpy.init(args=[]);node=Node('follower_drive_candidate')
    contract=None;port=None;fault_writer=None
    try:
        if a.fault_snapshot_dir:
            from fault_snapshot import FaultSnapshotWriter, capture_fault
            fault_writer=FaultSnapshotWriter(a.fault_snapshot_dir)
        import time
        end=time.monotonic()+2
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.1)
        allowed={node.get_name(),'follower_drive_contract_check','follower_guard_gateway','follower_guard_gateway_check'}
        if a.run_memory and any(n not in allowed or ns!='/' for n,ns in node.get_node_names_and_namespaces()):
            raise RuntimeError('Domain217 already in use; refusing memory test')
        if node.get_publishers_info_by_topic('/follower/odom_raw'):
            raise RuntimeError('Existing raw odom source; stop previous owner first')
        position=PositionCandidatePublisher(node)
        if a.run_hardware:
            import yaml
            cfg=yaml.safe_load(Path(__file__).with_name('follower_hardware.yaml').read_text())['follower_md200t_pid4']['ros__parameters']
            if (cfg['driver_id']!=1 or cfg['baudrate']!=19200 or cfg['wheel_radius']!=.065
                    or cfg['wheel_track']!=.560 or cfg['invert_left'] is not True or cfg['invert_right'] is not False
                    or not cfg['port'].endswith('usb-0:1.4:1.0-port0')):
                raise RuntimeError('Confirmed hardware configuration changed')
            port=BoundedDrivePort.open(cfg['port'],19200,max_rpm=a.trial_max_rpm)
        else:port=BoundedDrivePort(MemoryTransport(),max_rpm=a.trial_max_rpm)
        contract_options=dict(enabled=a.enable_motion_input,max_rpm=a.trial_max_rpm,trial_seconds=a.trial_seconds)
        if a.input_kind=='local_avoidance':contract_options['extended_follow']=a.ack_extended_follow_speed
        contract=DriveContract(port,**contract_options)
        if a.input_kind=='local_avoidance':contract.clearance_stop_enabled=not a.disable_clearance_stop
        if a.disable_clearance_stop:node.get_logger().warning('Follower LiDAR clearance stop DISABLED; collision stopping by distance is unavailable')
        if a.run_hardware:
            calibration=Path(__file__).with_name('follower_odometry_calibration.json')
            if calibration.exists():
                from dataclasses import replace
                values=json.loads(calibration.read_text())
                if values.get('version')!=1:raise ValueError('Unsupported odometry calibration version')
                contract.core.config=replace(contract.core.config,raw_linear_feedback_scale=float(values['raw_linear_feedback_scale']))
        if a.ack_distance_test_ignore_clearance:contract.distance_test_only=True
        if a.ack_continuous20_candidate:contract.continuous20=True
        raw_pub=node.create_publisher(Odometry,'/follower/odom_raw',20)
        status_pub=node.create_publisher(String,PREFIX+'/status',10)

        def command(m):
            contract.command(m.data,node.get_clock().now().nanoseconds,require_source=a.input_kind in ('guard','local_avoidance'))

        def arm(request,response):
            epoch=contract.arm(node.get_clock().now().nanoseconds)
            response.success=epoch is not None
            response.message=json.dumps(dict(epoch=epoch,reason=None if epoch else 'LOCKED_OR_FAULT_OR_ZERO_FEEDBACK_NOT_READY'))
            return response

        def disarm(request,response):
            contract.disarm();response.success=True;response.message='Disarmed; old epoch invalidated'
            return response

        command_topic=COMMAND_TOPIC if a.input_kind=='local_avoidance' else PREFIX+('/manual_command' if a.input_kind=='manual' else '/guard_command')
        cmd_sub=node.create_subscription(String,command_topic,command,10)
        arm_srv=node.create_service(Trigger,PREFIX+'/arm',arm)
        disarm_srv=node.create_service(Trigger,PREFIX+'/disarm',disarm)
        last_status=0.;last_fault=None;fault_snapshot_seen=None
        tick_history=deque(maxlen=24);last_finished=None
        def tick():
            nonlocal last_status,last_fault,last_finished,fault_snapshot_seen
            started=time.monotonic()
            profile=dict(start_mono=started,idle_since_previous_end_s=None if last_finished is None else started-last_finished)
            now=node.get_clock().now();before=contract.core.valid_count
            contract.tick(now.nanoseconds)
            profile['core_s']=time.monotonic()-started
            # The control tick processes the fault and requests zero first.
            # Capture before any potentially slow ROS publishing. Never perform
            # filesystem I/O on this callback or wait for the writer queue.
            if fault_writer is not None and contract.core.fault and contract.core.fault!=fault_snapshot_seen:
                snapshot=capture_fault(contract.core,now.nanoseconds,tick_history,profile)
                if not fault_writer.submit(snapshot):
                    node.get_logger().error('Fault evidence queue unavailable; snapshot not queued')
                fault_snapshot_seen=contract.core.fault
            if contract.core.valid_count!=before:
                stage=time.monotonic()
                raw_pub.publish(fill_odom(Odometry(),contract.core,now.to_msg()))
                profile['raw_publish_s']=time.monotonic()-stage
                stage=time.monotonic()
                publish_position_guarded(position,contract,now.to_msg())
                profile['position_publish_s']=time.monotonic()-stage
            if contract.core.clock()-last_status>=.1:
                last_status=contract.core.clock();s=contract.status()
                s['mode']='HARDWARE_CANDIDATE' if a.run_hardware else 'MEMORY_ONLY'
                s['input_kind']=a.input_kind
                s['raw_linear_feedback_scale']=contract.core.config.raw_linear_feedback_scale
                s['last_position_error']=getattr(contract,'last_position_error',None)
                s['status_ros_ns']=now.nanoseconds
                stage=time.monotonic()
                status_pub.publish(String(data=json.dumps(s,allow_nan=False)))
                profile['status_publish_s']=time.monotonic()-stage
                if contract.core.fault and contract.core.fault!=last_fault:
                    node.get_logger().error(json.dumps(dict(s,response_trace=contract.core.last_response_error,
                        feedback_trace=contract.core.last_feedback_error,
                        driver_tick_history=list(tick_history),current_tick=profile),allow_nan=False))
                    last_fault=contract.core.fault
            last_finished=time.monotonic();profile['total_callback_s']=last_finished-started
            tick_history.append(profile)
        timer=node.create_timer(.02,tick)
        node.get_logger().info(('HARDWARE CANDIDATE' if a.run_hardware else 'ISOLATED Domain217 MEMORY ONLY')+
            f': starts disarmed; {a.input_kind} envelope only, <={a.trial_max_rpm}RPM/'+
            ('no session time limit' if a.trial_seconds==0 else f'{a.trial_seconds}s')+
            '; command/feedback deadlines retained; no plain Twist subscription')
        rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        if contract is not None:contract.shutdown()
        elif port is not None:port.close()
        if fault_writer is not None:
            finished=fault_writer.close()
            if not finished or fault_writer.errors:
                node.get_logger().error('Fault evidence writer incomplete: '+str(list(fault_writer.errors)))
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    return 0


if __name__=='__main__':raise SystemExit(main())
