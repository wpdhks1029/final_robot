#!/usr/bin/env python3
"""ONE explicit real follow trial supervisor: arm/disarm only, no velocity publishing.

Requires enabled HARDWARE_CANDIDATE with guard input, fresh forward scene, and
operator clearance/power-cut preparation. Requests disarm after at most 4.5s;
driver independently limits its trial to 5s and its configured RPM cap (default 3).
No automatic rearm. Post-disarm faults are reported separately from zero RPM.
"""
import argparse
import json
import math
import os
from pathlib import Path
import time


def ready_reason(motor,guard,core,now_ns,mono):
    if motor is None:return 'WAIT_MOTOR_STATUS'
    s,rx=motor
    if not 0<=mono-rx<=.3:return 'MOTOR_STATUS_STALE'
    if s.get('mode')!='HARDWARE_CANDIDATE' or s.get('input_kind')!='guard':return 'WRONG_DRIVER_MODE'
    if s.get('enabled') is not True:return 'MOTION_INPUT_DISABLED'
    if s.get('armed') is not False:return 'ALREADY_ARMED_REFUSE'
    if s.get('fault') is not None or s.get('io_failed') is not False:return 'MOTOR_FAULT'
    if s.get('feedback_fresh') is not True or s.get('measured_rpm')!=[0,0]:return 'FRESH_ZERO_NOT_CONFIRMED'
    v,w,reason=core.command(now_ns,mono)
    if v<=0:return reason
    if guard is None:return 'WAIT_GUARD'
    g,rx=guard;stamp=g.header.stamp.sec*10**9+g.header.stamp.nanosec
    if g.header.frame_id!='follower/base_link' or stamp<=0:return 'INVALID_GUARD'
    if not 0<=(now_ns-stamp)/1e9<=.3 or not 0<=mono-rx<=.3:return 'GUARD_STALE_OR_FUTURE'
    v,w=g.twist.linear.x,g.twist.angular.z
    if (not math.isfinite(v) or not math.isfinite(w) or not 0<v<=.1 or abs(w)>.2
            or v<abs(w)*.560/2):return 'GUARD_NOT_FORWARD'
    return 'READY'


def trial_succeeded(reason,arm_requested,confirmed,stop_faults):
    return reason=='TRIAL_TIME_LIMIT' and arm_requested and confirmed and not stop_faults


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run-5s-trial',action='store_true')
    p.add_argument('--ack-clear-path-power-cut-ready',action='store_true')
    a=p.parse_args()
    if not a.run_5s_trial:p.print_help();return 0
    if not a.ack_clear_path_power_cut_ready:p.error('Operator preparation acknowledgment required')
    if os.environ.get('ROS_DOMAIN_ID')!='10':p.error('ROS_DOMAIN_ID must be 10')
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from nav_msgs.msg import Odometry,Path as RosPath
    from geometry_msgs.msg import TwistStamped
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from follower_pursuit_preview import PreviewCore
    from follower_remote_qos import latest_remote_qos
    rclpy.init(args=[],signal_handler_options=SignalHandlerOptions.NO)
    node=rclpy.create_node('guarded_follow_trial_operator');core=PreviewCore()
    motor=None;guard=None;guard_diagnostic=None;arm_requested=False;stop_reason='NOT_STARTED';confirmed=False
    stop_faults=set();exit_code=1;stopping=False
    class TrialAbort(Exception):pass
    arm=node.create_client(Trigger,'/follower/motor_candidate/arm')
    disarm=node.create_client(Trigger,'/follower/motor_candidate/disarm')
    directory=Path(__file__).parent/'logs';directory.mkdir(exist_ok=True)
    path=directory/f'guarded_follow_trial_{time.time_ns()}.jsonl';stream=path.open('x')
    def log(event,**fields):
        stream.write(json.dumps(dict(event=event,ros_ns=node.get_clock().now().nanoseconds,**fields),allow_nan=False)+'\n')
        stream.flush()
    def status_cb(m):
        nonlocal motor
        try:
            s=json.loads(m.data);motor=(s,time.monotonic());log('status',status=s)
            if stopping and s.get('fault'):stop_faults.add(str(s['fault']))
        except ValueError:motor=None
    def guard_cb(m):
        nonlocal guard
        guard=(m,time.monotonic())
    def diagnostic_cb(m):
        nonlocal guard_diagnostic
        try:
            state=json.loads(m.data);guard_diagnostic=state
            log('guard_diagnostic',diagnostic=state)
        except ValueError:pass
    def spin():rclpy.spin_once(node,timeout_sec=.02)
    def call(client):
        if not client.wait_for_service(timeout_sec=1.):raise RuntimeError('Service unavailable')
        future=client.call_async(Trigger.Request());end=time.monotonic()+1.5
        while not future.done() and time.monotonic()<end:spin()
        if not future.done():raise RuntimeError('Service response timeout')
        return future.result()
    subs=[node.create_subscription(String,'/follower/motor_candidate/status',status_cb,1),
          node.create_subscription(String,'/follower/obstacle_preview/status',diagnostic_cb,20),
          node.create_subscription(TwistStamped,'/follower/cmd_vel_guard_stamped_test',guard_cb,latest_remote_qos()),
          node.create_subscription(RosPath,'/leader/path',lambda m:core.path_callback(m,node.get_clock().now().nanoseconds,time.monotonic()),latest_remote_qos()),
          node.create_subscription(Odometry,'/leader/odom',lambda m:core.odom_callback('leader',m,node.get_clock().now().nanoseconds,time.monotonic()),latest_remote_qos()),
          node.create_subscription(Odometry,'/follower/odom_map',lambda m:core.odom_callback('follower',m,node.get_clock().now().nanoseconds,time.monotonic()),latest_remote_qos())]
    try:
        print('EXPLICIT ONE-TRIAL MODE: waiting up to 15s for fresh forward scene. Will arm ONCE when READY.',flush=True)
        print('TRIAL_LOG='+str(path),flush=True)
        end=time.monotonic()+15.;last_reason=None
        while time.monotonic()<end:
            spin();reason=ready_reason(motor,guard,core,node.get_clock().now().nanoseconds,time.monotonic())
            if reason=='READY':
                output='/follower/motor_candidate/guard_command'
                writers=node.get_publishers_info_by_topic(output)
                readers=node.get_subscriptions_info_by_topic(output)
                if not (len(writers)==1 and writers[0].node_name=='follower_guard_gateway'
                        and len(readers)==1 and readers[0].node_name=='follower_drive_candidate'):
                    reason='WAIT_GATEWAY_DRIVE_CONNECTION'
            if reason!=last_reason:print(reason,flush=True);last_reason=reason
            if reason=='READY':break
            if reason in ('MOTOR_FAULT','ALREADY_ARMED_REFUSE','MOTION_INPUT_DISABLED','WRONG_DRIVER_MODE'):
                raise TrialAbort(reason)
        else:raise TrialAbort('READY_TIMEOUT_NO_ARM')
        # An attempted arm must always be followed by a disarm attempt, even if its response is lost.
        arm_requested=True;started=time.monotonic();response=call(arm)
        if not response.success:raise TrialAbort('ARM_REJECTED')
        epoch=json.loads(response.message)['epoch'];log('arm_accepted',epoch=epoch)
        print('ARMED: 최대 4.5초 감독 창; 드라이버 5초 제한과 설정된 RPM 상한 유지.',flush=True)
        while time.monotonic()-started<4.5:
            spin();mono=time.monotonic();now=node.get_clock().now().nanoseconds
            if motor is None or mono-motor[1]>.3:stop_reason='MOTOR_STATUS_LOST';break
            s=motor[0]
            if s.get('fault') or not s.get('feedback_fresh'):stop_reason='MOTOR_FAULT_OR_FEEDBACK_LOST';break
            if s.get('armed') and s.get('epoch')!=epoch:stop_reason='SESSION_CHANGED';break
            v,w,reason=core.command(now,mono)
            if v<=0:stop_reason=reason;break
            if guard is None or mono-guard[1]>.3:stop_reason='GUARD_LOST';break
            g=guard[0];stamp=g.header.stamp.sec*10**9+g.header.stamp.nanosec
            if stamp<=0 or not 0<=(now-stamp)/1e9<=.3:stop_reason='GUARD_SOURCE_INVALID';break
            if g.twist.linear.x<=0:
                stop_reason='GUARD_STOP'
                log('guard_stop_observed',guard_source_ns=stamp,
                    guard_v=g.twist.linear.x,guard_w=g.twist.angular.z,
                    last_diagnostic=guard_diagnostic)
                break
        else:stop_reason='TRIAL_TIME_LIMIT'
        exit_code=0
    except TrialAbort as exc:
        stop_reason=str(exc)
    except KeyboardInterrupt:
        stop_reason='OPERATOR_INTERRUPT'
    except Exception as exc:
        stop_reason='SUPERVISOR_ERROR';log('supervisor_error',error=str(exc))
    finally:
        if arm_requested:
            stopping=True
            try:
                response=call(disarm);log('disarm_response',success=response.success)
                settle_until=time.monotonic()+1.
                end=time.monotonic()+2.;last_count=None if motor is None else motor[0].get('valid_feedback');zeros=0
                while time.monotonic()<end:
                    spin()
                    if motor is None:continue
                    s,rx=motor;count=s.get('valid_feedback')
                    if count==last_count:continue
                    last_count=count
                    zero=(time.monotonic()-rx<=.3 and s.get('armed') is False and s.get('feedback_fresh') is True
                          and s.get('io_failed') is False and s.get('measured_rpm')==[0,0])
                    zeros=zeros+1 if zero else 0
                    if zeros>=3 and time.monotonic()>=settle_until:confirmed=True;break
            except Exception as exc:log('stop_confirmation_error',error=str(exc))
        print('END_REASON='+stop_reason,flush=True)
        if arm_requested:
            print('FRESH_ZERO_FEEDBACK_CONFIRMED='+str(confirmed),flush=True)
            print('POST_STOP_FAULTS='+json.dumps(sorted(stop_faults)),flush=True)
            if not confirmed:print('STOP NOT VERIFIED: if robot is not physically stopped, cut motor power immediately.',flush=True)
        else:print('No arm request sent.',flush=True)
        success=trial_succeeded(stop_reason,arm_requested,confirmed,stop_faults)
        print('TRIAL_SUCCESS='+str(success),flush=True)
        if not success:exit_code=1
        log('end',reason=stop_reason,arm_requested=arm_requested,fresh_zero_confirmed=confirmed,
            post_stop_faults=sorted(stop_faults),trial_success=success)
        stream.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    return exit_code


if __name__=='__main__':raise SystemExit(main())
