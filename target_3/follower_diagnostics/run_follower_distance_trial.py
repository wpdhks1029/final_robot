#!/usr/bin/env python3
"""Follower-only1m forward distance comparison. Default opens nothing."""
import argparse,json,math,os,time
from follower_yaw_diagnostics import angle_delta,pivot_request,rotation_circle_diagnostic
from watch_follower_yaw import attach,save
from local_avoidance_drive import PROFILE,COMMAND_TOPIC
from local_avoidance_limits import SOURCE_TIMEOUT_S
from run_local_avoidance_trial import motor_problem as normal_motor_problem
COMMAND_TOPIC="/follower/motor_candidate/distance_test_command"

def motor_problem(sample,mono,seconds,stationary=False):
    if sample is None or sample[0].get("distance_test_only") is not True:return "REQUIRE_DISTANCE_TEST_DRIVER_RESTART"
    state=dict(sample[0]);state.pop("distance_test_only",None)
    return normal_motor_problem((state,sample[1]),mono,seconds,stationary)


def distance_request(along,lateral,raw,imu,elapsed,target=1.):
    if not all(math.isfinite(x) for x in (along,lateral,raw,imu,elapsed,target)):return 0.,'INVALID_INPUT'
    if abs(lateral)>.15:return 0.,'LATERAL_DRIFT_OVER_15CM'
    if max(abs(raw),abs(imu))>math.radians(15):return 0.,'HEADING_DRIFT_OVER_15_DEG'
    if along<-.03:return 0.,'OPPOSITE_TRANSLATION'
    if along>=target:return 0.,'DISTANCE_TARGET_REACHED'
    if elapsed>=45:return 0.,'DISTANCE_TIME_LIMIT'
    return (.02 if target-along<.1 else .04),'DRIVE'


def main():
    p=argparse.ArgumentParser(description=__doc__);m=p.add_mutually_exclusive_group()
    m.add_argument('--run',action='store_true');m.add_argument('--plan-only',action='store_true')
    p.add_argument('--ack-clear-path-power-cut-ready',action='store_true')
    p.add_argument('--distance-m',type=float,choices=(1.,),default=1.)
    a=p.parse_args()
    target_info=dict(target_distance_m=a.distance_m)
    if not a.run and not a.plan_only:p.print_help();return 0
    if os.environ.get('ROS_DOMAIN_ID')!='10' or (a.run and not a.ack_clear_path_power_cut_ready):
        p.error('ROS_DOMAIN_ID=10 and explicit pivot-area acknowledgement required for --run')
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import QoSProfile,ReliabilityPolicy
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from watch_apf_case3 import project
    from leader_short_forward_trial import wait_for_start
    rclpy.init(args=[],signal_handler_options=SignalHandlerOptions.NO)
    node=rclpy.create_node('follower_distance_trial_operator');watch=attach(node)
    publisher=None if a.plan_only else node.create_publisher(String,COMMAND_TOPIC,QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE))
    arm=None if a.plan_only else node.create_client(Trigger,'/follower/motor_candidate/arm')
    disarm=None if a.plan_only else node.create_client(Trigger,'/follower/motor_candidate/disarm')
    arm_requested=False;epoch=None;zero_confirmed=False;reason='NOT_STARTED';last_control=0.;turn_started=None
    measurement_begin_ns=None
    class Abort(Exception):pass
    def now():return node.get_clock().now().nanoseconds
    def spin():
        if not rclpy.ok():raise Abort('ROS_SHUTDOWN')
        rclpy.spin_once(node,timeout_sec=.01)
    def graph():
        pubs=node.get_publishers_info_by_topic(COMMAND_TOPIC)
        if a.run and (len(pubs)!=1 or pubs[0].node_name!=node.get_name()):return False
        if a.plan_only and pubs:return False
        subs=node.get_subscriptions_info_by_topic(COMMAND_TOPIC)
        return len(subs)==1 and subs[0].node_name=='follower_drive_candidate'
    def scene(stationary=False):
        ns=now();mono=time.monotonic();sample=watch.motor
        seconds=sample[0].get('trial_max_seconds') if sample else None
        if seconds not in (30,60,360):raise ValueError('REQUIRE_LOCAL_AVOIDANCE_DRIVER')
        problem=motor_problem(sample,mono,seconds,stationary)
        if problem:raise ValueError(problem)
        if not graph() or watch.mount is None:raise ValueError('GRAPH_OR_MOUNT_NOT_READY')
        stamps=[]
        for key in ('raw','filtered','imu'):
            row=watch.latest.get(key)
            if row is None or not 0<=(ns-row['stamp_ns'])/1e9<=SOURCE_TIMEOUT_S or not 0<=mono-row['rx_mono']<=SOURCE_TIMEOUT_S:
                raise ValueError(key.upper()+'_MISSING_OR_STALE')
            stamps.append(row['stamp_ns'])
        raw=watch.latest['raw'];imu=watch.latest['imu']
        if not all(math.isfinite(raw[k]) for k in ('x','y','v','w')):raise ValueError('RAW_INVALID')
        if stationary and (abs(raw['v'])>.01 or abs(raw['w'])>.01):raise ValueError('REQUIRE_STATIONARY_RAW')
        if max(abs(imu['roll']),abs(imu['pitch']))>math.radians(20):raise ValueError('IMU_MOUNT_TILT_OVER_20_DEG')
        if watch.scan is None:raise ValueError('SCAN_MISSING')
        msg,rx=watch.scan;stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        if msg.header.frame_id!='follower/laser' or not 0<=(ns-stamp)/1e9<=SOURCE_TIMEOUT_S or not 0<=mono-rx<=SOURCE_TIMEOUT_S:
            raise ValueError('SCAN_INVALID_OR_STALE')
        if len(msg.ranges)<180 or msg.angle_max-msg.angle_min<math.radians(330):raise ValueError('SCAN_COVERAGE_INSUFFICIENT')
        points=project(msg,watch.mount)
        if len(points)<100:raise ValueError('SCAN_VALID_POINTS_INSUFFICIENT')
        from apf_pivot_candidate import swept_clearance
        # Check the body and forward stopping corridor, not a rotation circle.
        clearance=swept_clearance(points,[(x,0.,0.) for x in (0.,.02,.04,.06,.08,.10,.12)])-.02
        watch.clearance_diagnostic=dict(forward_corridor_clearance_m=clearance,scan_stamp_ns=stamp)
        if clearance<=.30:watch.failed_scan=(msg,ns)
        # Explicit distance-test driver records actual clearance without using it as a motion gate.
        return dict(raw=raw,imu=imu,clearance=clearance,source=min(stamps+[stamp]),scan_stamp=stamp)
    def publish(s,v=0.):
        sent=now()
        if not 0<=(sent-s['source'])/1e9<=SOURCE_TIMEOUT_S:raise Abort('SOURCE_AGED_DURING_COMPUTE')
        publisher.publish(String(data=json.dumps(dict(version=3,controller=PROFILE,epoch=epoch,
            frame='follower/base_link',stamp_ns=sent,source_stamp_ns=s['source'],scan_stamp_ns=s['scan_stamp'],
            motion='DRIVE' if v else 'STOP',v=v,w=0.,clearance_m=s['clearance'],swept_clearance_m=s['clearance']))))
    def call(client):
        if not client.service_is_ready():raise Abort('SERVICE_UNAVAILABLE')
        f=client.call_async(Trigger.Request());until=time.monotonic()+1.
        while not f.done() and time.monotonic()<until:spin()
        if not f.done() or not f.result().success:raise Abort('SERVICE_FAILED')
        return f.result()
    print('후발만 오뎀 기준1m 직진 비교. 선두 정지 유지. 아직 주행/arm 요청 없음.',flush=True)
    try:
        end=time.monotonic()+10.;problem='WAIT_INPUTS'
        while time.monotonic()<end:
            spin()
            try:s=scene(True);break
            except ValueError as exc:problem=str(exc)
        else:raise Abort('PREFLIGHT_'+problem)
        print(json.dumps(dict(plan_only=a.plan_only,**target_info,max_seconds=45,max_v=.04,clearance_m=s['clearance']),ensure_ascii=False),flush=True)
        if a.plan_only:raise Abort('PLAN_ONLY_READY')
        print('START + Enter: 후발만1m 직진. 바닥에 차체 시작점을 표시하고 실제 이동 거리를 재세요.',flush=True)
        if not wait_for_start(spin):raise Abort('OPERATOR_CANCELLED')
        s=scene(True);base_raw=dict(s['raw']);base_imu=dict(s['imu']);distance_base={k:dict(watch.latest[k]) for k in ('raw','filtered')}
        measurement_begin_ns=max(base_raw['stamp_ns'],base_imu['stamp_ns'])
        arm_requested=True;epoch=json.loads(call(arm).message)['epoch'];armed=time.monotonic()
        watch.events.append(dict(event='baseline',raw=base_raw,imu=base_imu,**target_info))
        while True:
            spin();mono=time.monotonic();s=scene();state=watch.motor[0]
            if state.get('armed') is not True:
                if mono-armed>.2:raise Abort('ARM_STATUS_NOT_CONFIRMED')
                continue
            if state.get('epoch')!=epoch:raise Abort('SESSION_CHANGED')
            if min(s['source'],s['scan_stamp'])<state['arm_ros_ns']:
                if mono-armed>SOURCE_TIMEOUT_S:raise Abort('POST_ARM_INPUT_NOT_READY')
                publish(s);continue
            raw=angle_delta(s['raw']['yaw'],base_raw['yaw']);imu=angle_delta(s['imu']['yaw'],base_imu['yaw'])
            dx=s['raw']['x']-base_raw['x'];dy=s['raw']['y']-base_raw['y']
            co,si=math.cos(base_raw['yaw']),math.sin(base_raw['yaw'])
            along=co*dx+si*dy;lateral=-si*dx+co*dy
            if turn_started is None:turn_started=mono
            v,policy=distance_request(along,lateral,raw,imu,mono-turn_started,a.distance_m)
            if mono-last_control>=.1:
                watch.events.append(dict(event='control',ros_ns=now(),forward_m=along,lateral_m=lateral,
                    raw_delta_deg=math.degrees(raw),imu_delta_deg=math.degrees(imu),requested_v=v,reason=policy))
                last_control=mono
            if policy!='DRIVE':publish(s);raise Abort(policy)
            publish(s,v)
    except (Abort,ValueError,KeyError,TypeError) as exc:reason=str(exc)
    except (KeyboardInterrupt,EOFError):reason='OPERATOR_STOP'
    except Exception as exc:reason='DISTANCE_TOOL_ERROR';watch.events.append(dict(event='exception',detail=repr(exc)))
    finally:
        if arm_requested:
            try:
                if disarm.service_is_ready():disarm.call_async(Trigger.Request())
            except Exception:pass
            end=time.monotonic()+3.;settled=None
            while rclpy.ok() and time.monotonic()<end:
                try:spin()
                except Abort:break
                if watch.motor is not None:
                    state,rx=watch.motor
                    if time.monotonic()-rx<=.3 and state.get('armed') is False and state.get('measured_rpm')==[0,0] and state.get('zero_feedback_samples',0)>=3 and state.get('feedback_fresh'):
                        if settled is None:settled=time.monotonic()
                        if time.monotonic()-settled>=.5:zero_confirmed=True;break
                    else:settled=None
        distances={}
        if arm_requested:
            for key,b in distance_base.items():
                e=watch.latest.get(key)
                if e is None:continue
                dx=e['x']-b['x'];dy=e['y']-b['y'];co,si=math.cos(b['yaw']),math.sin(b['yaw'])
                distances[key]=dict(forward_m=co*dx+si*dy,lateral_m=-si*dx+co*dy,displacement_m=math.hypot(dx,dy))
        save(watch,'follower_distance_trial',dict(reason=reason,arm_requested=arm_requested,distance_sources=distances,
            follower_zero_confirmed=zero_confirmed,**target_info,
            measurement_begin_ns=measurement_begin_ns,
            trial_success=reason=='DISTANCE_TARGET_REACHED' and zero_confirmed,leader_commanded=False,
            automatic_calibration_applied=False,plan_only=a.plan_only))
        if arm_requested and not zero_confirmed:print('후발 정지 피드백 미확인. 실물이 멈추지 않으면 전원을 차단하세요.',flush=True)
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    return 0 if reason=='PLAN_ONLY_READY' or (reason=='DISTANCE_TARGET_REACHED' and zero_confirmed) else 1

if __name__=='__main__':raise SystemExit(main())
