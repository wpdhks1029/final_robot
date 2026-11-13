#!/usr/bin/env python3
"""후발 한 터미널에서 선두 직진·후발 제한 추종·양쪽 정지를 요청하는 시험 도구.

실제 동시 기동/제동은 보장하지 않는다. 선두0.04/0.05m/s 명령 최대8초/14초/34초/64초,
유효 경로가 생긴 뒤 후발을 한 번 arm하고 최대4.5초/9.5초/29.5초/59.5초 후 양쪽 정지 요청.
후발은 guard만 사용하며 드라이버의 별도 시간/RPM 제한을 유지한다.
"""
import argparse
from collections import deque
import json
import math
import os
from pathlib import Path
import time

from leader_short_forward_trial import wait_for_start


def fresh_sample(sample,now_ns,mono,*,remote=False):
    if sample is None:return False
    m,rx=sample;stamp=m.header.stamp.sec*10**9+m.header.stamp.nanosec
    return (stamp>0 and (-.05 if remote else 0)<=(now_ns-stamp)/1e9<=.3
            and 0<=mono-rx<=.3)


def motor_reason(sample,mono,*,stationary,trial_seconds=5):
    if sample is None:return 'WAIT_MOTOR'
    s,rx=sample
    if not 0<=mono-rx<=.3:return 'MOTOR_STATUS_STALE'
    if (s.get('mode')!='HARDWARE_CANDIDATE' or s.get('input_kind')!='guard'
            or s.get('trial_max_rpm')!=10 or s.get('enabled') is not True):
        return 'REQUIRE_ENABLED_GUARD_10RPM'
    if s.get('trial_max_seconds',5)!=trial_seconds:return 'DRIVER_TRIAL_SECONDS_MISMATCH'
    if s.get('fault') or s.get('io_failed') is not False:return 'MOTOR_FAULT'
    if s.get('feedback_fresh') is not True:return 'FEEDBACK_STALE'
    if stationary and (s.get('armed') is not False or s.get('measured_rpm')!=[0,0]):
        return 'REQUIRE_DISARMED_ZERO'
    return None


def scene_reason(core,guard,now_ns,mono,*,temporary_detour=False):
    v,w,reason=core.speed_gate(now_ns,mono) if temporary_detour else core.command(now_ns,mono)
    if v<=0:return reason
    if not fresh_sample(guard,now_ns,mono):return 'GUARD_SOURCE_OR_RX_INVALID'
    g=guard[0];v,w=g.twist.linear.x,g.twist.angular.z
    other=(g.twist.linear.y,g.twist.linear.z,g.twist.angular.x,g.twist.angular.y)
    if (g.header.frame_id!='follower/base_link' or not all(math.isfinite(x) for x in (v,w,*other))
            or any(abs(x)>1e-9 for x in other) or not 0<v<=.1 or abs(w)>.2 or v<abs(w)*.560/2):
        return 'GUARD_STOP_OR_INVALID'
    return 'READY'


def detour_preflight_problem(diagnostic):
    reason=diagnostic.get('reason','')
    if reason in ('STOP_SESSION_FINISHED','STOP_SESSION_CHANGED','STOP_ARMED_BEFORE_PLAN_READY',
                  'STOP_LOCAL_TRIAL_TIME_LIMIT','STOP_LOCALIZATION_JUMP','STOP_INITIAL_PLAN_POSE_MISMATCH',
                  'STOP_REQUIRE_FRESH_MATCHING_GUARD_DRIVER'):
        return reason
    return None


def leader_turn_rate(direction):
    return {'straight':0.,'left':.03,'right':-.03}[direction]


def obstacle_stop_evidence(diagnostic,rx_mono,failure_ros,failure_mono,guard):
    """같은 원본 시각의 최신 전방 장애물 정지 진단만 시험 증거로 인정한다."""
    try:
        if not guard or guard['frame']!='follower/base_link' or not guard['zero']:return None
        stamp=diagnostic['stamp_ns'];source=diagnostic['source_stamp_ns'];scan=diagnostic['scan_source_stamp_ns']
        if any(type(t) is not int or t<=0 for t in (stamp,source,scan)):return None
        if (abs(failure_mono-rx_mono)>.3 or abs(failure_ros-stamp)>300_000_000
                or source!=guard['source_ns'] or not 0<=(failure_ros-source)/1e9<=.3
                or not 0<=(failure_ros-scan)/1e9<=.3):return None
        if diagnostic['stamped_reason']!='STOP_FOOTPRINT_CLEARANCE' or diagnostic['clearance_zone']!='stop':return None
        d=diagnostic['clearance_m']
        if not math.isfinite(d) or not 0<d<=.3:return None
        if diagnostic['output_v']!=0 or diagnostic['output_w']!=0:return None
        front=[p for p in diagnostic['nearest_points']
               if all(math.isfinite(p[k]) for k in ('base_x_m','base_y_m','clearance_m'))
               and p['base_x_m']>=.27 and abs(p['base_y_m'])<=.30 and 0<p['clearance_m']<=.3]
        if not front:return None
        return dict(clearance_m=d,source_ns=source,scan_source_ns=scan,front_points=len(front))
    except (KeyError,TypeError,ValueError,OverflowError):return None


def command_readers_ok(readers,allow_recorder=False):
    """제어기 1개를 필수로 하며 명시한 경우에만 루트 기록기 1개를 허용한다."""
    controllers=[r for r in readers if r.node_name=='md200t_controller'
                 and getattr(r,'node_namespace','/')=='/']
    recorders=[r for r in readers if r.node_name=='rosbag2_recorder'
               and getattr(r,'node_namespace','/')=='/']
    return (len(controllers)==1 and len(recorders)<=(1 if allow_recorder else 0)
            and len(readers)==len(controllers)+len(recorders))


def relative_scene(core):
    inputs=getattr(core,'inputs',{})
    if 'leader' not in inputs or 'follower' not in inputs:return None
    l=inputs['leader'][2];f=inputs['follower'][2]
    dx,dy=l[0]-f[0],l[1]-f[1];distance=math.hypot(dx,dy)
    if not math.isfinite(distance):return None
    return dict(leader_xy=list(l[:2]),follower_xy=list(f[:2]),distance_m=distance,
                ahead_m=math.cos(f[2])*dx+math.sin(f[2])*dy,
                left_m=-math.sin(f[2])*dx+math.cos(f[2])*dy,
                leader_v=l[3],follower_v=f[3],leader_yaw_rad=l[2],follower_yaw_rad=f[2],
                heading_difference_rad=math.atan2(math.sin(l[2]-f[2]),math.cos(l[2]-f[2])),
                path_points=len(inputs['path'][2]) if 'path' in inputs else 0)


class Window:
    """시간 제한만 계산하며 ROS/모터에 접근하지 않는다."""
    def __init__(self,trial_seconds=5):
        if type(trial_seconds) is not int or trial_seconds not in (5,10,30,60):
            raise ValueError('시험 시간 상한은 5초, 10초, 30초, 60초 중 하나여야 합니다.')
        self.started=None;self.armed=None
        self.follow_limit=trial_seconds-.5
        self.leader_limit={5:8.,10:14.,30:34.,60:64.}[trial_seconds]
    def start(self,now):self.started=now
    def follow(self,now):self.armed=now
    def expired(self,now):
        if self.started is not None and now-self.started>=self.leader_limit:return 'LEADER_TIME_LIMIT'
        if self.armed is not None and now-self.armed>=self.follow_limit:return 'TRIAL_TIME_LIMIT'
        if self.started is not None and self.armed is None and now-self.started>=4.:return 'FORWARD_SCENE_TIMEOUT'
        return None


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run',action='store_true')
    p.add_argument('--ack-clear-both-power-cut-ready',action='store_true')
    p.add_argument('--trial-seconds',type=int,choices=[5,10,30,60],default=5)
    p.add_argument('--leader-speed',type=float,choices=[.04,.05],default=.05)
    p.add_argument('--leader-turn',choices=['straight','left','right'],default='straight')
    p.add_argument('--trace-timing',action='store_true',help='최근 입력 시각을 메모리에 기록하고 정지 확인 뒤 로그에 저장')
    p.add_argument('--allow-rosbag-recorder',action='store_true',help='/cmd_vel의 rosbag2_recorder 구독 1개를 추가 허용')
    p.add_argument('--expect-obstacle-stop',action='store_true',help='전방 장애물 정지 진단과 양쪽 정지가 확인돼야 통과. 제어 조건은 유지')
    p.add_argument('--temporary-detour',action='store_true',help='검증한 임시 경로의 5/30초 직진 선두 시험 전용')
    a=p.parse_args()
    if a.temporary_detour and (a.trial_seconds not in (5,30) or a.leader_speed!=.04 or a.leader_turn!='straight' or a.expect_obstacle_stop):
        p.error('임시 경로는 --trial-seconds 5 또는 30, --leader-speed 0.04 --leader-turn straight 전용입니다.')
    detour_hash=None
    if a.temporary_detour:
        from follower_temporary_detour_preview import load_plan,PLAN,KIND,NODE,TOPICS,STATUS
        _,detour_hash=load_plan(PLAN)
    if a.expect_obstacle_stop and a.leader_turn!='straight':p.error('장애물 정지 확인은 straight만 사용합니다.')
    if not a.run:p.print_help();return 0
    if not a.ack_clear_both_power_cut_ready:p.error('양쪽 빈 경로와 전원 차단 준비 확인이 필요합니다.')
    if os.environ.get('ROS_DOMAIN_ID')!='10':p.error('ROS_DOMAIN_ID=10이 필요합니다.')
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from geometry_msgs.msg import Twist,TwistStamped
    from nav_msgs.msg import Odometry,Path as RosPath
    from sensor_msgs.msg import LaserScan
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from follower_pursuit_preview import PreviewCore
    from follower_remote_qos import latest_remote_qos,latest_state_qos
    rclpy.init(args=[],signal_handler_options=SignalHandlerOptions.NO)
    node=rclpy.create_node('joint_follow_trial_operator');core=PreviewCore()
    motor=None;guard=None;samples={};window=Window(a.trial_seconds)
    leader_started=False;arm_requested=False;stopping=False;epoch=None
    reason='NOT_STARTED';faults=set();leader_zero=False;follower_zero=False;last_publish=0.;last_scene=0.
    directory=Path(__file__).parent/'logs';directory.mkdir(exist_ok=True)
    path=directory/f'joint_follow_trial_{time.time_ns()}.jsonl';stream=path.open('x')
    timing=deque(maxlen=12000 if a.trial_seconds==60 else 6000)
    diagnostics=deque(maxlen=64);failure=None
    def trace(name,m):
        if not a.trace_timing:return
        stamp=m.header.stamp.sec*10**9+m.header.stamp.nanosec
        timing.append(dict(name=name,source_ns=stamp,rx_ros_ns=node.get_clock().now().nanoseconds,
                           rx_mono_s=time.monotonic()))
    pub=node.create_publisher(Twist,'/cmd_vel',1)
    arm=node.create_client(Trigger,'/follower/motor_candidate/arm')
    disarm=node.create_client(Trigger,'/follower/motor_candidate/disarm')
    class Abort(Exception):pass

    def log(event,**kw):
        stream.write(json.dumps(dict(event=event,ros_ns=node.get_clock().now().nanoseconds,**kw),allow_nan=False)+'\n');stream.flush()
    def receive(name,m):
        trace(name,m)
        samples[name]=(m,time.monotonic())
        if name=='path':core.path_callback(m,node.get_clock().now().nanoseconds,time.monotonic())
        elif name in ('leader','follower'):core.odom_callback(name,m,node.get_clock().now().nanoseconds,time.monotonic())
    def motor_cb(m):
        nonlocal motor
        try:
            state=json.loads(m.data);motor=(state,time.monotonic());log('motor',status=state)
            if stopping and state.get('fault'):faults.add(str(state['fault']))
        except ValueError:motor=None
    def guard_cb(m):
        nonlocal guard
        trace('guard',m)
        guard=(m,time.monotonic())
        stamp=m.header.stamp.sec*10**9+m.header.stamp.nanosec
        log('guard_received',source_ns=stamp,source_age_s=None if stamp<=0 else
            (node.get_clock().now().nanoseconds-stamp)/1e9,v=m.twist.linear.x,w=m.twist.angular.z)
    def diagnostic_cb(m):
        try:
            d=json.loads(m.data);diagnostics.append((d,time.monotonic()))
            log('guard_diagnostic',diagnostic=d)
        except ValueError:pass
    subs=[node.create_subscription(String,'/follower/motor_candidate/status',motor_cb,1),
          node.create_subscription(TwistStamped,'/follower/cmd_vel_guard_stamped_test',guard_cb,latest_state_qos()),
          node.create_subscription(String,'/follower/obstacle_preview/status',diagnostic_cb,latest_state_qos())]
    for name,topic,kind,qos in [('leader','/leader/odom',Odometry,latest_remote_qos()),('path','/leader/path',RosPath,latest_remote_qos()),
                               ('follower','/follower/odom_map',Odometry,latest_state_qos()),('scan','/follower/scan',LaserScan,latest_state_qos())]:
        subs.append(node.create_subscription(kind,topic,lambda m,n=name:receive(n,m),qos))
    if a.trace_timing:
        from tf2_msgs.msg import TFMessage
        from rclpy.qos import QoSProfile,ReliabilityPolicy
        for name,topic,kind in [('position_input','/follower/odom_position_candidate',Odometry),
                                ('filtered','/follower/odometry/filtered',Odometry),
                                ('pursuit','/follower/cmd_vel_stamped_test',TwistStamped)]:
            subs.append(node.create_subscription(kind,topic,lambda m,n=name:trace(n,m),latest_state_qos()))
        def trace_tf(m):
            for t in m.transforms:
                if (t.header.frame_id,t.child_frame_id)==('follower/odom','follower/base_link'):
                    trace('tf_odom_base',t)
                elif (t.header.frame_id,t.child_frame_id)==('map','follower/odom'):
                    trace('tf_map_odom',t)
        subs.append(node.create_subscription(TFMessage,'/tf',trace_tf,
                    QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT)))

    def leader_command(v,w=0.):
        msg=Twist();msg.linear.x=v;msg.angular.z=w;pub.publish(msg)
    def spin():
        nonlocal last_publish,last_scene
        rclpy.spin_once(node,timeout_sec=.02)
        now=time.monotonic()
        if leader_started and now-last_publish>=.05:
            if not stopping and now-window.started<window.leader_limit:
                leader_command(a.leader_speed,leader_turn_rate(a.leader_turn))
            else:leader_command(0.)
            last_publish=now
        if leader_started and now-last_scene>=.2:
            log('scene',scene=relative_scene(core));last_scene=now
    def graph_ok():
        writers=node.get_publishers_info_by_topic('/cmd_vel')
        readers=node.get_subscriptions_info_by_topic('/cmd_vel')
        gw=node.get_publishers_info_by_topic('/follower/motor_candidate/guard_command')
        drive=node.get_subscriptions_info_by_topic('/follower/motor_candidate/guard_command')
        if a.temporary_detour:
            for topic in TOPICS+(STATUS,):
                entries=node.get_publishers_info_by_topic(topic)
                if len(entries)!=1 or entries[0].node_name!=NODE:return False
            if not diagnostics:return False
            d,rx=diagnostics[-1];now=node.get_clock().now().nanoseconds
            if (d.get('controller_kind')!=KIND or d.get('plan_sha256')!=detour_hash
                    or d.get('trial_max_seconds')!=a.trial_seconds or not 0<=time.monotonic()-rx<=.3
                    or not 0<=(now-d.get('stamp_ns',0))/1e9<=.3):return False
        if a.expect_obstacle_stop:
            diagnostic_writers=node.get_publishers_info_by_topic('/follower/obstacle_preview/status')
            if len(diagnostic_writers)!=1 or diagnostic_writers[0].node_name!='follower_obstacle_preview':return False
        return (len(writers)==1 and writers[0].node_name==node.get_name()
                and command_readers_ok(readers,a.allow_rosbag_recorder)
                and len(gw)==1 and gw[0].node_name=='follower_guard_gateway'
                and len(drive)==1 and drive[0].node_name=='follower_drive_candidate')
    def current_scene(ros,mono):
        result=scene_reason(core,guard,ros,mono,temporary_detour=a.temporary_detour)
        if a.temporary_detour and result=='READY':
            if not diagnostics or diagnostics[-1][0].get('reason')!='DETOUR_READY':
                return 'STOP_TEMPORARY_DETOUR_NOT_READY'
        return result

    def inputs_ok():
        now=node.get_clock().now().nanoseconds;mono=time.monotonic()
        for name in ('leader','path','follower','scan'):
            if not fresh_sample(samples.get(name),now,mono,remote=name in ('leader','path')):return False
        l=samples['leader'][0];f=samples['follower'][0];scan=samples['scan'][0]
        return (l.header.frame_id=='map' and l.child_frame_id=='base_link'
                and f.header.frame_id=='map' and f.child_frame_id=='follower/base_link'
                and samples['path'][0].header.frame_id=='map'
                and scan.header.frame_id=='follower/laser'
                and any(math.isfinite(r) and scan.range_min<=r<=scan.range_max for r in scan.ranges))
    def stationary():
        l=samples['leader'][0]
        return all(math.isfinite(v) and abs(v)<=.01 for v in
                   (l.twist.twist.linear.x,l.twist.twist.angular.z))
    def snapshot():
        ros=node.get_clock().now().nanoseconds;mono=time.monotonic()
        ages={}
        for name,sample in samples.items():
            m,rx=sample;stamp=m.header.stamp.sec*10**9+m.header.stamp.nanosec
            ages[name]=dict(source_age_s=(ros-stamp)/1e9,rx_age_s=mono-rx)
        return dict(motor_reason=motor_reason(motor,mono,stationary=True,trial_seconds=a.trial_seconds),inputs=ages,
                    cmd_publishers=[e.node_name for e in node.get_publishers_info_by_topic('/cmd_vel')],
                    cmd_subscribers=[e.node_name for e in node.get_subscriptions_info_by_topic('/cmd_vel')],
                    gateway_publishers=[e.node_name for e in node.get_publishers_info_by_topic('/follower/motor_candidate/guard_command')])
    def await_response(client):
        until=time.monotonic()+.5
        while not client.service_is_ready() and time.monotonic()<until:spin()
        if not client.service_is_ready():raise Abort('SERVICE_UNAVAILABLE')
        log('service_request',service='arm' if client is arm else 'disarm')
        future=client.call_async(Trigger.Request());end=time.monotonic()+1.
        while not future.done() and time.monotonic()<end:spin()
        if not future.done():raise Abort('SERVICE_RESPONSE_TIMEOUT')
        log('service_response',service='arm' if client is arm else 'disarm')
        return future.result()

    try:
        print('시작 전 연결 확인 중. 아직 명령을 보내지 않습니다.',flush=True)
        print('TRIAL_LOG='+str(path),flush=True)
        until=time.monotonic()+12
        while time.monotonic()<until:
            spin()
            if a.temporary_detour and diagnostics:
                problem=detour_preflight_problem(diagnostics[-1][0])
                if problem:
                    log('detour_preflight_failed',reason=problem,diagnostic=diagnostics[-1][0])
                    raise Abort(problem)
            if (motor_reason(motor,time.monotonic(),stationary=True,trial_seconds=a.trial_seconds) is None
                    and graph_ok() and inputs_ok() and stationary()
                    and arm.service_is_ready() and disarm.service_is_ready()):break
        else:
            state=snapshot();log('preflight_failed',diagnostic=state)
            print(json.dumps(state,ensure_ascii=False,indent=2),flush=True)
            raise Abort('PREFLIGHT_NOT_READY')
        print('START + Enter: 선두 출발 요청 → 유효 경로 확인 후 후발 출발. 종료 시 양쪽 정지 요청. Ctrl+C 중단.',flush=True)
        if not wait_for_start(spin):raise Abort('START_CANCELLED')
        if a.temporary_detour and diagnostics:
            problem=detour_preflight_problem(diagnostics[-1][0])
            if problem:raise Abort(problem)
        if not (graph_ok() and inputs_ok() and stationary()
                and motor_reason(motor,time.monotonic(),stationary=True,trial_seconds=a.trial_seconds) is None):
            state=snapshot();log('start_not_ready',diagnostic=state)
            print(json.dumps(state,ensure_ascii=False,indent=2),flush=True)
            raise Abort('START_INPUT_NOT_READY')
        window.start(time.monotonic());leader_started=True
        log('leader_start',command_v=a.leader_speed,command_w=leader_turn_rate(a.leader_turn))
        leader_command(a.leader_speed,leader_turn_rate(a.leader_turn))
        print(f'선두 출발 요청({a.leader_speed}m/s, {a.leader_turn}). 후발 유효 전진 조건 대기 중.',flush=True)
        while True:
            spin();now=time.monotonic();ros=node.get_clock().now().nanoseconds
            expired=window.expired(now)
            if expired:raise Abort(expired)
            if not graph_ok() or not inputs_ok():raise Abort('INPUT_OR_CONNECTION_LOST')
            problem=motor_reason(motor,now,stationary=True,trial_seconds=a.trial_seconds)
            if problem:raise Abort(problem)
            if current_scene(ros,now)=='READY':break
        arm_requested=True;response=await_response(arm)
        if not response.success:raise Abort('ARM_REJECTED')
        epoch=json.loads(response.message)['epoch'];window.follow(time.monotonic())
        log('follower_arm',epoch=epoch)
        print(f'후발 추종 시작. 최대{window.follow_limit}초 뒤 양쪽 정지 요청.',flush=True)
        while True:
            spin();now=time.monotonic();ros=node.get_clock().now().nanoseconds
            expired=window.expired(now)
            if expired:raise Abort(expired)
            if not graph_ok() or not inputs_ok():raise Abort('INPUT_OR_CONNECTION_LOST')
            problem=motor_reason(motor,now,stationary=False,trial_seconds=a.trial_seconds)
            if problem:raise Abort(problem)
            s=motor[0]
            if s.get('armed') and s.get('epoch')!=epoch:raise Abort('SESSION_CHANGED')
            if not s.get('armed') and now-window.armed>.2:raise Abort('UNEXPECTED_DISARM')
            scene=current_scene(ros,now)
            if scene!='READY':
                g=None if guard is None else guard[0]
                stamp=None if g is None else g.header.stamp.sec*10**9+g.header.stamp.nanosec
                failure=(ros,now,None if g is None else dict(source_ns=stamp,frame=g.header.frame_id,
                    zero=all(v==0 for v in (g.twist.linear.x,g.twist.linear.y,g.twist.linear.z,
                                           g.twist.angular.x,g.twist.angular.y,g.twist.angular.z))))
                log('scene_failure',reason=scene,guard_source_ns=stamp,
                    guard_source_age_s=None if not stamp else (ros-stamp)/1e9,
                    guard_rx_age_s=None if guard is None else now-guard[1],
                    guard_v=None if g is None else g.twist.linear.x)
                raise Abort(scene)
    except Abort as exc:reason=str(exc)
    except (KeyboardInterrupt,EOFError):reason='OPERATOR_STOP'
    except Exception as exc:
        reason='OPERATOR_TOOL_ERROR';log('exception',error=str(exc))
    finally:
        stopping=True;stop_future=None
        if leader_started:
            leader_command(0.);log('leader_stop_requested')
        if arm_requested:
            # 선두 영속도 발행 직후 같은 제어 경로에서 후발 disarm을 요청한다.
            try:
                if disarm.service_is_ready():
                    stop_future=disarm.call_async(Trigger.Request());log('follower_stop_requested')
                else:log('disarm_unavailable')
            except Exception as exc:log('disarm_error',error=str(exc))
        if leader_started or arm_requested:
            until=time.monotonic()+3;settled=time.monotonic()+1
            mc=None if motor is None else motor[0].get('valid_feedback');lc=None
            mz=lz=0
            while time.monotonic()<until:
                spin();now=time.monotonic();ros=node.get_clock().now().nanoseconds
                if motor is not None:
                    s,rx=motor;count=s.get('valid_feedback')
                    if count!=mc:
                        mc=count
                        ok=(0<=now-rx<=.3 and s.get('armed') is False and s.get('feedback_fresh') is True
                            and s.get('io_failed') is False and s.get('measured_rpm')==[0,0])
                        mz=mz+1 if ok else 0
                sample=samples.get('leader')
                if sample is not None:
                    m=sample[0];stamp=m.header.stamp.sec*10**9+m.header.stamp.nanosec
                    if stamp!=lc:
                        lc=stamp
                        lz=lz+1 if fresh_sample(sample,ros,now,remote=True) and stationary() else 0
                if now>=settled and mz>=3 and lz>=3:
                    follower_zero=leader_zero=True;break
            follower_zero=follower_zero or mz>=3
            leader_zero=leader_zero or lz>=3
            if stop_future is not None and stop_future.done():
                try:log('disarm_response',success=stop_future.result().success)
                except Exception as exc:log('disarm_response_error',error=str(exc))
        evidence=None
        if a.expect_obstacle_stop and reason=='GUARD_STOP_OR_INVALID' and failure is not None:
            fr,fm,fg=failure
            evidence=next((e for d,rx in diagnostics
                           if (e:=obstacle_stop_evidence(d,rx,fr,fm,fg)) is not None),None)
        completed=(evidence is not None) if a.expect_obstacle_stop else reason in ('TRIAL_TIME_LIMIT','LEADER_TIME_LIMIT')
        success=(completed and arm_requested
                 and follower_zero and leader_zero and not faults)
        result=dict(reason=reason,leader_started=leader_started,arm_requested=arm_requested,
                    leader_zero_confirmed=leader_zero,follower_zero_confirmed=follower_zero,
                    post_stop_faults=sorted(faults),trial_success=success)
        if a.temporary_detour:result.update(controller_kind=KIND,plan_sha256=detour_hash,evaluation_pass=False)
        if a.expect_obstacle_stop:result.update(expected_obstacle_stop=True,obstacle_stop_confirmed=evidence is not None,
                                               obstacle_stop_evidence=evidence)
        print(json.dumps(result,ensure_ascii=False,indent=2),flush=True)
        if leader_started and not (leader_zero and follower_zero):
            print('정지 확인이 완료되지 않았습니다. 실물이 멈추지 않으면 즉시 전원을 차단하세요.',flush=True)
        if a.trace_timing:
            for row in timing:log('timing_sample',**row)
        log('end',**result);stream.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    return 0 if success else 1


if __name__=='__main__':raise SystemExit(main())
