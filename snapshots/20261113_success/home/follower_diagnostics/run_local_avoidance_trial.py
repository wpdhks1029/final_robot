#!/usr/bin/env python3
"""Explicit joint bounded local avoidance trial. Default starts nothing."""
import argparse,json,math,os,time
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from types import SimpleNamespace as S
from local_avoidance_drive import PROFILE,COMMAND_TOPIC
from local_avoidance_limits import SOURCE_TIMEOUT_S
from local_stop_turn_go import LocalCore,build_local_plan
from temporary_detour_candidate import Pose
from check_temporary_detour import reference_from_plan
from leader_short_forward_trial import wait_for_start

ROOT=Path(__file__).resolve().parent


def motor_problem(sample,mono,seconds,stationary=False,continuous20=False):
    if sample is None or not 0<=mono-sample[1]<=.3:return 'MOTOR_STATUS_MISSING_OR_STALE'
    s=sample[0]
    if bool(s.get('continuous20',False))!=continuous20:return 'REQUIRE_MATCHING_CONTINUOUS20_DRIVER'
    if continuous20 and s.get('continuous_stop_clearance_m')!=.1:return 'REQUIRE_CONTINUOUS10_STOP_DRIVER_RESTART'
    if s.get('distance_test_only'):return 'DISTANCE_TEST_DRIVER_NOT_FOR_AVOIDANCE'
    if (s.get('mode')!='HARDWARE_CANDIDATE' or s.get('input_kind')!='local_avoidance'
            or s.get('motion_profile')!=PROFILE or s.get('input_source_timeout_s')!=SOURCE_TIMEOUT_S or s.get('trial_max_rpm')!=10
            or s.get('trial_max_seconds')!=seconds or s.get('enabled') is not True):return 'REQUIRE_LOCAL_AVOIDANCE_DRIVER_MATCH'
    if s.get('zero_stop_prearm_supported') is not True:return 'REQUIRE_ZERO_STOP_HANDSHAKE_DRIVER_RESTART'
    if s.get('zero_stop_response_retry_supported') is not True or s.get('zero_response_retry_version')!=2:return 'REQUIRE_ZERO_RESPONSE_RETRY_DRIVER_RESTART'
    if s.get('zero_reply_hold_supported') is not True:return 'REQUIRE_ZERO_REPLY_HOLD_DRIVER_RESTART'
    if s.get('fault') or s.get('io_failed') is not False or s.get('feedback_fresh') is not True:return 'MOTOR_FAULT_OR_FEEDBACK_STALE'
    if stationary and (s.get('armed') is not False or s.get('measured_rpm')!=[0,0] or s.get('zero_feedback_samples',0)<5):return 'REQUIRE_DISARMED_ZERO'
    return None


def main():
    p=argparse.ArgumentParser(description=__doc__)
    modes=p.add_mutually_exclusive_group()
    modes.add_argument('--run',action='store_true');modes.add_argument('--plan-only',action='store_true')
    p.add_argument('--ack-detour-area-power-cut-ready','--ack-clear-both-power-cut-ready',dest='ack_area',action='store_true')
    p.add_argument('--seconds',type=int,choices=(30,360),default=30)
    p.add_argument('--apf-pivot',action='store_true',help='APF-guided checked pivot/10cm steps; explicit candidate')
    p.add_argument('--gentle-pivot',action='store_true',help='Require a complete checked diagonal bypass; prefer smaller initial turns')
    p.add_argument('--map-rejoin',action='store_true',help='Use live map pose and leader path targets after bypass')
    p.add_argument('--continue-follow-after-rejoin',action='store_true',help='After verified rejoin, stop then hand off to tracked Pure Pursuit for a bounded confirmation')
    p.add_argument('--frame-check-distance',type=float,choices=(.2,),default=None,help='Follower-only coordinate diagnostic: checked initial pivot and20cm translation, then stop')
    p.add_argument('--follow-only',action='store_true',help='Tracked Pure Pursuit check only; stop on obstacle trigger instead of planning avoidance')
    p.add_argument('--follow-check-seconds',type=int,choices=(10,20),default=10)
    p.add_argument('--near-return-hold',action='store_true',help='APF only: zero output until 5 fresh clear scans; maximum 2s')
    p.add_argument('--pose-source',choices=('raw','filtered'),default='raw',help='Pose and measured-time scan projection source; raw remains monitored')
    p.add_argument('--reference-plan',type=Path,default=ROOT/'logs/current_scene_left_detour_plan.json')
    p.add_argument('--continuous20',action='store_true')
    p.add_argument('--dynamic-box',action='store_true',help='Start following without a prepared box; detect a newly appearing box and plan once after braking')
    p.add_argument('--post-rejoin-follow-m',type=float,choices=(1.,2.),default=1.,help='Continuous mode: map-verified following distance after rejoin')
    a=p.parse_args()
    if a.dynamic_box and not a.continuous20:p.error('--dynamic-box requires --continuous20')
    if a.post_rejoin_follow_m!=1. and not a.continuous20:p.error('--post-rejoin-follow-m requires --continuous20')
    if a.continuous20 and a.near_return_hold:p.error('Continuous20 uses immediate safety stop, not legacy near-return pause')
    if a.continuous20 and (not a.map_rejoin or not a.continue_follow_after_rejoin or a.frame_check_distance or a.follow_only):p.error('Continuous20 requires map rejoin with handoff')
    if a.frame_check_distance and (not a.gentle_pivot or a.follow_only or a.continue_follow_after_rejoin):p.error('--frame-check-distance requires gentle filtered pivot without follow modes')
    if a.follow_only and not a.map_rejoin:p.error('--follow-only requires --map-rejoin')
    if a.continue_follow_after_rejoin and not a.map_rejoin:p.error('--continue-follow-after-rejoin requires --map-rejoin')
    if a.map_rejoin and not a.gentle_pivot:p.error('--map-rejoin requires --gentle-pivot')
    if a.near_return_hold and not a.apf_pivot:p.error('--near-return-hold requires --apf-pivot')
    if a.gentle_pivot and (not a.apf_pivot or a.pose_source!='filtered'):
        p.error('--gentle-pivot requires --apf-pivot --pose-source filtered')
    from local_near_return_pause import NearReturnPause
    near_pause=NearReturnPause() if a.near_return_hold else None
    if a.continuous20:
        from continuous_near_return_pause import ContinuousNearReturnPause
        near_pause=ContinuousNearReturnPause()
    planner=build_local_plan
    controller_kind=PROFILE
    if a.apf_pivot:
        from apf_pivot_candidate import build_apf_pivot_plan
        planner=build_apf_pivot_plan;controller_kind='APF_PIVOT_STEP_V1'
        if a.gentle_pivot:
            from gentle_pivot_candidate import build_gentle_plan
            planner=build_gentle_plan;controller_kind='GENTLE_PIVOT_CHECKED_ROUTE_V1'
            if a.map_rejoin:
                def planner(points,start):
                    plan=dict(build_gentle_plan(points,start),live_map_rejoin=True)
                    plan['config']['rejoin_source']='LIVE_MAP_AND_LEADER_PATH'
                    plan['geometry']['scope']='STATIC_REFERENCE_ROUTE; LIVE_MAP_REJOIN_CHECKED_PER_STEP'
                    return plan
                controller_kind='GENTLE_PIVOT_MAP_REJOIN_V1'
    if a.follow_only:
        def planner(points,start):raise ValueError('NO_OBSTACLE_CLUSTER')
        controller_kind='TRACKED_PURSUIT_BOUNDED_CHECK_V1'
    if not a.run and not a.plan_only:p.print_help();return 0
    if (a.run and not a.ack_area) or os.environ.get('ROS_DOMAIN_ID')!='10':p.error('Explicit trial acknowledgement and ROS_DOMAIN_ID=10 required')
    if a.continuous20:
        from continuous_local_core import ContinuousCore,build_plan
        core_type=ContinuousCore;planner=build_plan;controller_kind='CONTINUOUS20_MAP_REJOIN_V1'
    else:core_type=LocalCore
    command_topic='/follower/motor_candidate/continuous20_command' if a.continuous20 else COMMAND_TOPIC
    reference=json.loads(a.reference_plan.read_text())
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry,Path as RosPath
    from sensor_msgs.msg import LaserScan,Imu
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from tf2_msgs.msg import TFMessage
    from follower_remote_qos import latest_state_qos
    from follower_pursuit_preview import PreviewCore
    from watch_apf_case3 import project
    from follower_input_guard import valid_odom
    rclpy.init(args=[],signal_handler_options=SignalHandlerOptions.NO)
    node=rclpy.create_node('local_avoidance_trial_operator')
    core=core_type(pose_source='filtered') if a.pose_source=='filtered' else core_type()
    if a.map_rejoin:
        from tracked_pursuit_candidate import TrackedPreviewCore
        nav=TrackedPreviewCore(lookahead=.35 if a.continuous20 else .8)
    else:nav=PreviewCore()
    nav.timeout=SOURCE_TIMEOUT_S
    motor=None;mount=None;leader_started=False;arm_requested=False;stopping=False;epoch=None
    started=None;armed=None;last_leader=0.;last_status=0.;desired_leader=.04
    leader_zero=follower_zero=False;faults=set();reason='NOT_STARTED';motion_seen=False;last_report_key=None
    raw_scan_ring=deque(maxlen=20);scan_failure_frames=None
    alignment_wait_started=None
    map_alignment_wait_started=None
    stable_since=None;stable_count=0;stable_stamp=None
    handoff_started=None;follow_started=None;follow_progress_start=None;handoff_confirmed=False
    folder=ROOT/'logs';folder.mkdir(exist_ok=True)
    logfile=folder/f'local_avoidance_trial_{time.time_ns()}.jsonl';logstream=logfile.open('x')
    samples={};end_rows=[];worker=ThreadPoolExecutor(max_workers=1)
    commands=None if a.plan_only else node.create_publisher(String,command_topic,QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE))
    leadpub=None if a.plan_only else node.create_publisher(Twist,'/cmd_vel',1)
    statuspub=None if a.plan_only else node.create_publisher(String,'/follower/local_avoidance/status',latest_state_qos())
    arm=node.create_client(Trigger,'/follower/motor_candidate/arm');disarm=node.create_client(Trigger,'/follower/motor_candidate/disarm')
    class Abort(Exception):pass
    def now():return node.get_clock().now().nanoseconds
    def log(event,**data):
        logstream.write(json.dumps(dict(event=event,ros_ns=now(),**data),allow_nan=False)+'\n');logstream.flush()
    def leader_command(v):
        msg=Twist();msg.linear.x=0. if a.frame_check_distance else float(v);leadpub.publish(msg)
    def publish_zero_hold(source,scan_stamp):
        sent=now()
        if not 0<=(sent-source)/1e9<=SOURCE_TIMEOUT_S:raise Abort('ZERO_HOLD_SOURCE_STALE')
        commands.publish(String(data=json.dumps(dict(version=3,controller=PROFILE,epoch=epoch,
            frame='follower/base_link',stamp_ns=sent,source_stamp_ns=source,
            scan_stamp_ns=scan_stamp,motion='STOP',v=0.,w=0.,clearance_m=0.,swept_clearance_m=0.))))
    def wait_for_post_arm_sources(source,scan_stamp,arm_ns,mono,sources):
        log('post_arm_wait',arm_ros_ns=arm_ns,source_stamp_ns=source,
            scan_stamp_ns=scan_stamp,source_stamps=sources)
        if mono-armed>SOURCE_TIMEOUT_S:raise Abort('POST_ARM_MEASUREMENTS_NOT_READY')
        publish_zero_hold(source,scan_stamp)
    path_observation_fault=[None]
    def odom_cb(key,m):
        stamp=m.header.stamp.sec*10**9+m.header.stamp.nanosec;rx=time.monotonic()
        if key in ('raw','filtered'):
            if not valid_odom(m,'follower/odom','follower/base_link'):core.inputs.pop(key,None);return
            q=m.pose.pose.orientation;p=m.pose.pose.position;t=m.twist.twist
            yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            if abs(t.linear.x)>.12 or abs(t.angular.z)>.5:core.inputs.pop(key,None);return
            callback=core.raw if key=='raw' else core.filtered
            callback(stamp,rx,(p.x,p.y,yaw,t.linear.x,t.angular.z))
        else:
            nav.odom_callback(key,m,now(),rx)
            item=nav.inputs.get(key)
            if key=='follower' and item is not None and hasattr(nav,'path_tracker'):
                try:nav.path_tracker.observe_position(item[2])
                except ValueError as exc:path_observation_fault[0]=str(exc)
            if key=='leader':
                if item is None:core.inputs.pop('leader',None)
                else:core.inputs['leader']=(stamp,rx,item[2])
        samples[key]=(m,rx)
    def scan_cb(m):
        core.scan=None
        raw_scan_ring.append((m,now()))
        if mount is None:return
        try:
            points=project(m,mount);stamp=m.header.stamp.sec*10**9+m.header.stamp.nanosec
            core.offer_scan(stamp,time.monotonic(),points)
        except (ValueError,TypeError):return
    def imu_cb(m):
        try:
            from follower_yaw_diagnostics import rpy
            if m.header.frame_id!='follower/imu_link' or m.orientation_covariance[0]<0:raise ValueError('IMU_INVALID')
            roll,pitch,yaw=rpy(m.orientation)
            if max(abs(roll),abs(pitch))>math.radians(20):raise ValueError('IMU_MOUNT_TILT')
            stamp=m.header.stamp.sec*10**9+m.header.stamp.nanosec
            core.imu(stamp,time.monotonic(),(roll,pitch,yaw,0.,m.angular_velocity.z))
        except (ValueError,TypeError,IndexError):core.inputs.pop('imu',None)
    def tf_cb(m):
        nonlocal mount
        for tf in m.transforms:
            if (tf.header.frame_id,tf.child_frame_id)==('follower/base_link','follower/laser'):
                t=tf.transform.translation;q=tf.transform.rotation
                expected=(t.x-.26,t.y,t.z-.24,q.x,q.y,q.z,abs(q.w)-1.)
                mount=tf if all(math.isfinite(v) and abs(v)<.001 for v in expected) else None
    def motor_cb(m):
        nonlocal motor,motion_seen
        try:
            s=json.loads(m.data)
            if not isinstance(s,dict):raise ValueError('status not object')
            motor=(s,time.monotonic());log('motor',status=s)
            if s.get('armed') and any(abs(v)>0 for v in s.get('measured_rpm',[0,0])):motion_seen=True
            if stopping and s.get('fault'):faults.add(str(s['fault']))
        except (ValueError,TypeError):motor=None
    subs=[node.create_subscription(Odometry,'/follower/odom_raw',lambda m:odom_cb('raw',m),latest_state_qos()),
          node.create_subscription(Odometry,'/follower/odom_map',lambda m:odom_cb('follower',m),latest_state_qos()),
          node.create_subscription(Odometry,'/leader/odom',lambda m:odom_cb('leader',m),latest_state_qos()),
          node.create_subscription(RosPath,'/leader/path',lambda m:nav.path_callback(m,now(),time.monotonic()),latest_state_qos()),
          node.create_subscription(LaserScan,'/follower/scan',scan_cb,latest_state_qos()),
          node.create_subscription(String,'/follower/motor_candidate/status',motor_cb,latest_state_qos()),
          node.create_subscription(TFMessage,'/tf_static',tf_cb,QoSProfile(depth=20,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL))]
    if a.pose_source=='filtered':
        subs.append(node.create_subscription(Odometry,'/follower/odometry/filtered',lambda m:odom_cb('filtered',m),latest_state_qos()))
        subs.append(node.create_subscription(Imu,'/follower/imu/data',imu_cb,latest_state_qos()))
    def graph_ok():
        for topic,owner in [('/cmd_vel',node.get_name()),(command_topic,node.get_name())]:
            entries=node.get_publishers_info_by_topic(topic)
            if a.plan_only:
                if entries:return False
            elif len(entries)!=1 or entries[0].node_name!=owner:return False
        for topic in ('/cmd_vel',command_topic):
            entries=node.get_subscriptions_info_by_topic(topic)
            expected='md200t_controller' if topic=='/cmd_vel' else 'follower_drive_candidate'
            if len(entries)!=1 or entries[0].node_name!=expected:return False
        for topic in ('/follower/odom_raw','/follower/scan','/leader/odom','/follower/motor_candidate/status'):
            if len(node.get_publishers_info_by_topic(topic))!=1:return False
        if a.pose_source=='filtered' and len(node.get_publishers_info_by_topic('/follower/odometry/filtered'))!=1:return False
        if a.pose_source=='filtered' and len(node.get_publishers_info_by_topic('/follower/imu/data'))!=1:return False
        return True
    def spin():
        nonlocal last_leader
        if not rclpy.ok():raise Abort('ROS_SHUTDOWN')
        rclpy.spin_once(node,timeout_sec=.01)
        if a.pose_source=='filtered':
            # Drain pending state callbacks before computing: one spin handles
            # only one callback, while IMU/EKF/scan arrive independently.
            drain_started=time.monotonic()
            for _ in range(6):
                if time.monotonic()-drain_started>.003:break
                rclpy.spin_once(node,timeout_sec=0.)
        mono=time.monotonic()
        if leader_started and mono-last_leader>=.05:
            leader_command(0. if stopping or mono-started>a.seconds+4. else desired_leader);last_leader=mono
    def call(client):
        if not client.service_is_ready():raise Abort('SERVICE_UNAVAILABLE')
        f=client.call_async(Trigger.Request());end=time.monotonic()+1.
        while not f.done() and time.monotonic()<end:spin()
        if not f.done():raise Abort('SERVICE_TIMEOUT')
        response=f.result()
        if not response.success:raise Abort('SERVICE_REJECTED')
        return response
    from live_rejoin_verification import LiveRejoinVerification
    rejoin_verifier=LiveRejoinVerification()
    post_follow_verifier=LiveRejoinVerification(required_distance_m=a.post_rejoin_follow_m)
    from map_rejoin_guidance import MapPoseContinuity
    map_continuity=MapPoseContinuity()
    reference_diagnostic=None
    def fresh_reference():
        nonlocal reference_diagnostic
        mono=time.monotonic();ns=now()
        for key in ('leader','follower'):
            r=nav.inputs.get(key)
            if r is None:raise ValueError(key.upper()+'_MISSING')
            stamp=r[0].sec*10**9+r[0].nanosec-(50_000_000 if key in ('path','leader') else 0)
            if not (0<=(ns-stamp)/1e9<=SOURCE_TIMEOUT_S and 0<=mono-r[1]<=SOURCE_TIMEOUT_S):raise ValueError(key.upper()+'_STALE')
        if 'raw' not in core.inputs:raise ValueError('RAW_MISSING')
        if a.pose_source not in core.inputs:raise ValueError(a.pose_source.upper()+'_MISSING')
        if a.pose_source=='filtered' and 'imu' not in core.inputs:raise ValueError('IMU_MISSING')
        m=nav.inputs['follower'][2];l=nav.inputs['leader'][2]
        if any(abs(v)>.01 for v in (l[3],l[4])):raise ValueError('REQUIRE_STATIONARY_LEADER')
        path=nav.inputs.get('path')
        # A stopped leader can publish only one point. Use the stored map line
        # with fresh measured robot anchors solely for the initial frame.
        # Normal PP still requires its own fresh live path after leader start.
        points=path[2] if path is not None else (m[:2],l[:2])
        source_kind='live_path' if path is not None else 'robot_anchors'
        if path is not None:
            stamp=path[0].sec*10**9+path[0].nanosec-50_000_000
            if not (0<=(ns-stamp)/1e9<=SOURCE_TIMEOUT_S and 0<=mono-path[1]<=SOURCE_TIMEOUT_S):raise ValueError('PATH_STALE')
        try:
            origin,heading,cross,residual=reference_from_plan(reference,points,Pose(*m[:3]),l,source_kind=source_kind)
        except ValueError:
            try:
                from local_reference_diagnostics import alignment_diagnostics
                reference_diagnostic=dict(alignment_diagnostics(reference,points,m,l,source_kind=source_kind),ros_ns=ns)
            except (ValueError,TypeError,KeyError,IndexError) as exc:
                reference_diagnostic=dict(diagnostic_error=str(exc),ros_ns=ns)
            raise
        core.freeze(core.inputs[a.pose_source][2],(0.,cross,math.atan2(math.sin(m[2]-heading),math.cos(m[2]-heading))))
        return origin,heading
    origin=heading=None;planning=None;planning_started=None;box_count=0;last_box_scan=None;phase='PREFLIGHT'
    try:
        print('로컬 회피 연결 확인. 아직 주행/arm 요청 없음.',flush=True);print('TRIAL_LOG='+str(logfile),flush=True)
        end=time.monotonic()+12.;last_problem='WAIT_INPUTS'
        while time.monotonic()<end:
            spin()
            problem=motor_problem(motor,time.monotonic(),a.seconds,True,continuous20=a.continuous20)
            if problem:last_problem=problem;continue
            if not graph_ok() or mount is None:last_problem='GRAPH_OR_STATIC_MOUNT_NOT_READY';continue
            try:
                origin,heading=fresh_reference();scene=core.scene(now(),time.monotonic())
                if not arm.service_is_ready() or not disarm.service_is_ready():raise ValueError('SERVICE_NOT_READY')
                break
            except ValueError as exc:last_problem=str(exc)
        else:
            if reference_diagnostic is not None and last_problem.startswith('SAVED_REFERENCE'):
                print('REFERENCE_ALIGNMENT='+json.dumps(reference_diagnostic,ensure_ascii=False),flush=True)
                log('reference_alignment',diagnostic=reference_diagnostic)
            raise Abort('PREFLIGHT_'+last_problem)
        start_pose=scene['pose']
        log('planning_input',points=scene['points'].tolist(),start=[start_pose.x,start_pose.y,start_pose.yaw],
            source_stamp_ns=scene['source'],scan_stamp_ns=scene['scan_stamp'],reference_origin=origin,reference_heading=heading,
            pose_source=a.pose_source,pose_anchor=list(core.inputs[a.pose_source][2]),
            imu_anchor=core.inputs.get('imu'),raw_anchor=core.inputs['raw'])
        if a.dynamic_box:
            plan=None;phase='FOLLOW'
            log('dynamic_box_follow_ready',obstacle_front_trigger_m=.7,control_connected=False)
        else:
            future=worker.submit(planner,scene['points'].tolist(),(start_pose.x,start_pose.y,start_pose.yaw))
            end=time.monotonic()+(65. if a.continuous20 else 15.)
            while not future.done() and time.monotonic()<end:
                spin()
                problem=motor_problem(motor,time.monotonic(),a.seconds,True,continuous20=a.continuous20)
                if problem:raise Abort(problem)
            if not future.done():raise Abort('PLANNING_TIMEOUT')
            try:
                plan=future.result();core.install(plan);phase='AVOID'
                log('checked_local_plan',plan=plan,reference_origin=origin,reference_heading=heading)
            except ValueError as exc:
                if str(exc)!='NO_OBSTACLE_CLUSTER':raise
                plan=None;phase='FOLLOW'
        if a.plan_only:
            if a.follow_only:
                selection=nav.path_tracker.select(nav.inputs['path'][2],nav.inputs['follower'][2],nav.lookahead,tip_heading=nav.inputs['leader'][2][2])
                pose=nav.inputs['follower'][2];target=selection['target_map'];dx,dy=target[0]-pose[0],target[1]-pose[1]
                x=math.cos(pose[2])*dx+math.sin(pose[2])*dy;y=-math.sin(pose[2])*dx+math.cos(pose[2])*dy
                if x<=0 or abs(math.atan2(y,x))>nav.heading_limit:raise Abort('FOLLOW_CHECK_TARGET_NOT_FORWARD')
                probe=core.command(now(),time.monotonic(),motor[0],follow_command=(.04,max(-.14,min(.14,2*y/(x*x+y*y)*.04))))
                log('follow_check_preview',selection=selection,control=probe,control_connected=False)
                print(json.dumps(dict(reason=probe['reason'],clearance_m=probe['clearance_m'],swept_clearance_m=probe['swept_clearance_m'],selection=selection,control_connected=False),ensure_ascii=False),flush=True)
                if probe['reason'].startswith('STOP_'):raise Abort(probe['reason'])
            if plan is not None:
                saved=folder/f'local_plan_preview_{time.time_ns()}.json'
                saved.write_text(json.dumps(dict(plan,start=[start_pose.x,start_pose.y,start_pose.yaw],control_connected=False),allow_nan=False)+'\n')
                print('PLAN_JSON='+str(saved),flush=True)
                print(json.dumps(dict(reason='PLAN_READY',config=plan['config'],geometry=plan['geometry'],attempts=len(plan['attempts']),control_connected=False),ensure_ascii=False),flush=True)
            raise Abort('PLAN_ONLY_READY' if plan is not None or a.follow_only or a.dynamic_box else 'PLAN_ONLY_NO_OBSTACLE')
        print('현재 모드 '+phase+' / '+controller_kind+' / 자세 '+a.pose_source+'. START + Enter: 선두·후발 제한 시험 시작.',flush=True)
        if not wait_for_start(spin):raise Abort('START_CANCELLED')
        problem=motor_problem(motor,time.monotonic(),a.seconds,True,continuous20=a.continuous20)
        if problem or not graph_ok():raise Abort(problem or 'GRAPH_CHANGED')
        scene=core.scene(now(),time.monotonic());current=scene['pose']
        if math.hypot(current.x-start_pose.x,current.y-start_pose.y)>.03 or abs(current.yaw-start_pose.yaw)>.05:raise Abort('POSE_CHANGED_AFTER_PLANNING')
        from watch_temporary_detour_shadow import PreparedPathReview
        if plan is not None:
            if a.continuous20:
                from continuous_local_core import route_still_clear
                clear=route_still_clear(plan,scene['points'].tolist())
            else:clear=PreparedPathReview(plan['path']).review(scene['points'].tolist())['reason']=='LIVE_PATH_GEOMETRY_CLEAR'
            if not clear:
                log('start_route_rejected',points=scene['points'].tolist(),scan_stamp_ns=scene['scan_stamp'],pose=[current.x,current.y,current.yaw],plan_config=plan['config'])
                raise Abort('START_ROUTE_CHANGED')
        if a.frame_check_distance:
            started=time.monotonic();leader_command(0.);log('follower_only_frame_check',distance_m=a.frame_check_distance)
        else:
            started=time.monotonic();leader_started=True;leader_command(.04);log('leader_start')
            # Only the measured leader start is awaited; a stopped leader is later
            # permitted exclusively inside this explicit cooperative avoidance session.
            while time.monotonic()-started<4.:
                spin()
                if not graph_ok():raise Abort('GRAPH_CHANGED')
                problem=motor_problem(motor,time.monotonic(),a.seconds,True,continuous20=a.continuous20)
                if problem:raise Abort(problem)
                core.scene(now(),time.monotonic())
                if core.inputs['leader'][2][3]>.03 and (phase=='AVOID' or 'path' in nav.inputs):break
            else:raise Abort('LEADER_START_TIMEOUT')
        arm_requested=True;response=call(arm);epoch=json.loads(response.message)['epoch'];armed=time.monotonic();log('follower_arm',epoch=epoch)
        initial_leader=core.inputs['leader'][2]
        leader_start_x=math.cos(heading)*(initial_leader[0]-origin[0])+math.sin(heading)*(initial_leader[1]-origin[1])
        gap=max(1.54,leader_start_x-start_pose.x)
        if a.frame_check_distance:print('후발 좌표 진단: 선두 정지 유지. 초기 회전 후20cm 이동 또는 최대60초에 종료.',flush=True)
        print(f'로컬 회피 시작. 후발 최대 {a.seconds-.5}초. 회전/직진 사이 정지 확인.',flush=True)
        while True:
            spin();mono=time.monotonic();ns=now()
            if path_observation_fault[0]:raise Abort(path_observation_fault[0])
            if a.frame_check_distance and mono-armed>=60.:raise Abort('FRAME_CHECK_TIME_LIMIT')
            if a.follow_only and mono-armed>=a.follow_check_seconds:raise Abort('FOLLOW_CHECK_TIME_LIMIT')
            if mono-armed>=a.seconds-.5:raise Abort('TRIAL_TIME_LIMIT')
            if not graph_ok():raise Abort('GRAPH_CHANGED')
            problem=motor_problem(motor,mono,a.seconds,continuous20=a.continuous20)
            if problem:raise Abort(problem)
            state=motor[0]
            if state.get('armed') and state.get('epoch')!=epoch:raise Abort('SESSION_CHANGED')
            if not state.get('armed') and mono-armed>.2:raise Abort('UNEXPECTED_DISARM')
            # A disarmed status may retain the previous trial's arm timestamp.
            # Never validate new commands against that cached session.
            if state.get('armed') is not True or state.get('epoch')!=epoch:
                continue
            try:scene=core.scene(ns,mono)
            except ValueError as exc:
                diagnostic=getattr(core,'last_alignment_failure',None)
                if diagnostic and 'SCAN_BRACKET_MISSING' in str(exc):
                    log('scan_alignment_failure',diagnostic=diagnostic)
                    if scan_failure_frames is None:scan_failure_frames=list(raw_scan_ring)
                if (a.pose_source!='filtered' or str(exc)!='FILTERED_ODOM_SCAN_BRACKET_MISSING'
                        or not diagnostic or diagnostic.get('kind') not in ({'WAITING_FOR_NEWER_POSE','POSE_HISTORY_GAP'} if a.continuous20 else {'WAITING_FOR_NEWER_POSE'})):raise
                if alignment_wait_started is None:
                    alignment_wait_started=mono;scan_failure_frames=list(raw_scan_ring);last_report_key=None
                    desired_leader=0.;leader_command(0.)
                    print('['+phase+'] STOP: WAIT_FILTERED_SCAN_ALIGNMENT',flush=True)
                log('scan_alignment_wait',diagnostic=diagnostic,elapsed_s=mono-alignment_wait_started)
                if mono-alignment_wait_started>=(.4 if a.continuous20 else .2):raise Abort('FILTERED_SCAN_ALIGNMENT_TIMEOUT')
                stamps=[core.inputs[k][0] for k in ('raw','filtered','imu')]
                stamps.extend([core.inputs['leader'][0]-50_000_000,core.scan[0]])
                publish_zero_hold(min(stamps),core.scan[0])
                continue
            if alignment_wait_started is not None:
                if a.continuous20:
                    core.last_curve=(mono,0.,0.)
                    if mono-alignment_wait_started>=.4:raise Abort('FILTERED_SCAN_ALIGNMENT_TIMEOUT')
                    if state.get('measured_rpm')!=[0,0] or state.get('zero_feedback_samples',0)<3:
                        publish_zero_hold(scene['source'],scene['scan_stamp'])
                        continue
                log('scan_alignment_recovered',waited_s=mono-alignment_wait_started)
                alignment_wait_started=None
            if phase=='REJOIN_BRAKE':
                desired_leader=0.;leader_command(0.)
                publish_zero_hold(scene['source'],scene['scan_stamp'])
                if state.get('measured_rpm')==[0,0] and state.get('zero_feedback_samples',0)>=3:
                    core.handoff_to_follow(state);phase='FOLLOW';follow_started=mono
                    follow_progress_start=nav.path_tracker.progress;desired_leader=.04
                    log('handoff_to_tracked_pursuit',path_progress_m=follow_progress_start)
                elif mono-handoff_started>2.:raise Abort('REJOIN_HANDOFF_ZERO_TIMEOUT')
                continue
            arm_ns=state.get('arm_ros_ns')
            if type(arm_ns) is not int or min(scene['source'],scene['scan_stamp'])<arm_ns:
                wait_for_post_arm_sources(scene['source'],scene['scan_stamp'],arm_ns,mono,scene.get('source_stamps',{}))
                continue
            if a.continuous20 and phase=='FOLLOW' and 'follower' in nav.inputs:
                map_continuity.check(nav.inputs['follower'])
            if phase=='FOLLOW':
                if scene['scan_stamp']!=last_box_scan:
                    last_box_scan=scene['scan_stamp'];p0=scene['pose'];cloud=scene['points']
                    from dynamic_box_detection import front_obstacle_count
                    count=front_obstacle_count(cloud,(p0.x,p0.y,p0.yaw),.7 if a.dynamic_box else .9)
                    box_count=box_count+1 if count>=5 else 0
                    if a.follow_only and box_count>=3:raise Abort('FOLLOW_CHECK_OBSTACLE_DETECTED')
                    if box_count>=3:phase='PLAN_BRAKE';planning_started=mono;desired_leader=0.;log('obstacle_trigger',points=count)
            if phase in ('PLAN_BRAKE','PLANNING'):
                desired_leader=0.
                if mono-planning_started>(65. if a.continuous20 else 15.):raise Abort('LOCAL_REPLANNING_TIMEOUT')
                if phase=='PLAN_BRAKE' and state.get('measured_rpm')==[0,0] and state.get('zero_feedback_samples',0)>=3:
                    p0=scene['pose']
                    log('planning_input',points=scene['points'].tolist(),start=[p0.x,p0.y,p0.yaw],source_stamp_ns=scene['source'],scan_stamp_ns=scene['scan_stamp'])
                    planning=worker.submit(planner,scene['points'].tolist(),(p0.x,p0.y,p0.yaw));phase='PLANNING'
                if phase=='PLANNING' and planning.done():
                    plan=planning.result()
                    if a.continuous20:
                        from continuous_local_core import route_still_clear
                        clear=route_still_clear(plan,scene['points'].tolist())
                    else:clear=PreparedPathReview(plan['path']).review(scene['points'].tolist())['reason']=='LIVE_PATH_GEOMETRY_CLEAR'
                    if not clear:raise Abort('REPLANNED_ROUTE_CHANGED')
                    core.install(plan);phase='AVOID';log('checked_local_plan',plan=plan,reference_origin=origin,reference_heading=heading)
            follow=None
            if core.fsm is None:
                follow=(0.,0.)
                if phase=='FOLLOW':
                    from follower_stamped_preview import oldest_pursuit_source
                    v,w,follow_reason=nav.command(ns,mono)
                    if a.map_rejoin and follow_reason.startswith('STOP_') and follow_reason not in ('STOP_LEADER_STOPPED','STOP_FOLLOW_DISTANCE'):
                        log('tracked_pursuit_stop',reason=follow_reason,selection=nav.last_selection)
                        raise Abort(follow_reason)
                    follow_source=oldest_pursuit_source(nav.inputs,remote_clock_margin_ns=50_000_000)
                    if follow_source<=0 or not 0<=(ns-follow_source)/1e9<=SOURCE_TIMEOUT_S:raise Abort('FOLLOW_REFERENCE_STALE_OR_MISSING')
                    follow=(v,w)
            compute_started=time.monotonic()
            map_guidance=None
            if a.map_rejoin and core.fsm is not None and core.fsm.controller.stage>=1:
                from map_rejoin_guidance import guidance,MapPoseBracketMissing
                try:map_guidance=guidance(nav.inputs,core,ns,mono,tracker=nav.path_tracker)
                except MapPoseBracketMissing as exc:
                    log('map_pose_alignment_failure',diagnostic=exc.diagnostic)
                    if scan_failure_frames is None:scan_failure_frames=list(raw_scan_ring)
                    if not exc.diagnostic['waiting_for_newer_pose']:raise
                    if map_alignment_wait_started is None:
                        map_alignment_wait_started=mono;leader_command(0.);desired_leader=0.
                    if mono-map_alignment_wait_started>=.2:raise Abort('MAP_FILTERED_POSE_ALIGNMENT_TIMEOUT')
                    publish_zero_hold(scene['source'],scene['scan_stamp'])
                    continue
                if map_alignment_wait_started is not None:
                    log('map_pose_alignment_recovered',waited_s=mono-map_alignment_wait_started)
                    map_alignment_wait_started=None
                map_continuity.check(nav.inputs['follower'])
                core.fsm.controller.map_guidance=map_guidance
            result=core.command(ns,mono,state,follow_command=follow)
            if map_guidance is not None:
                result['map_rejoin_guidance']=map_guidance
                result['source_stamp_ns']=min(result['source_stamp_ns'],map_guidance['source_stamp_ns'])
            result['compute_s']=time.monotonic()-compute_started
            if phase=='FOLLOW':result['source_stamp_ns']=min(result['source_stamp_ns'],follow_source)
            result['trial_phase']=phase
            if a.frame_check_distance:
                result['frame_check_travel_m']=math.hypot(result['pose'][0]-start_pose.x,result['pose'][1]-start_pose.y)
                if result['frame_check_travel_m']>=a.frame_check_distance:
                    result.update(v=0.,w=0.,motion='STOP',reason='FRAME_CHECK_COMPLETE')
                    scan_failure_frames=list(raw_scan_ring)
                    log('stopped_control',control=result,points=scene['points'].tolist())
                    raise Abort('FRAME_CHECK_COMPLETE')
            if near_pause is not None:
                result=near_pause.apply(result,mono,state)
                pause=result.get('near_return_pause',{})
                if a.continuous20 and pause:
                    core.last_curve=(mono,0.,0.)
                    if pause.get('event')=='started':leader_command(0.);desired_leader=0.
                if pause.get('event') in ('started','resumed'):
                    log('near_return_pause_'+pause['event'],control=result,points=scene['points'].tolist())
                    if pause['event']=='started':scan_failure_frames=list(raw_scan_ring)
                    elif scan_failure_frames is not None:scan_failure_frames+=list(raw_scan_ring)
            report_key=(phase,result.get('goal_index'),result['motion'],result['reason'])
            if report_key!=last_report_key:
                apf_info=result.get('apf',{})
                extra=f" / APF 목표각 {math.degrees(apf_info['selected_heading_rad']):.0f}°" if 'selected_heading_rad' in apf_info else ''
                if 'stage_name' in apf_info:
                    extra+=' / 단계 '+{'MOVE_OUT':'측방확보','PASS_BOX':'상자옆통과','REJOIN':'경로복귀','TRACK_REFERENCE':'직선추종'}[apf_info['stage_name']]
                print(f"[{phase}] {result['motion']}: {result['reason']} / 여유 {result['clearance_m']:.2f}m"+extra,flush=True)
                last_report_key=report_key
            if result['reason'].startswith('STOP_') or result['reason'] in ('LOCAL_ROUTE_END','PLAN_REQUIRED'):
                scan_failure_frames=list(raw_scan_ring)
                log('stopped_control',control=result,points=scene['points'].tolist())
                raise Abort(result['reason'])
            if a.gentle_pivot and phase=='AVOID':
                eligible=result.get('apf',{}).get('stage')==3 and result['pose'][0]>=core.plan['return_x']
                if a.map_rejoin:
                    p=result['pose'];rear=p[0]-.27*abs(math.cos(p[2]))-.3*abs(math.sin(p[2]))
                    eligible=result.get('apf',{}).get('stage')==3 and rear>core.plan['box_far']+.30
                verification=rejoin_verifier.check(nav.inputs,ns,mono,result['pose'],origin,heading,eligible,require_frozen_alignment=not a.map_rejoin,path_selection=map_guidance.get('path_selection') if map_guidance else None)
                result['live_rejoin_verification']=verification
                if mono-last_status>=.1 or result['geometry_complete']:
                    log('map_path_verification',**verification)
                if a.continuous20:
                    stamp=nav.inputs['follower'][0].sec*10**9+nav.inputs['follower'][0].nanosec
                    if verification['reason']=='LIVE_MAP_PATH_ALIGNED':
                        if stable_stamp is not None and stamp-stable_stamp>300_000_000:stable_since=None;stable_count=0
                        if stable_since is None:stable_since=stamp
                        if stamp!=stable_stamp:stable_count+=1
                        stable_stamp=stamp
                    else:stable_since=None;stable_count=0;stable_stamp=None
                    result['geometry_complete']=stable_since is not None and stamp-stable_since>=1_000_000_000 and stable_count>=5
                elif a.map_rejoin:result['geometry_complete']=verification['verified']
                if not a.continuous20 and result['geometry_complete'] and not verification['verified']:
                    log('completion_rejected',control=result)
                    raise Abort('STOP_REJOIN_NOT_VERIFIED_IN_MAP')
            if result['geometry_complete']:
                if a.continue_follow_after_rejoin:
                    if a.continuous20:
                        core.handoff_to_follow(state,follow_distance_m=a.post_rejoin_follow_m);phase='FOLLOW';follow_started=mono
                        follow_progress_start=nav.path_tracker.progress;desired_leader=.04
                        log('continuous_handoff_to_tracked_pursuit',verification=verification)
                    else:phase='REJOIN_BRAKE';handoff_started=mono
                    if not a.continuous20:result.update(v=0.,w=0.,motion='STOP',geometry_complete=False,reason='BRAKE_BEFORE_TRACKED_PURSUIT')
                    else:result['geometry_complete']=False
                    log('map_rejoin_verified_before_handoff',verification=verification)
                else:raise Abort('GEOMETRIC_PASS_REJOIN_PLUS_1M')
            if follow_started is not None:
                progress=(nav.path_tracker.progress or 0.)-(follow_progress_start or 0.)
                result['post_rejoin_follow_progress_m']=progress
                result['tracked_pursuit_selection']=nav.last_selection
                if a.continuous20:
                    check=post_follow_verifier.check(nav.inputs,ns,mono,result['pose'],origin,heading,True,require_frozen_alignment=False,path_selection=nav.last_selection)
                    result['post_rejoin_map_verification']=check
                    progress=check['forward_on_live_path_m']
                result['post_rejoin_verified_progress_m']=progress
                result['post_rejoin_follow_target_m']=a.post_rejoin_follow_m
                if (check['verified'] if a.continuous20 else mono-follow_started>=10. and progress>=.10):
                    handoff_confirmed=True;raise Abort('REJOIN_AND_FOLLOW_CONFIRMED')
                if mono-follow_started>=(90.+40.*(a.post_rejoin_follow_m-1.) if a.continuous20 else 20.):raise Abort('POST_REJOIN_FOLLOW_NOT_CONFIRMED')
            # Old measurement stamps are retained; never stamp old data as now.
            source=result['source_stamp_ns'];scan_stamp=result['scan_stamp_ns'];arm_ns=state.get('arm_ros_ns')
            if type(arm_ns) is not int or min(source,scan_stamp)<arm_ns:
                wait_for_post_arm_sources(source,scan_stamp,arm_ns,mono,result.get('input_source_stamps',{}))
                continue
            sent=now()
            result['source_age_after_compute_s']=(sent-source)/1e9
            if not 0<=(sent-source)/1e9<=SOURCE_TIMEOUT_S:
                log('input_expired',control=result)
                raise Abort('INPUT_AGED_DURING_COMPUTE')
            obj=dict(version=3,controller=PROFILE,epoch=epoch,frame='follower/base_link',stamp_ns=sent,
                     source_stamp_ns=source,scan_stamp_ns=scan_stamp,motion=result['motion'],v=result['v'],w=result['w'],
                     clearance_m=result['clearance_m'],swept_clearance_m=result['swept_clearance_m'])
            commands.publish(String(data=json.dumps(obj,allow_nan=False)))
            if mono-last_status>=.1:
                statuspub.publish(String(data=json.dumps(result,allow_nan=False)));log('local_control',**result);last_status=mono
            leader=core.inputs['leader'][2]
            lx=math.cos(heading)*(leader[0]-origin[0])+math.sin(heading)*(leader[1]-origin[1])
            raw=core.inputs[a.pose_source][2];f=result['pose']
            if phase=='FOLLOW':desired_leader=.04
            elif phase in ('PLAN_BRAKE','PLANNING'):desired_leader=0.
            else:desired_leader=max(0.,min(.04,raw[3]*math.cos(f[2])+.5*(f[0]+gap-lx)))
            if a.continuous20 and near_pause.active:desired_leader=0.
    except Abort as exc:reason=str(exc)
    except (ValueError,KeyError,TypeError) as exc:
        detail=str(exc);reason=detail if detail.isupper() and len(detail)<100 else 'LOCAL_INPUT_OR_PLAN_INVALID'
        log('exception',detail=detail)
        if hasattr(exc,'apf_diagnostic'):
            log('apf_planning_failed',diagnostic=exc.apf_diagnostic)
            print('APF_PLAN_DIAGNOSTIC='+json.dumps(exc.apf_diagnostic,ensure_ascii=False),flush=True)
        if hasattr(exc,'diagnostic') and isinstance(exc.diagnostic,dict) and 'attempts' in exc.diagnostic:
            log('planning_failed',diagnostic=exc.diagnostic)
            examples=[dict(advance=x['advance'],offset=x['offset'],buffer=x['buffer'],blocking=x.get('blocking')) for x in exc.diagnostic['attempts'][:3]]
            print(json.dumps(dict(planning_failure=detail,candidates_checked=len(exc.diagnostic['attempts']),
                                 search_time_limit_hit=exc.diagnostic['search_time_limit_hit'],examples=examples),ensure_ascii=False),flush=True)
    except (KeyboardInterrupt,EOFError):reason='OPERATOR_STOP'
    except Exception as exc:reason='LOCAL_TOOL_ERROR';log('exception',detail=repr(exc))
    finally:
        stopping=True
        if leader_started:
            try:leader_command(0.);log('leader_stop_requested')
            except Exception as exc:log('leader_stop_error',detail=str(exc))
        if arm_requested:
            try:
                if disarm.service_is_ready():disarm.call_async(Trigger.Request());log('follower_disarm_requested')
            except Exception as exc:log('disarm_error',detail=str(exc))
        if leader_started or arm_requested:
            end=time.monotonic()+3.;motor_count=None;leader_stamp=None;mz=lz=0
            while time.monotonic()<end:
                if not rclpy.ok():break
                try:spin()
                except Abort:break
                mono=time.monotonic()
                if motor is not None:
                    s,rx=motor;count=s.get('valid_feedback')
                    if count!=motor_count:
                        motor_count=count;ok=(0<=mono-rx<=.3 and s.get('armed') is False and s.get('measured_rpm')==[0,0] and s.get('feedback_fresh') is True)
                        mz=mz+1 if ok else 0
                leader=core.inputs.get('leader')
                if leader and leader[0]!=leader_stamp:
                    leader_stamp=leader[0];ok=(0<=mono-leader[1]<=.3 and -.05<=(now()-leader[0])/1e9<=.3 and abs(leader[2][3])<=.01 and abs(leader[2][4])<=.01)
                    lz=lz+1 if ok else 0
                if mz>=3 and lz>=3:follower_zero=leader_zero=True;break
            follower_zero=follower_zero or mz>=3;leader_zero=leader_zero or lz>=3
        if scan_failure_frames is not None:
            # Serialize only after both stop requests/feedback checks. Capturing
            # does not delay the next zero command and never alters scan data.
            try:
                from local_scan_evidence import scan_evidence
                log('stop_scan_evidence',**scan_evidence(scan_failure_frames,list(raw_scan_ring),
                    list(getattr(core,'history',[])),pose_source=a.pose_source,
                    raw_monitor_history=list(getattr(core,'raw_history',[]))),
                    imu_orientation_history=list(getattr(core,'imu_history',[])))
            except Exception as exc:log('scan_evidence_error',detail=str(exc))
        success=(reason in ('TRIAL_TIME_LIMIT','GEOMETRIC_PASS_REJOIN_PLUS_1M','REJOIN_AND_FOLLOW_CONFIRMED','FOLLOW_CHECK_TIME_LIMIT') and motion_seen and arm_requested
                 and follower_zero and leader_zero and not faults)
        if a.continuous20:success=success and reason=='REJOIN_AND_FOLLOW_CONFIRMED'
        result=dict(reason=reason,leader_started=leader_started,arm_requested=arm_requested,
                    leader_zero_confirmed=leader_zero,follower_zero_confirmed=follower_zero,
                    post_stop_faults=sorted(faults),motion_feedback_seen=motion_seen,trial_success=success,
                    geometric_complete_candidate=reason in ('GEOMETRIC_PASS_REJOIN_PLUS_1M','REJOIN_AND_FOLLOW_CONFIRMED'),evaluation_pass=False,
                    controller_kind=controller_kind,trial_seconds=a.seconds,input_source_timeout_s=SOURCE_TIMEOUT_S,
                    plan_only=a.plan_only,plan_ready=reason=='PLAN_ONLY_READY')
        result['near_return_hold_enabled']=a.near_return_hold
        result['continuous_reobservation_enabled']=a.continuous20
        result['pose_source']=a.pose_source
        result['dynamic_box']=a.dynamic_box
        result['post_rejoin_follow_target_m']=a.post_rejoin_follow_m
        result['obstacle_front_trigger_m']=.7 if a.dynamic_box else .9
        result['persistent_replan']=False
        result['continuous20']=a.continuous20
        result['clearance_policy_m']=.2 if a.continuous20 else .3
        result['stop_clearance_m']=.1 if a.continuous20 else .3
        result['follow_lookahead_m']=.35 if a.continuous20 else .8
        result['tracked_pursuit_handoff_confirmed']=handoff_confirmed
        result['frame_check_distance_m']=a.frame_check_distance
        result['frame_check_complete']=reason=='FRAME_CHECK_COMPLETE' and follower_zero and leader_zero and not faults
        result['follow_only']=a.follow_only
        result['follow_check_seconds']=a.follow_check_seconds if a.follow_only else None
        print(json.dumps(result,ensure_ascii=False,indent=2),flush=True);log('end',**result)
        if (leader_started or arm_requested) and not (leader_zero and follower_zero):print('정지 피드백 미확인. 실물이 멈추지 않으면 전원을 차단하세요.',flush=True)
        worker.shutdown(wait=False,cancel_futures=True);logstream.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    return 0 if success or reason=='PLAN_ONLY_READY' else 1

if __name__=='__main__':raise SystemExit(main())
