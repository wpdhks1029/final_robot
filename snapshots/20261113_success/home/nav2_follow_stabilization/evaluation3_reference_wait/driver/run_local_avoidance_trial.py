#!/usr/bin/env python3
"""Explicit joint bounded local avoidance trial. Default starts nothing."""
import argparse,json,math,os,time
import traceback
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
from local_trial_preflight import graph_issues,laser_mount_issue,mount_snapshot
from follow_input_readiness import InputReadiness
from follow_reference_recovery import FollowReferenceRecovery, recoverable_reference_loss

ROOT=Path(__file__).resolve().parent


def motor_zero_feedback_trusted(status):
    """Faulted encoder feedback cannot establish physical zero speed."""
    device=status.get('device_status')
    return (status.get('armed') is False and status.get('fault') is None
            and status.get('io_failed') is False
            and status.get('feedback_fresh') is True
            and status.get('measured_rpm')==[0,0]
            and (device is None or device==dict(m1=0,m2=0)))


def motor_problem(sample,mono,seconds,stationary=False,continuous20=False,follow_only=False,allow_disabled=False,disable_clearance_stop=False,expected_max_rpm=10):
    if sample is None or not 0<=mono-sample[1]<=.3:return 'MOTOR_STATUS_MISSING_OR_STALE'
    s=sample[0]
    if s.get('clearance_stop_enabled',True) != (not disable_clearance_stop):
        return 'DRIVER_CLEARANCE_POLICY_MISMATCH'
    if bool(s.get('continuous20',False))!=continuous20:
        return ('FOLLOW_ONLY_REQUIRES_STANDARD_DRIVER_WITHOUT_CONTINUOUS20_ACK'
                if follow_only else 'REQUIRE_MATCHING_CONTINUOUS20_DRIVER')
    if continuous20 and s.get('continuous_stop_clearance_m')!=.1:return 'REQUIRE_CONTINUOUS10_STOP_DRIVER_RESTART'
    if s.get('distance_test_only'):return 'DISTANCE_TEST_DRIVER_NOT_FOR_AVOIDANCE'
    if (s.get('mode')!='HARDWARE_CANDIDATE' or s.get('input_kind')!='local_avoidance'
            or s.get('motion_profile')!=PROFILE or s.get('input_source_timeout_s')!=SOURCE_TIMEOUT_S or s.get('trial_max_rpm')!=expected_max_rpm
            or s.get('trial_max_seconds')!=seconds or (s.get('enabled') is not True
            and not (allow_disabled and s.get('enabled') is False))):return 'REQUIRE_LOCAL_AVOIDANCE_DRIVER_MATCH'
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
    p.add_argument('--seconds',type=int,choices=(0,30,360),default=30,help='0: no session limit; Nav2 automatic follow-only mode')
    p.add_argument('--apf-pivot',action='store_true',help='APF-guided checked pivot/10cm steps; explicit candidate')
    p.add_argument('--gentle-pivot',action='store_true',help='Require a complete checked diagonal bypass; prefer smaller initial turns')
    p.add_argument('--map-rejoin',action='store_true',help='Use live map pose and leader path targets after bypass')
    p.add_argument('--continue-follow-after-rejoin',action='store_true',help='After verified rejoin, stop then hand off to tracked Pure Pursuit for a bounded confirmation')
    p.add_argument('--frame-check-distance',type=float,choices=(.2,),default=None,help='Follower-only coordinate diagnostic: checked initial pivot and20cm translation, then stop')
    p.add_argument('--follow-only',action='store_true',help='Tracked Pure Pursuit only; fail-closed stop on obstacle, no local detour')
    p.add_argument('--follow-max-linear',type=float,choices=(.04,.05,.06,.08,.10),default=.04,
                   help='Follow-only speed cap in m/s; default preserves 0.04')
    p.add_argument('--follow-min-linear',type=float,choices=(0.,.04),default=0.,
                   help='Experimental Nav2 follow speed floor; disabled by default, not physically validated')
    p.add_argument('--exclude-body-internal-scan',action='store_true',help='Follow-only: exclude points strictly inside existing base_link footprint; retain raw scan and exterior stop')
    p.add_argument('--disable-clearance-stop',action='store_true',help='Nav2 follow-only: disable follower LiDAR distance stops; requires matching driver setting')
    p.add_argument('--tf-static-topic',choices=('/tf_static','/follower/tf_static_internal'),
                   default='/tf_static',help='Use the mount TF topic matching the sensor launch')
    p.add_argument('--follow-check-seconds',type=int,choices=(0,10,20,360),default=None)
    p.add_argument('--auto-start-on-goal','--auto-start-on-plan',dest='auto_start_on_plan',action='store_true',help='For Nav2 follow-only: arm on an active 2D goal; Pure Pursuit follows the measured leader bridge path')
    p.add_argument('--prepare-only',action='store_true',help='Check consecutive input readiness and exit without arm or motion; accepts a disabled driver')
    p.add_argument('--direct-unicast',action='store_true',help='Use the separate latest-state unicast relay for leader pose/path/goal status')
    p.add_argument('--near-return-hold',action='store_true',help='APF only: zero output until 5 fresh clear scans; maximum 2s')
    p.add_argument('--pose-source',choices=('raw','filtered'),default='raw',help='Pose and measured-time scan projection source; raw remains monitored')
    p.add_argument('--reference-plan',type=Path,default=ROOT/'logs/current_scene_left_detour_plan.json')
    p.add_argument('--continuous20',action='store_true')
    p.add_argument('--nav2-leader',action='store_true',help='Use externally controlled Nav2 leader; this process never publishes /cmd_vel')
    p.add_argument('--dynamic-box',action='store_true',help='Start following without a prepared box; detect a newly appearing box and plan once after braking')
    p.add_argument('--post-rejoin-follow-m',type=float,choices=(1.,2.),default=1.,help='Continuous mode: map-verified following distance after rejoin')
    p.add_argument('--ack-extended-follow-speed',action='store_true',help='Opt-in 0.08/0.10m/s and matching 15RPM driver')
    p.add_argument('--smooth-follow',action='store_true',help='Continuous gap/leader-speed control with bounded acceleration; opt-in')
    a=p.parse_args()
    if a.seconds==0 and not (a.run and a.nav2_leader and a.follow_only and a.auto_start_on_plan):
        p.error('--seconds 0 requires automatic Nav2 follow-only mode')
    if a.follow_check_seconds==0 and a.seconds!=0:
        p.error('--follow-check-seconds 0 requires --seconds 0')
    if a.smooth_follow and not (a.nav2_leader and a.follow_only and a.auto_start_on_plan and a.ack_extended_follow_speed):
        p.error('--smooth-follow requires extended Nav2 automatic follow-only mode')
    if a.smooth_follow and a.follow_min_linear:
        p.error('--smooth-follow cannot use a minimum-speed override')
    if (a.follow_max_linear>.06)!=a.ack_extended_follow_speed:
        p.error('0.08/0.10 requires --ack-extended-follow-speed; low-speed profiles use the original limits')
    if a.ack_extended_follow_speed and not (a.nav2_leader and a.follow_only):
        p.error('Extended speed requires Nav2 follow-only')
    expected_rpm=15 if a.ack_extended_follow_speed else 10
    if a.disable_clearance_stop and not (a.nav2_leader and a.follow_only):
        p.error('--disable-clearance-stop requires --nav2-leader --follow-only')
    if a.direct_unicast and not (a.nav2_leader and a.follow_only):
        p.error('--direct-unicast requires --nav2-leader --follow-only')
    if a.follow_max_linear!=.04 and not a.follow_only:
        p.error('--follow-max-linear requires --follow-only')
    if a.follow_min_linear and not (a.nav2_leader and a.follow_only):
        p.error('--follow-min-linear requires --nav2-leader --follow-only')
    if a.follow_min_linear>a.follow_max_linear:
        p.error('--follow-min-linear must not exceed --follow-max-linear')
    if a.exclude_body_internal_scan and not a.follow_only:
        p.error('--exclude-body-internal-scan requires --follow-only')
    if a.dynamic_box and not a.continuous20:p.error('--dynamic-box requires --continuous20')
    if a.nav2_leader:
        follow_only_ready = (a.follow_only and not a.continuous20 and not a.dynamic_box)
        avoidance_ready = (a.continuous20 and a.dynamic_box and a.map_rejoin
                           and a.continue_follow_after_rejoin and not a.follow_only)

        if not a.run or not (follow_only_ready or avoidance_ready):
            p.error('--nav2-leader requires follow-only with map-rejoin, or the full avoidance option set')
    if a.post_rejoin_follow_m!=1. and not a.continuous20:p.error('--post-rejoin-follow-m requires --continuous20')
    if a.continuous20 and a.near_return_hold:p.error('Continuous20 uses immediate safety stop, not legacy near-return pause')
    if a.continuous20 and (not a.map_rejoin or not a.continue_follow_after_rejoin or a.frame_check_distance or a.follow_only):p.error('Continuous20 requires map rejoin with handoff')
    if a.frame_check_distance and (not a.gentle_pivot or a.follow_only or a.continue_follow_after_rejoin):p.error('--frame-check-distance requires gentle filtered pivot without follow modes')
    if a.follow_only and a.continuous20:p.error('--follow-only is separate from continuous avoidance')
    if a.auto_start_on_plan and not (a.run and a.nav2_leader and a.follow_only):
        p.error('--auto-start-on-plan requires --run --nav2-leader --follow-only')
    if a.prepare_only and not (a.run and a.nav2_leader and a.follow_only and a.auto_start_on_plan):
        p.error('--prepare-only requires --run --nav2-leader --follow-only --auto-start-on-goal')
    if a.follow_check_seconds is None:
        a.follow_check_seconds=(0 if a.seconds==0 else 360) if a.auto_start_on_plan else 10
    if a.auto_start_on_plan and a.follow_check_seconds!=(0 if a.seconds==0 else 360):
        p.error('Automatic follow time limit must match the session policy')
    if a.continue_follow_after_rejoin and not a.map_rejoin:p.error('--continue-follow-after-rejoin requires --map-rejoin')
    if a.map_rejoin and not a.gentle_pivot and not a.follow_only:p.error('--map-rejoin requires --gentle-pivot')
    if a.near_return_hold and not a.apf_pivot:p.error('--near-return-hold requires --apf-pivot')
    if a.gentle_pivot and not a.follow_only and (not a.apf_pivot or a.pose_source!='filtered'):
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
    reference=None if a.follow_only else json.loads(a.reference_plan.read_text())
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry,Path as RosPath
    from sensor_msgs.msg import LaserScan,Imu
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from tf2_msgs.msg import TFMessage
    if a.auto_start_on_plan:
        from action_msgs.msg import GoalStatusArray
    from follower_remote_qos import latest_state_qos
    from follower_pursuit_preview import PreviewCore
    from watch_apf_case3 import project
    from follower_input_guard import valid_odom
    from nav2_follow_safety import build_scene as build_follow_scene, bounded_pursuit_result
    from nav2_goal_state import has_active_goal
    rclpy.init(args=[],signal_handler_options=SignalHandlerOptions.NO)
    node=rclpy.create_node('local_avoidance_trial_operator')
    node._use_direct_unicast=a.direct_unicast
    core=core_type(pose_source='filtered') if a.pose_source=='filtered' else core_type()
    if a.map_rejoin or a.follow_only:
        from tracked_pursuit_candidate import TrackedPreviewCore
        nav=TrackedPreviewCore(lookahead=.35 if a.continuous20 else .8)
    else:nav=PreviewCore()
    if a.smooth_follow:
        from smooth_tracked_pursuit import SmoothTrackedPreviewCore
        nav=SmoothTrackedPreviewCore(lookahead=.8)
    nav.timeout=SOURCE_TIMEOUT_S
    motor=None;mount=None;mount_observed=None;mount_problem='LASER_MOUNT_TF_MISSING'
    leader_started=False;arm_requested=False;stopping=False;epoch=None
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
    leadpub=None if (a.plan_only or a.nav2_leader) else node.create_publisher(Twist,'/cmd_vel',1)
    statuspub=None if a.plan_only else node.create_publisher(String,'/follower/local_avoidance/status',latest_state_qos())
    arm=node.create_client(Trigger,'/follower/motor_candidate/arm');disarm=node.create_client(Trigger,'/follower/motor_candidate/disarm')
    class Abort(Exception):pass
    def now():return node.get_clock().now().nanoseconds
    def log(event,**data):
        logstream.write(json.dumps(dict(event=event,ros_ns=now(),**data),allow_nan=False)+'\n');logstream.flush()
    def leader_command(v):
        if a.nav2_leader:
            # Leader is controlled only by Nav2 through nav2_convoy_gate.
            return
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
    nav_goal_active=[False]
    nav_goal_last_report=[None]
    leader_path_seen=[False]
    reference_recovery=FollowReferenceRecovery()
    last_map_pose_log={}
    last_input_rejection_report={}
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
            if item is None and rx-last_input_rejection_report.get(key,float('-inf'))>=2.:
                diagnostic=getattr(nav,'last_input_rejection',None)
                if diagnostic is not None and diagnostic.get('key')==key:
                    last_input_rejection_report[key]=rx
                    print('ODOM_INPUT_REJECTION='+json.dumps(diagnostic,ensure_ascii=False),flush=True)
                    log('odom_input_rejection',diagnostic=diagnostic)
            if item is not None and rx-last_map_pose_log.get(key,float('-inf'))>=.2:
                last_map_pose_log[key]=rx
                log('map_pose_trace',robot=key,source_ns=stamp,
                    pose=list(item[2][:3]),twist=list(item[2][3:]),
                    trial_phase=phase)
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
        nonlocal mount,mount_observed,mount_problem
        for tf in m.transforms:
            if tf.child_frame_id=='follower/laser':
                mount_observed=tf
                mount_problem=laser_mount_issue(tf)
                mount=tf if mount_problem is None else None
    def motor_cb(m):
        nonlocal motor,motion_seen
        try:
            s=json.loads(m.data)
            if not isinstance(s,dict):raise ValueError('status not object')
            motor=(s,time.monotonic());log('motor',status=s)
            if s.get('armed') and any(abs(v)>0 for v in s.get('measured_rpm',[0,0])):motion_seen=True
            if stopping and s.get('fault'):faults.add(str(s['fault']))
        except (ValueError,TypeError):motor=None
    # The follower follows the leader bridge's measured map path. Waiting for
    # controller_server's private received_global_plan can deadlock startup:
    # the Nav2 gate intentionally holds the leader until the follower is armed.
    nav2_goal_follow = bool(a.nav2_leader and a.follow_only)
    follow_wait_reasons=('STOP_LEADER_STOPPED','STOP_FOLLOW_DISTANCE')
    if nav2_goal_follow:follow_wait_reasons+=('STOP_LEADER_ROTATING','STOP_LEADER_REVERSE')
    use_auto_nav2_goal = bool(a.nav2_leader and a.follow_only and a.auto_start_on_plan)
    node._require_nav2_plan = False
    node._require_nav2_goal_status = use_auto_nav2_goal
    subs=[node.create_subscription(Odometry,'/follower/odom_raw',lambda m:odom_cb('raw',m),latest_state_qos()),
          node.create_subscription(Odometry,'/follower/odom_map',lambda m:odom_cb('follower',m),latest_state_qos()),
          node.create_subscription(Odometry,'/leader/odom_unicast' if a.direct_unicast else '/leader/odom',lambda m:odom_cb('leader',m),latest_state_qos()),
          node.create_subscription(LaserScan,'/follower/scan',scan_cb,latest_state_qos()),
          node.create_subscription(String,'/follower/motor_candidate/status',motor_cb,latest_state_qos()),
          node.create_subscription(TFMessage,a.tf_static_topic,tf_cb,QoSProfile(depth=20,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL))]
    last_path_rejection_report=[float('-inf')]
    def leader_path_cb(m):
        had_path='path' in nav.inputs
        nav.path_callback(m,now(),time.monotonic(),key='leader_path')
        nav.path_callback(m,now(),time.monotonic(),key='path')
        has_path='path' in nav.inputs
        if has_path and not leader_path_seen[0]:
            leader_path_seen[0]=True
            print(f'[NAV2] 선두 실측 경로 수신: {len(nav.inputs["path"][2])}점',flush=True)
        if not has_path and time.monotonic()-last_path_rejection_report[0]>=2.:
            last_path_rejection_report[0]=time.monotonic()
            diagnostic=dict(getattr(nav, 'last_input_rejection', None) or {}, frame=m.header.frame_id,
                            points_received=len(m.poses))
            print('PATH_DIAGNOSTIC='+json.dumps(diagnostic,ensure_ascii=False),flush=True)
            log('leader_path_rejected',diagnostic=diagnostic)
    subs.append(node.create_subscription(RosPath,'/leader/path_unicast' if a.direct_unicast else '/leader/path',leader_path_cb,latest_state_qos()))
    if use_auto_nav2_goal:
        def goal_status_cb(msg):
            # Nav2 publishes this latched action status as soon as the goal is
            # accepted. The follower can arm while the leader is still held.
            nav_goal_active[0]=has_active_goal(msg.status_list)
            summary=tuple(sorted((int(s.status) for s in msg.status_list)))
            if summary!=nav_goal_last_report[0]:
                nav_goal_last_report[0]=summary
                print(f'[NAV2] NavigateToPose 상태={summary}, active={nav_goal_active[0]}',flush=True)
        subs.append(node.create_subscription(GoalStatusArray,
            '/leader/nav2_goal_status_unicast' if a.direct_unicast else '/navigate_to_pose/_action/status',goal_status_cb,latest_state_qos()))
    if a.pose_source=='filtered':
        subs.append(node.create_subscription(Odometry,'/follower/odometry/filtered',lambda m:odom_cb('filtered',m),latest_state_qos()))
        subs.append(node.create_subscription(Imu,'/follower/imu/data',imu_cb,latest_state_qos()))
    def graph_problem():
        return graph_issues(node,nav2_leader=a.nav2_leader,command_topic=command_topic,
                            node_name=node.get_name(),plan_only=a.plan_only,
                            pose_source=a.pose_source,follow_only=a.follow_only)
    def graph_ok():
        return not graph_problem()
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
            leader_command(0. if stopping or (a.seconds>0 and mono-started>a.seconds+4.) else desired_leader);last_leader=mono
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
        path=nav.inputs.get('leader_path')
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
    last_body_filter_scan=[None]
    def follow_scene(require_reference=True):
        """Nav2 follow-only scene: map pose + latest base-frame scan, no APF/odom interpolation."""
        row=nav.inputs.get('follower')
        scan=core.scan
        scene=build_follow_scene(pose_row=(
            row[0].sec*10**9+row[0].nanosec, row[1], row[2]) if row else None,
            scan_row=scan, now_ns=now(), now_mono=time.monotonic(),
            timeout_s=SOURCE_TIMEOUT_S,exclude_body_internal=a.exclude_body_internal_scan)
        if scene['excluded_body_points'] and scene['scan_stamp']!=last_body_filter_scan[0]:
            last_body_filter_scan[0]=scene['scan_stamp']
            log('body_scan_filter',scan_stamp_ns=scene['scan_stamp'],
                excluded_base_points=scene['excluded_body_points'],
                footprint_half_extents_m=[.27,.30],boundary_preserved=True)
        if not require_reference:
            return scene
        from follower_stamped_preview import oldest_pursuit_source
        path=nav.inputs.get('path')
        pursuit_source=oldest_pursuit_source(nav.inputs,remote_clock_margin_ns=50_000_000,
                                               include_path=path is not None)
        if pursuit_source<=0 or not 0<=(now()-pursuit_source)/1e9<=SOURCE_TIMEOUT_S:
            raise ValueError('FOLLOW_REFERENCE_STALE_OR_MISSING')
        scene['source']=min(scene['source'],pursuit_source)
        scene['source_stamps']['leader']=nav.inputs['leader'][0].sec*10**9+nav.inputs['leader'][0].nanosec-50_000_000
        if path is not None:
            path_ns=path[0].sec*10**9+path[0].nanosec-50_000_000
            if not (path_ns>0 and 0<=(now()-path_ns)/1e9<=SOURCE_TIMEOUT_S
                    and 0<=time.monotonic()-path[1]<=SOURCE_TIMEOUT_S):
                raise ValueError('FOLLOW_LEADER_PATH_STALE')
            scene['source']=min(scene['source'],path_ns)
            scene['source_stamps']['path']=path_ns
        scene['follow_source_ns']=pursuit_source
        return scene
    origin=heading=None;planning=None;planning_started=None;box_count=0;last_box_scan=None;phase='PREFLIGHT'
    try:
        if nav2_goal_follow:
            print('선두 Nav2 goal → 후발 Pure Pursuit 연결 준비. Goal 승인 전에는 주행/arm하지 않습니다.',flush=True)
        else:
            print('로컬 회피 연결 확인. 아직 주행/arm 요청 없음.',flush=True)
        print('TRIAL_LOG='+str(logfile),flush=True)
        end=time.monotonic()+(360. if a.auto_start_on_plan else (60. if nav2_goal_follow else 12.));last_problem='WAIT_INPUTS';last_graph_issues=[]
        last_wait_report=None;last_wait_report_mono=0.
        input_readiness=InputReadiness()
        if nav2_goal_follow:
            print('대기 중: RViz에서 선두의 2D Goal을 지정하세요. 수락되면 입력 점검 후 후발이 자동 arm됩니다.',flush=True)
        while time.monotonic()<end:
            spin()
            problem=motor_problem(motor,time.monotonic(),a.seconds,True,continuous20=a.continuous20,follow_only=a.follow_only,allow_disabled=a.prepare_only,disable_clearance_stop=a.disable_clearance_stop,expected_max_rpm=expected_rpm)
            if problem:
                input_readiness.update(False,time.monotonic())
                last_problem=problem
                if problem=='FOLLOW_ONLY_REQUIRES_STANDARD_DRIVER_WITHOUT_CONTINUOUS20_ACK':
                    raise Abort(problem)
                if last_problem!=last_wait_report or time.monotonic()-last_wait_report_mono>=2.:
                    print('[PREFLIGHT] '+last_problem,flush=True);last_wait_report=last_problem;last_wait_report_mono=time.monotonic()
                continue
            last_graph_issues=graph_problem()
            if last_graph_issues:last_problem='ROS_GRAPH_NOT_READY'
            elif mount is None:last_problem=mount_problem
            if last_graph_issues or mount is None:
                input_readiness.update(False,time.monotonic())
                if last_problem!=last_wait_report or time.monotonic()-last_wait_report_mono>=2.:
                    print('[PREFLIGHT] '+last_problem,flush=True);last_wait_report=last_problem;last_wait_report_mono=time.monotonic()
                continue
            try:
                if a.follow_only:
                    scene=follow_scene()
                    origin=(scene['pose'].x,scene['pose'].y);heading=scene['pose'].yaw
                else:
                    origin,heading=fresh_reference();scene=core.scene(now(),time.monotonic())
                if a.follow_only:
                    _,_,preview_reason=nav.command(now(),time.monotonic())
                    route_wait=(use_auto_nav2_goal and not leader_path_seen[0] and preview_reason in
                                ('STOP_MISSING_OR_INVALID_PATH','STOP_PATH_NEEDS_TWO_POINTS',
                                 'STOP_PATH_NEEDS_TWO_DISTINCT_POINTS'))
                    if preview_reason.startswith('STOP_') and preview_reason not in follow_wait_reasons and not route_wait:
                        input_readiness.update(False,time.monotonic())
                        last_problem=preview_reason;continue
                if not arm.service_is_ready() or not disarm.service_is_ready():raise ValueError('SERVICE_NOT_READY')
                if nav2_goal_follow:
                    ready=input_readiness.update(True,time.monotonic())
                    if not ready or (use_auto_nav2_goal and not a.prepare_only and not nav_goal_active[0]):
                        last_problem=('WAITING_FOR_STABLE_INPUTS_5S' if not ready
                                      else 'INPUTS_READY_WAITING_FOR_ACTIVE_NAV2_GOAL')
                        if last_problem!=last_wait_report or time.monotonic()-last_wait_report_mono>=2.:
                            print('[PREFLIGHT] '+last_problem,flush=True)
                            last_wait_report=last_problem;last_wait_report_mono=time.monotonic()
                        continue
                break
            except ValueError as exc:
                input_readiness.update(False,time.monotonic())
                last_problem=str(exc)
                if last_problem!=last_wait_report or time.monotonic()-last_wait_report_mono>=2.:
                    print('[PREFLIGHT] '+last_problem,flush=True);last_wait_report=last_problem;last_wait_report_mono=time.monotonic()
                    from follow_input_diagnostics import snapshot
                    diagnostic=dict(reason=last_problem, inputs=snapshot(samples, now(), time.monotonic()))
                    if 'ambiguous' in last_problem:
                        diagnostic['traceback']=traceback.format_exc()
                    print('PREFLIGHT_INPUTS='+json.dumps(diagnostic,ensure_ascii=False),flush=True)
                    log('preflight_inputs',diagnostic=diagnostic)
        else:
            if last_problem in ('ROS_GRAPH_NOT_READY','LASER_MOUNT_TF_MISSING',
                                'LASER_MOUNT_FRAME_MISMATCH','LASER_MOUNT_VALUE_MISMATCH'):
                diagnostic={
                    'reason':last_problem,
                    'graph_issues':last_graph_issues,
                    'mount_issue':mount_problem,
                    'mount':mount_snapshot(mount_observed),
                }
                print('PREFLIGHT_DIAGNOSTIC='+json.dumps(diagnostic,ensure_ascii=False),flush=True)
                log('preflight_diagnostic',diagnostic=diagnostic)
            if reference_diagnostic is not None and last_problem.startswith('SAVED_REFERENCE'):
                print('REFERENCE_ALIGNMENT='+json.dumps(reference_diagnostic,ensure_ascii=False),flush=True)
                log('reference_alignment',diagnostic=reference_diagnostic)
            raise Abort('PREFLIGHT_'+last_problem)
        if a.prepare_only:
            print('FOLLOW_INPUTS_READY_NO_ARM: 입력이 5초 연속 유효했습니다. 주행·arm 없이 종료합니다.',flush=True)
            raise Abort('FOLLOW_INPUTS_READY_NO_ARM')
        start_pose=scene['pose']
        log('planning_input',points=scene['points'].tolist(),start=[start_pose.x,start_pose.y,start_pose.yaw],
            source_stamp_ns=scene['source'],scan_stamp_ns=scene['scan_stamp'],reference_origin=origin,reference_heading=heading,
            pose_source='map' if a.follow_only else a.pose_source,
            pose_anchor=list(core.inputs[a.pose_source][2]) if not a.follow_only else None,
            imu_anchor=core.inputs.get('imu') if not a.follow_only else None,
            raw_anchor=core.inputs.get('raw') if not a.follow_only else None)
        if a.follow_only or a.dynamic_box:
            plan=None;phase='FOLLOW'
            log('nav2_follow_ready' if a.follow_only else 'dynamic_box_follow_ready',
                obstacle_front_trigger_m=None if nav2_goal_follow else .9,control_connected=False,
                    automatic_start_on_goal=a.auto_start_on_plan)
        else:
            future=worker.submit(planner,scene['points'].tolist(),(start_pose.x,start_pose.y,start_pose.yaw))
            end=time.monotonic()+(65. if a.continuous20 else 15.)
            while not future.done() and time.monotonic()<end:
                spin()
                problem=motor_problem(motor,time.monotonic(),a.seconds,True,continuous20=a.continuous20,follow_only=a.follow_only,disable_clearance_stop=a.disable_clearance_stop,expected_max_rpm=expected_rpm)
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
                v,w,preview_reason=nav.command(now(),time.monotonic())
                probe=bounded_pursuit_result(scene,v,w,preview_reason,max_linear=a.follow_max_linear,allow_leader_rotation_wait=nav2_goal_follow,allow_leader_reverse_wait=nav2_goal_follow,enforce_clearance=not a.disable_clearance_stop,min_linear=a.follow_min_linear,max_rpm=expected_rpm)
                log('follow_check_preview',selection=nav.last_selection,control=probe,control_connected=False)
                print(json.dumps(dict(reason=probe['reason'],clearance_m=probe['clearance_m'],swept_clearance_m=probe['swept_clearance_m'],selection=nav.last_selection,control_connected=False),ensure_ascii=False),flush=True)
                if probe['reason'].startswith('STOP_'):raise Abort(probe['reason'])
            if plan is not None:
                saved=folder/f'local_plan_preview_{time.time_ns()}.json'
                saved.write_text(json.dumps(dict(plan,start=[start_pose.x,start_pose.y,start_pose.yaw],control_connected=False),allow_nan=False)+'\n')
                print('PLAN_JSON='+str(saved),flush=True)
                print(json.dumps(dict(reason='PLAN_READY',config=plan['config'],geometry=plan['geometry'],attempts=len(plan['attempts']),control_connected=False),ensure_ascii=False),flush=True)
            raise Abort('PLAN_ONLY_READY' if plan is not None or a.follow_only or a.dynamic_box else 'PLAN_ONLY_NO_OBSTACLE')
        if use_auto_nav2_goal:
            print('Nav2 2D Goal 수락과 입력 사전점검 통과. 후발을 자동 arm합니다. 선두 경로가 생길 때까지 gate는 제한 속도로 대기합니다.',flush=True)
        else:
            print('현재 모드 '+phase+' / '+controller_kind+' / 자세 '+a.pose_source+'. START + Enter: 선두·후발 제한 시험 시작.',flush=True)
            if not wait_for_start(spin):raise Abort('START_CANCELLED')
        problem=motor_problem(motor,time.monotonic(),a.seconds,True,continuous20=a.continuous20,follow_only=a.follow_only,disable_clearance_stop=a.disable_clearance_stop,expected_max_rpm=expected_rpm)
        if problem or not graph_ok():raise Abort(problem or 'GRAPH_CHANGED')
        scene=follow_scene() if a.follow_only else core.scene(now(),time.monotonic())
        current=scene['pose']
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
        elif a.nav2_leader:
            # Arm the follower first. The gate releases Nav2 only after a fresh
            # FOLLOW heartbeat; this removes the leader-start/follower-arm race.
            started=time.monotonic();leader_started=True
            log('nav2_leader_supervised_start',leader_command_topic='/cmd_vel_smoothed',
                leader_motor_topic='/cmd_vel',gate='nav2_convoy_gate')
        else:
            started=time.monotonic();leader_started=True;leader_command(.04);log('leader_start')
            # Only the measured leader start is awaited; a stopped leader is later
            # permitted exclusively inside this explicit cooperative avoidance session.
            while time.monotonic()-started<4.:
                spin()
                if not graph_ok():raise Abort('GRAPH_CHANGED')
                problem=motor_problem(motor,time.monotonic(),a.seconds,True,continuous20=a.continuous20,follow_only=a.follow_only,disable_clearance_stop=a.disable_clearance_stop,expected_max_rpm=expected_rpm)
                if problem:raise Abort(problem)
                core.scene(now(),time.monotonic())
                if core.inputs['leader'][2][3]>.03 and (phase=='AVOID' or 'path' in nav.inputs):break
            else:raise Abort('LEADER_START_TIMEOUT')
        arm_requested=True;response=call(arm);epoch=json.loads(response.message)['epoch'];armed=time.monotonic();log('follower_arm',epoch=epoch)
        if a.smooth_follow:nav.reset_motion()
        initial_leader=core.inputs['leader'][2]
        leader_start_x=math.cos(heading)*(initial_leader[0]-origin[0])+math.sin(heading)*(initial_leader[1]-origin[1])
        gap=max(1.54,leader_start_x-start_pose.x)
        if a.frame_check_distance:
            print('후발 좌표 진단: 선두 정지 유지. 초기 회전 후20cm 이동 또는 최대60초에 종료.',flush=True)
        elif a.follow_only:
            policy='후발 LiDAR 거리 정지 해제 상태입니다.' if a.disable_clearance_stop else '안전거리 위반 시 정지합니다.'
            duration='시간 제한 없음' if a.seconds==0 else f'최대 {a.follow_check_seconds}초'
            print(f'Nav2 목표 경로 Pure Pursuit 추종 시작. {duration}; 장애물 회피는 수행하지 않고 '+policy,flush=True)
        else:
            print(f'로컬 회피 시작. 후발 최대 {a.seconds-.5}초. 회전/직진 사이 정지 확인.',flush=True)
        def hold_reference(reason, mono):
            nonlocal last_status
            # Stop intent relies only on fresh local pose/scan support. Do not
            # relabel the expired remote path/leader measurements as current.
            local=follow_scene(require_reference=False)
            if a.smooth_follow:nav.reset_motion()
            was_active=reference_recovery.active
            publish_zero_hold(local['source'],local['scan_stamp'])
            if mono-last_status>=.1 or not was_active:
                hold=dict(trial_phase='FOLLOW',motion='STOP',v=0.,w=0.,
                    reason='STOP_FOLLOW_REFERENCE_WAIT',follower_requested_v=0.,
                    follower_follow_reason=reason,status_ros_ns=now(),
                    source_stamp_ns=local['source'],scan_stamp_ns=local['scan_stamp'],
                    input_source_stamps=local['source_stamps'],
                    wait_reason=reason)
                hold['smooth_follow']=a.smooth_follow
                statuspub.publish(String(data=json.dumps(hold,allow_nan=False)))
                log('follow_reference_wait',**hold)
                last_status=mono
            reference_recovery.wait(mono)
            if not was_active:
                print('[FOLLOW] STOP: 입력 갱신 대기. 새 입력 검증 후 자동 재개합니다.',flush=True)

        while True:
            spin();mono=time.monotonic();ns=now()
            if path_observation_fault[0]:raise Abort(path_observation_fault[0])
            if a.frame_check_distance and mono-armed>=60.:raise Abort('FRAME_CHECK_TIME_LIMIT')
            if a.follow_only and a.follow_check_seconds>0 and mono-armed>=a.follow_check_seconds:raise Abort('FOLLOW_CHECK_TIME_LIMIT')
            if a.seconds>0 and mono-armed>=a.seconds-.5:raise Abort('TRIAL_TIME_LIMIT')
            if use_auto_nav2_goal and not nav_goal_active[0]:raise Abort('NAV2_GOAL_FINISHED')
            if not graph_ok():raise Abort('GRAPH_CHANGED')
            problem=motor_problem(motor,mono,a.seconds,continuous20=a.continuous20,follow_only=a.follow_only,disable_clearance_stop=a.disable_clearance_stop,expected_max_rpm=expected_rpm)
            if problem:raise Abort(problem)
            state=motor[0]
            if state.get('armed') and state.get('epoch')!=epoch:raise Abort('SESSION_CHANGED')
            if not state.get('armed') and mono-armed>.2:raise Abort('UNEXPECTED_DISARM')
            # A disarmed status may retain the previous trial's arm timestamp.
            # Never validate new commands against that cached session.
            if state.get('armed') is not True or state.get('epoch')!=epoch:
                continue
            try:scene=follow_scene() if a.follow_only else core.scene(ns,mono)
            except ValueError as exc:
                if (use_auto_nav2_goal and recoverable_reference_loss(str(exc),
                        getattr(nav,'last_input_rejection',None))):
                    hold_reference(str(exc),mono)
                    continue
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
            if use_auto_nav2_goal and reference_recovery.active:
                _,_,candidate_reason=nav.command(ns,mono)
                if recoverable_reference_loss(candidate_reason,getattr(nav,'last_input_rejection',None)):
                    hold_reference(candidate_reason,mono)
                    continue
                if candidate_reason.startswith('STOP_') and candidate_reason not in follow_wait_reasons:
                    raise Abort(candidate_reason)
                zero_ok=(state.get('feedback_fresh') is True and
                    state.get('measured_rpm')==[0,0] and state.get('zero_feedback_samples',0)>=3)
                if not reference_recovery.recovered(mono,scene['source_stamps'],zero_ok):
                    # Keep stable-input progress; unlike a loss, this does not
                    # reset its timer or its distinct measurement counters.
                    publish_zero_hold(scene['source_stamps']['follower_map'],scene['scan_stamp'])
                    if mono-last_status>=.1:
                        hold=dict(trial_phase='FOLLOW',motion='STOP',v=0.,w=0.,
                            reason='STOP_FOLLOW_REFERENCE_WAIT',follower_requested_v=0.,
                            follower_follow_reason='RECOVERY_VALIDATING',status_ros_ns=now(),
                            source_stamp_ns=scene['source'],scan_stamp_ns=scene['scan_stamp'],
                            input_source_stamps=scene['source_stamps'])
                        hold['smooth_follow']=a.smooth_follow
                        statuspub.publish(String(data=json.dumps(hold,allow_nan=False)))
                        log('follow_reference_validating',**hold);last_status=mono
                    continue
                log('follow_reference_resumed',input_source_stamps=scene['source_stamps'])
                if a.smooth_follow:nav.reset_motion()
                print('[FOLLOW] 입력 갱신·정지 피드백 확인. 추종을 재개합니다.',flush=True)
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
            if phase=='FOLLOW' and not nav2_goal_follow:
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
                follow_reason='FOLLOW_NOT_ACTIVE'
                if phase=='FOLLOW':
                    from follower_stamped_preview import oldest_pursuit_source
                    if use_auto_nav2_goal and not leader_path_seen[0]:
                        # The fail-closed gate needs an armed FOLLOW heartbeat
                        # before Nav2 can move and create the measured bridge
                        # path. Advertise the bounded startup-wait state.
                        v,w,follow_reason=0.,0.,'STOP_LEADER_STOPPED'
                    else:
                        v,w,follow_reason=nav.command(ns,mono)
                        if (use_auto_nav2_goal and not leader_path_seen[0] and follow_reason in
                                ('STOP_MISSING_OR_INVALID_PATH','STOP_PATH_NEEDS_TWO_POINTS',
                                 'STOP_PATH_NEEDS_TWO_DISTINCT_POINTS')):
                            # Let the leader move at the gate's bounded startup
                            # crawl until its measured path has two distinct points.
                            v,w,follow_reason=0.,0.,'STOP_LEADER_STOPPED'
                    if (a.map_rejoin or a.follow_only) and follow_reason.startswith('STOP_') and follow_reason not in follow_wait_reasons:
                        if use_auto_nav2_goal and recoverable_reference_loss(follow_reason,getattr(nav,'last_input_rejection',None)):
                            hold_reference(follow_reason,mono)
                            continue
                        log('tracked_pursuit_stop',reason=follow_reason,selection=nav.last_selection)
                        raise Abort(follow_reason)
                    follow_source=oldest_pursuit_source(nav.inputs,remote_clock_margin_ns=50_000_000,
                                                       include_path='path' in nav.inputs)
                    if follow_source<=0 or not 0<=(ns-follow_source)/1e9<=SOURCE_TIMEOUT_S:
                        if use_auto_nav2_goal:
                            hold_reference('FOLLOW_REFERENCE_STALE_OR_MISSING',mono)
                            continue
                        raise Abort('FOLLOW_REFERENCE_STALE_OR_MISSING')
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
            if a.follow_only:
                result=bounded_pursuit_result(scene,follow[0],follow[1],follow_reason,max_linear=a.follow_max_linear,allow_leader_rotation_wait=nav2_goal_follow,allow_leader_reverse_wait=nav2_goal_follow,enforce_clearance=not a.disable_clearance_stop,min_linear=a.follow_min_linear,max_rpm=expected_rpm)
            else:
                result=core.command(ns,mono,state,follow_command=follow)
            if map_guidance is not None:
                result['map_rejoin_guidance']=map_guidance
                result['source_stamp_ns']=min(result['source_stamp_ns'],map_guidance['source_stamp_ns'])
            result['compute_s']=time.monotonic()-compute_started
            result['smooth_follow']=a.smooth_follow
            if a.smooth_follow:
                nav.feedback_bounded(result['v'],result['w'],mono)
                result['smooth_speed']=dict(nav.speed_diagnostic,
                    bounded_v_mps=result['v'],bounded_w_radps=result['w'])
            if phase=='FOLLOW':result['source_stamp_ns']=min(result['source_stamp_ns'],follow_source)
            if phase=='FOLLOW':
                result['follower_follow_reason']=follow_reason
                result['follower_requested_v']=result['v'] if (a.follow_min_linear or a.ack_extended_follow_speed) else follow[0]
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
                if use_auto_nav2_goal:
                    hold_reference('INPUT_AGED_DURING_COMPUTE',time.monotonic())
                    continue
                raise Abort('INPUT_AGED_DURING_COMPUTE')
            obj=dict(version=3,controller=PROFILE,epoch=epoch,frame='follower/base_link',stamp_ns=sent,
                     source_stamp_ns=source,scan_stamp_ns=scan_stamp,motion=result['motion'],v=result['v'],w=result['w'],
                     clearance_m=result['clearance_m'],swept_clearance_m=result['swept_clearance_m'])
            commands.publish(String(data=json.dumps(obj,allow_nan=False)))
            if mono-last_status>=.1:
                result['status_ros_ns']=now()
                statuspub.publish(String(data=json.dumps(result,allow_nan=False)));log('local_control',**result);last_status=mono
            # Nav2 owns leader commands. Its map-follow mode does not depend
            # on the legacy local odometry controller's optional input cache.
            if not a.nav2_leader:
                if phase=='FOLLOW':desired_leader=.04
                elif phase in ('PLAN_BRAKE','PLANNING'):desired_leader=0.
                else:
                    leader=core.inputs['leader'][2]
                    lx=math.cos(heading)*(leader[0]-origin[0])+math.sin(heading)*(leader[1]-origin[1])
                    raw=core.inputs[a.pose_source][2];f=result['pose']
                    desired_leader=max(0.,min(.04,raw[3]*math.cos(f[2])+.5*(f[0]+gap-lx)))
                if a.continuous20 and near_pause.active:desired_leader=0.
    except Abort as exc:reason=str(exc)
    except (ValueError,KeyError,TypeError) as exc:
        detail=str(exc);reason=detail if detail.isupper() and len(detail)<100 else 'LOCAL_INPUT_OR_PLAN_INVALID'
        log('exception',detail=detail)
        print('TRIAL_DIAGNOSTIC='+json.dumps(dict(reason=reason,detail=detail),ensure_ascii=False),flush=True)
        if hasattr(exc,'apf_diagnostic'):
            log('apf_planning_failed',diagnostic=exc.apf_diagnostic)
            print('APF_PLAN_DIAGNOSTIC='+json.dumps(exc.apf_diagnostic,ensure_ascii=False),flush=True)
        if hasattr(exc,'diagnostic') and isinstance(exc.diagnostic,dict) and 'attempts' in exc.diagnostic:
            log('planning_failed',diagnostic=exc.diagnostic)
            examples=[dict(advance=x['advance'],offset=x['offset'],buffer=x['buffer'],blocking=x.get('blocking')) for x in exc.diagnostic['attempts'][:3]]
            print(json.dumps(dict(planning_failure=detail,candidates_checked=len(exc.diagnostic['attempts']),
                                 search_time_limit_hit=exc.diagnostic['search_time_limit_hit'],examples=examples),ensure_ascii=False),flush=True)
    except (KeyboardInterrupt,EOFError):reason='OPERATOR_STOP'
    except Exception as exc:
        reason='LOCAL_TOOL_ERROR';detail=traceback.format_exc()
        log('exception',detail=detail)
        print('TRIAL_DIAGNOSTIC='+json.dumps(dict(reason=reason,detail=detail),ensure_ascii=False),flush=True)
    finally:
        stopping=True
        if a.nav2_leader and statuspub is not None:
            try:
                statuspub.publish(String(data=json.dumps(dict(trial_phase='STOP',motion='STOP',reason=reason,status_ros_ns=now()))))
            except Exception as exc:log('nav2_gate_stop_status_error',detail=str(exc))
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
                        motor_count=count;ok=(0<=mono-rx<=.3 and motor_zero_feedback_trusted(s))
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
        success=(reason in ('TRIAL_TIME_LIMIT','GEOMETRIC_PASS_REJOIN_PLUS_1M','REJOIN_AND_FOLLOW_CONFIRMED','FOLLOW_CHECK_TIME_LIMIT','NAV2_GOAL_FINISHED') and motion_seen and arm_requested
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
        result['obstacle_front_trigger_m']=None if nav2_goal_follow else (.7 if a.dynamic_box else .9)
        result['persistent_replan']=False
        result['continuous20']=a.continuous20
        result['clearance_policy_m']=None if a.disable_clearance_stop else (.2 if a.continuous20 else .3)
        result['stop_clearance_m']=None if a.disable_clearance_stop else (.1 if a.continuous20 else .3)
        result['follow_lookahead_m']=.35 if a.continuous20 else .8
        result['tracked_pursuit_handoff_confirmed']=handoff_confirmed
        result['frame_check_distance_m']=a.frame_check_distance
        result['frame_check_complete']=reason=='FRAME_CHECK_COMPLETE' and follower_zero and leader_zero and not faults
        result['follow_only']=a.follow_only
        result['follower_rpm_limit']=expected_rpm
        result['extended_follow_speed']=a.ack_extended_follow_speed
        result['smooth_follow']=a.smooth_follow
        result['session_unlimited']=a.seconds==0
        result['follow_gap_deadband_m']=getattr(nav,'gap_deadband',None) if a.smooth_follow else None
        result['follow_max_linear_mps']=a.follow_max_linear if a.follow_only else None
        result['follow_min_linear_mps']=a.follow_min_linear if a.follow_only else None
        result['reference_recovery_waits']=reference_recovery.count
        result['reference_recovery_timeout_s']=reference_recovery.timeout_s if use_auto_nav2_goal else None
        result['follow_check_seconds']=a.follow_check_seconds if a.follow_only else None
        result['auto_start_on_plan']=a.auto_start_on_plan
        result['auto_start_on_goal']=use_auto_nav2_goal
        result['prepare_only']=a.prepare_only
        result['direct_unicast']=a.direct_unicast
        result['exclude_body_internal_scan']=a.exclude_body_internal_scan
        result['clearance_stop_enabled']=not a.disable_clearance_stop
        result['leader_control']='NAV2_THROUGH_FAIL_CLOSED_GATE' if a.nav2_leader else 'BOUNDED_TRIAL_PUBLISHER'
        print(json.dumps(result,ensure_ascii=False,indent=2),flush=True);log('end',**result)
        if (leader_started or arm_requested) and not (leader_zero and follower_zero):print('정지 피드백 미확인. 실물이 멈추지 않으면 전원을 차단하세요.',flush=True)
        worker.shutdown(wait=False,cancel_futures=True);logstream.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    return 0 if success or reason in ('PLAN_ONLY_READY','FOLLOW_INPUTS_READY_NO_ARM') else 1

if __name__=='__main__':raise SystemExit(main())
