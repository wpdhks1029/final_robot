#!/usr/bin/env python3
"""실제 scan/TF/pose/path의 임시 경로 계산. 발행/arm/모터 접근 없음."""
import argparse,json,math,os,time
from pathlib import Path
from temporary_detour_candidate import Settings,Pose,reference_frame,to_reference,plan


def stationary_inputs(nav,motor,now,mono):
    if motor is None or not 0<=mono-motor[1]<=.3:raise ValueError('MOTOR_STATUS_MISSING_OR_STALE')
    state=motor[0]
    if (state.get('mode')!='HARDWARE_CANDIDATE' or state.get('armed') is not False
            or state.get('measured_rpm')!=[0,0] or state.get('fault')
            or state.get('feedback_fresh') is not True or state.get('io_failed') is not False):
        raise ValueError('REQUIRE_DISARMED_ZERO_FEEDBACK')
    for key in ('leader','follower'):
        row=nav.inputs.get(key)
        if row is None:raise ValueError(key.upper()+'_MISSING')
        stamp=row[0].sec*10**9+row[0].nanosec-(50_000_000 if key=='leader' else 0)
        if not (stamp>0 and 0<=(now-stamp)/1e9<=.3 and 0<=mono-row[1]<=.3):
            raise ValueError(key.upper()+'_STALE')
        if any(not math.isfinite(v) or abs(v)>.01 for v in row[2][3:5]):
            raise ValueError('REQUIRE_BOTH_STATIONARY')


def reference_from_plan(data,path,pose,leader,*,source_kind='live_path'):
    """Reuse only the map line; all obstacle geometry and poses remain live."""
    from apf_case3_candidate import wrap
    snapshot=data['snapshot'];anchor=snapshot['reference_origin'];heading=snapshot['reference_heading']
    if (data.get('reason')!='DETOUR_PLANNED' or len(anchor)!=2 or len(path)<2
            or source_kind not in ('live_path','robot_anchors')
            or not all(math.isfinite(v) for v in (*anchor,heading,pose.x,pose.y,pose.yaw,*leader))
            or snapshot['line_fit_residual_m']>.10
            or not all(math.isfinite(v) for point in path for v in point)):
        raise ValueError('SAVED_REFERENCE_INVALID')
    if source_kind=='robot_anchors':
        expected=((pose.x,pose.y),leader[:2])
        if len(path)!=2 or any(len(p)!=2 or math.hypot(p[0]-q[0],p[1]-q[1])>1e-9 for p,q in zip(path,expected)):
            raise ValueError('SAVED_REFERENCE_ANCHORS_INVALID')
    co,si=math.cos(heading),math.sin(heading)
    def cross(x,y):return -si*(x-anchor[0])+co*(y-anchor[1])
    residual=max(abs(cross(x,y)) for x,y in path)
    offset=cross(pose.x,pose.y)
    if (source_kind=='live_path' and residual>.10) or abs(offset)>.15 or abs(cross(leader[0],leader[1]))>.15:
        raise ValueError('SAVED_REFERENCE_LIVE_ALIGNMENT_MISMATCH')
    if abs(wrap(pose.yaw-heading))>math.radians(20) or abs(wrap(leader[2]-heading))>math.radians(20):
        raise ValueError('SAVED_REFERENCE_HEADING_MISMATCH')
    # Move the origin along the same map line, never laterally onto the robot.
    along=co*(pose.x-anchor[0])+si*(pose.y-anchor[1])
    origin=(anchor[0]+co*along,anchor[1]+si*along)
    return origin,heading,offset,residual


def save_current_plan(output,target):
    from watch_temporary_detour_shadow import PreparedPathReview
    if output.get('reason')!='DETOUR_PLANNED' or not output.get('path'):
        raise ValueError('No checked plan to save')
    review=PreparedPathReview(output['path']).review(output['snapshot']['points'])
    if review['reason']!='LIVE_PATH_GEOMETRY_CLEAR':raise ValueError('STOP_LIVE_PATH_BLOCKED')
    data=dict(output,box_forward_m=0.,live_review=review)
    target=Path(target);previous=None
    if target.exists():
        previous=target.with_name(f'previous_current_detour_{time.time_ns()}.json')
        previous.write_bytes(target.read_bytes())
    temporary=target.with_suffix('.pending')
    temporary.write_text(json.dumps(data,allow_nan=False)+'\n');temporary.replace(target)
    return target


def build_output(result,snapshot,cfg,passage,side,reference_plan):
    output=dict(result)
    output.update(control_connected=False,evaluation_pass=False,settings=vars(cfg),
                  passage=passage,snapshot=snapshot,requested_side=side,
                  reference_plan=str(reference_plan) if reference_plan else None)
    return output


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds',type=int,default=10)
    parser.add_argument('--confirm-obstacle-visible',action='store_true')
    parser.add_argument('--obstacle-center-y',type=float)
    parser.add_argument('--side',choices=['auto','left','right'],default='auto')
    parser.add_argument('--objective',choices=['length','time'],default='length')
    parser.add_argument('--nominal-speed',type=float,default=.04)
    parser.add_argument('--reference-plan',type=Path,help='기존 검증 지도 기준선 사용; 실제 path/양쪽 pose와 정합 확인')
    parser.add_argument('--save-current-plan',action='store_true',help='양쪽 정지·후발 비무장 확인 후 검사된 경로를 현재 시험 경로로 저장')
    args=parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID')!='10':parser.error('ROS_DOMAIN_ID=10 required')
    if not args.confirm_obstacle_visible:parser.error('실제 스캔 높이의 상자 면 확인 필요')
    if not 5<=args.seconds<=30:parser.error('seconds must be 5..30')
    if not 0<args.nominal_speed<=.06:parser.error('nominal-speed must be >0 and <=.06')
    saved_reference=json.loads(args.reference_plan.read_text()) if args.reference_plan else None
    cfg=Settings(lidar_height=.25,visibility_confirmed=True)
    passage=cfg.passage(args.obstacle_center_y) if args.obstacle_center_y is not None else None
    if args.side=='auto':
        if passage is None:parser.error('auto side requires obstacle-center-y')
        if not (passage['left_possible'] or passage['right_possible']):parser.error('현재 여유를 만족하는 평행 통로 없음')
        side=-1 if passage['right_gap_m']>=passage['left_gap_m'] else 1
    else:side=1 if args.side=='left' else -1
    import rclpy
    from rclpy.time import Time
    from rclpy.duration import Duration
    from nav_msgs.msg import Odometry,Path as RosPath
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    from tf2_ros import Buffer,TransformListener,TransformException
    from follower_remote_qos import latest_state_qos
    from follower_pursuit_preview import PreviewCore
    from watch_apf_case3 import project
    rclpy.init(args=[]);node=rclpy.create_node('temporary_detour_readonly_check')
    buffer=Buffer();listener=TransformListener(buffer,node,qos=latest_state_qos())
    nav=PreviewCore();snapshot=None;failure='WAIT_INPUTS';latest_scan=None;motor=None
    def motor_cb(m):
        nonlocal motor
        try:
            state=json.loads(m.data)
            motor=(state,time.monotonic()) if isinstance(state,dict) else None
        except (ValueError,TypeError):motor=None
    def leader(m):nav.odom_callback('leader',m,node.get_clock().now().nanoseconds,time.monotonic())
    def odom(m):nav.odom_callback('follower',m,node.get_clock().now().nanoseconds,time.monotonic())
    def path(m):nav.path_callback(m,node.get_clock().now().nanoseconds,time.monotonic())
    def scan(m):
        nonlocal latest_scan
        latest_scan=(m,time.monotonic())
    subs=[node.create_subscription(Odometry,'/follower/odom_map',odom,latest_state_qos()),
          node.create_subscription(RosPath,'/leader/path',path,latest_state_qos()),
          node.create_subscription(LaserScan,'/follower/scan',scan,latest_state_qos())]
    if args.save_current_plan or saved_reference is not None:
        subs.extend([node.create_subscription(String,'/follower/motor_candidate/status',motor_cb,latest_state_qos()),
                     node.create_subscription(Odometry,'/leader/odom',leader,latest_state_qos())])
    print('조회 전용: 양쪽 정지 유지. 실제 입력으로 임시 경로만 계산·저장. 명령 발행/arm 없음.',flush=True)
    start=time.monotonic()
    try:
        while time.monotonic()-start<args.seconds:
            rclpy.spin_once(node,timeout_sec=.05)
            try:
                now_ns=node.get_clock().now().nanoseconds;mono=time.monotonic()
                if latest_scan is None:raise ValueError('SCAN_MISSING')
                m,rx=latest_scan;s=m.header.stamp.sec*10**9+m.header.stamp.nanosec
                if not (s>0 and 0<=(now_ns-s)/1e9<=.3 and 0<=mono-rx<=.3):raise ValueError('SCAN_STALE')
                for key in ('path','follower'):
                    row=nav.inputs.get(key)
                    if row is None:raise ValueError(key.upper()+'_MISSING')
                    stamp=row[0].sec*10**9+row[0].nanosec-(50_000_000 if key=='path' else 0)
                    if not (stamp>0 and 0<=(now_ns-stamp)/1e9<=.3 and 0<=mono-row[1]<=.3):raise ValueError(key.upper()+'_STALE')
                if args.save_current_plan or saved_reference is not None:
                    stationary_inputs(nav,motor,now_ns,mono)
                tf=buffer.lookup_transform('follower/base_link','follower/laser',Time.from_msg(m.header.stamp),timeout=Duration(seconds=0.))
                body_points=project(m,tf)
                pose=Pose(*nav.inputs['follower'][2][:3])
                reference=(reference_from_plan(saved_reference,nav.inputs['path'][2],pose,nav.inputs['leader'][2])
                           if saved_reference is not None else reference_frame(nav.inputs['path'][2],pose))
                origin,heading,cross,residual=reference
                points=to_reference(body_points,pose,origin,heading)
                snapshot=dict(captured_ros_ns=now_ns,scan_source_ns=s,pose=vars(pose),reference_origin=origin,
                              reference_heading=heading,cross_error_m=cross,line_fit_residual_m=residual,
                              body_points=body_points,points=points,source_stamps={
                                  k:nav.inputs[k][0].sec*10**9+nav.inputs[k][0].nanosec for k in ('path','follower')})
                failure=None
            except (ValueError,TransformException) as exc:failure=str(exc)
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    # ROS 콜백에서 계획 연산을 수행하지 않는다. 신선한 정지 스냅샷만 계획에 사용한다.
    if snapshot is None or failure is not None:
        print(json.dumps(dict(reason='STOP_INPUT_NOT_READY',detail=failure,control_connected=False),ensure_ascii=False));return 1
    if (now_ns-snapshot['captured_ros_ns'])/1e9>.3:
        print('STOP_SNAPSHOT_STALE');return 1
    pose=snapshot['pose'];yaw=pose['yaw']-snapshot['reference_heading']
    result=plan(snapshot['points'],cfg,side=side,initial_cross=snapshot['cross_error_m'],initial_yaw=yaw,
                objective=args.objective,nominal_speed=args.nominal_speed)
    output=build_output(result,snapshot,cfg,passage,args.side,args.reference_plan)
    folder=Path(__file__).parent/'logs';folder.mkdir(exist_ok=True)
    target=folder/f'temporary_detour_check_{time.time_ns()}.json'
    target.write_text(json.dumps(output,allow_nan=False)+'\n')
    if args.save_current_plan and result.get('reason')=='DETOUR_PLANNED':
        saved=save_current_plan(output,folder/'current_scene_left_detour_plan.json')
        print('CURRENT_PLAN='+str(saved),flush=True)
    summary={k:v for k,v in result.items() if k!='path'}
    summary.update(path_points=len(result['path']),control_connected=False,requested_side=args.side,
                   reference_residual_m=snapshot['line_fit_residual_m'])
    print(json.dumps(summary,ensure_ascii=False),flush=True);print('JSON='+str(target),flush=True)
    return 0 if result['path'] else 1


if __name__=='__main__':raise SystemExit(main())
