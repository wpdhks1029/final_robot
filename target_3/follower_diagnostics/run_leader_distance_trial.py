#!/usr/bin/env python3
"""Explicit leader-only1m forward distance comparison. Default opens nothing."""
import argparse,json,math,os,time
from collections import deque
from pathlib import Path
from follower_yaw_diagnostics import rpy,angle_delta,synchronized_summary
from leader_short_forward_trial import wait_for_start

def scan_evidence(m,tf):
    roll,pitch,yaw=rpy(tf.transform.rotation);t=tf.transform.translation
    points=[]
    for i,r in enumerate(m.ranges):
        if not math.isfinite(r) or not m.range_min<=r<=m.range_max:continue
        angle=m.angle_min+i*m.angle_increment
        x=t.x+r*math.cos(angle+yaw);y=t.y+r*math.sin(angle+yaw)
        points.append(dict(index=i,scan_angle_deg=math.degrees(angle),raw_range_m=r,
            base_x_m=x,base_y_m=y,center_distance_m=math.hypot(x,y)))
    points.sort(key=lambda p:p['center_distance_m'])
    radius=math.hypot(.27,.30)+.02+.30
    return dict(required_center_distance_m=radius,nearest_points=points[:8],
                inside_count=sum(p['center_distance_m']<=radius for p in points),
                mount_translation_m=[t.x,t.y,t.z],mount_yaw_deg=math.degrees(yaw))

def pack_scan(m):
    return dict(stamp_ns=m.header.stamp.sec*10**9+m.header.stamp.nanosec,frame=m.header.frame_id,
                angle_min=m.angle_min,angle_increment=m.angle_increment,range_min=m.range_min,range_max=m.range_max,
                ranges=[float(v) if math.isfinite(v) else None for v in m.ranges])

def main():
    p=argparse.ArgumentParser(description=__doc__);group=p.add_mutually_exclusive_group()
    group.add_argument('--run',action='store_true');group.add_argument('--plan-only',action='store_true')
    p.add_argument('--ack-clear-path-power-cut-ready',action='store_true')
    p.add_argument('--distance-m',type=float,choices=(.2,1.),default=1.)
    p.add_argument('--speed',type=float,choices=(.04,),default=.04)
    p.add_argument('--operator-clear-path',action='store_true',help='Leader distance test only: operator verifies clear straight path; near returns recorded without aborting')
    a=p.parse_args()
    if not a.run and not a.plan_only:p.print_help();return 0
    if os.environ.get('ROS_DOMAIN_ID')!='10' or a.run and not a.ack_clear_path_power_cut_ready:p.error('Domain10 and explicit run acknowledgement required')
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import QoSProfile,ReliabilityPolicy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu,LaserScan
    from tf2_ros import Buffer,TransformListener
    rclpy.init(args=[],signal_handler_options=SignalHandlerOptions.NO)
    node=rclpy.create_node('leader_distance_trial_operator');buffer=Buffer();listener=TransformListener(buffer,node)
    qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
    streams={};latest={};scan=None;rejects={};started=False;zero=False;reason='NOT_STARTED';begin=None
    scan_ring=deque(maxlen=12);failure_scans=[];clearance_diagnostic=None
    pub=node.create_publisher(Twist,'/cmd_vel',1) if a.run else None
    def now():return node.get_clock().now().nanoseconds
    def spin():rclpy.spin_once(node,timeout_sec=.02)
    def receive(key,m):
        try:
            if key!='imu' and (m.child_frame_id!='base_link' or m.header.frame_id!=('map' if key=='map' else 'odom')):raise ValueError('FRAME_MISMATCH')
            if key=='imu' and (not m.header.frame_id or m.orientation_covariance[0]<0):raise ValueError('IMU_ORIENTATION_MISSING')
            roll,pitch,yaw=rpy(m.orientation if key=='imu' else m.pose.pose.orientation)
            stamp=m.header.stamp.sec*10**9+m.header.stamp.nanosec
            rows=streams.setdefault(key,[])
            if stamp<=0 or rows and stamp<=rows[-1]['stamp_ns']:raise ValueError('STAMP_ORDER')
            row=dict(stamp_ns=stamp,rx_mono=time.monotonic(),roll=roll,pitch=pitch,yaw=yaw)
            if key!='imu':
                t=m.twist.twist;v=m.pose.pose.position
                row.update(x=v.x,y=v.y,v=t.linear.x,w=t.angular.z)
                if not all(math.isfinite(row[k]) for k in ('x','y','v','w')):raise ValueError('NONFINITE_ODOM')
            rows.append(row);latest[key]=row
        except (ValueError,TypeError) as exc:rejects[key]=str(exc);latest.pop(key,None)
    def scan_cb(m):
        nonlocal scan
        scan=(m,time.monotonic())
        scan_ring.append(pack_scan(m))
    subs=[node.create_subscription(Odometry,topic,lambda m,k=key:receive(k,m),qos) for key,topic in [('raw','/odom'),('filtered','/odometry/filtered'),('map','/leader/odom')]]
    subs.extend([node.create_subscription(Imu,'/imu/data',lambda m:receive('imu',m),qos),node.create_subscription(LaserScan,'/scan',scan_cb,qos)])
    def graph():
        writers=node.get_publishers_info_by_topic('/cmd_vel');readers=node.get_subscriptions_info_by_topic('/cmd_vel')
        if a.run and (len(writers)!=1 or writers[0].node_name!=node.get_name()):raise ValueError('OTHER_CMD_VEL_PUBLISHER')
        if a.plan_only and writers:raise ValueError('OTHER_CMD_VEL_PUBLISHER')
        if len(readers)!=1 or readers[0].node_name!='md200t_controller':raise ValueError('CMD_SUBSCRIBER_MISMATCH')
        for topic in ('/odom','/odometry/filtered','/imu/data','/scan'):
            if len(node.get_publishers_info_by_topic(topic))!=1:raise ValueError('SENSOR_OWNER_MISMATCH:'+topic)
    def scene(stationary=False):
        nonlocal clearance_diagnostic,failure_scans
        graph();ns=now();mono=time.monotonic()
        for k in ('raw','filtered','imu'):
            r=latest.get(k)
            if r is None or not 0<=(ns-r['stamp_ns'])/1e9<=.3 or not 0<=mono-r['rx_mono']<=.3:raise ValueError(k.upper()+'_MISSING_OR_STALE')
        if max(abs(latest['imu'][k]) for k in ('roll','pitch'))>math.radians(20):raise ValueError('IMU_TILT')
        if stationary and (abs(latest['raw']['v'])>.01 or abs(latest['raw']['w'])>.01):raise ValueError('NOT_STATIONARY')
        # In this explicit operator-clear distance test, scans are recorded
        # only. Neither a near return, scan delay nor missing scan TF controls
        # motion. Odometry/IMU freshness, sole command ownership and all
        # distance/time/drift limits remain mandatory.
        if a.operator_clear_path:return None
        if scan is None:raise ValueError('SCAN_MISSING')
        m,rx=scan;stamp=m.header.stamp.sec*10**9+m.header.stamp.nanosec
        if not 0<=(ns-stamp)/1e9<=.3 or not 0<=mono-rx<=.3:raise ValueError('SCAN_STALE')
        if not 180<=len(m.ranges)<=50000 or m.angle_max-m.angle_min<math.radians(330) or not 0<m.range_min<m.range_max or m.angle_increment<=0:raise ValueError('SCAN_INVALID')
        tf=buffer.lookup_transform('base_link',m.header.frame_id,rclpy.time.Time.from_msg(m.header.stamp))
        roll,pitch,yaw=rpy(tf.transform.rotation)
        if max(abs(roll),abs(pitch))>math.radians(2):raise ValueError('SCAN_MOUNT_NOT_PLANAR')
        points=[(tf.transform.translation.x+r*math.cos(m.angle_min+i*m.angle_increment+yaw),tf.transform.translation.y+r*math.sin(m.angle_min+i*m.angle_increment+yaw)) for i,r in enumerate(m.ranges) if math.isfinite(r) and m.range_min<=r<=m.range_max]
        if len(points)<100:raise ValueError('SCAN_POINTS_INSUFFICIENT')
        clearance=min(math.hypot(x,y) for x,y in points)-math.hypot(.27,.30)-.02
        if clearance<=.3:
            clearance_diagnostic=dict(scan_evidence(m,tf),scan_stamp_ns=stamp)
            failure_scans=list(scan_ring)
            if not a.operator_clear_path:raise ValueError('ROTATION_CIRCLE_NOT_CLEAR')
        return clearance
    def send(v=0.):
        m=Twist();m.linear.x=v;pub.publish(m)
    limit=15. if a.distance_m==.2 else 40.
    print(f'선두만 오뎀 기준{a.distance_m:g}m 직진. 후발 비무장 유지. 자동 거리 보정 없음.',flush=True)
    try:
        until=time.monotonic()+12;problem='WAIT_INPUTS'
        while time.monotonic()<until:
            spin()
            try:clearance=scene(True);break
            except Exception as exc:problem=str(exc)
        else:raise ValueError('PREFLIGHT_'+problem)
        if a.plan_only:reason='PLAN_ONLY_READY';return 0
        print(f'START + Enter: 선두만 최대0.04m/s, 오뎀 기준{a.distance_m:g}m 직진. 최대{limit:g}초.',flush=True)
        if not wait_for_start(spin):reason='CANCELLED';return 1
        scene(True);base={k:latest[k].copy() for k in ('raw','filtered','imu')};begin=now();start=time.monotonic();started=True
        while True:
            spin();clearance=scene();raw=angle_delta(latest['raw']['yaw'],base['raw']['yaw']);imu=angle_delta(latest['imu']['yaw'],base['imu']['yaw'])
            dx=latest['raw']['x']-base['raw']['x'];dy=latest['raw']['y']-base['raw']['y']
            co,si=math.cos(base['raw']['yaw']),math.sin(base['raw']['yaw'])
            along=co*dx+si*dy;lateral=-si*dx+co*dy
            if max(abs(raw),abs(imu))>math.radians(15):reason='HEADING_DRIFT_OVER_15_DEG';break
            if abs(lateral)>.15:reason='LATERAL_DRIFT_OVER_15CM';break
            if along>=a.distance_m:reason='DISTANCE_TARGET_REACHED';break
            if along<-.03:reason='OPPOSITE_TRANSLATION';break
            if time.monotonic()-start>=limit:reason='DISTANCE_TIME_LIMIT';break
            send(.02 if a.distance_m-along<.1 else a.speed)
    except KeyboardInterrupt:reason='INTERRUPTED'
    except Exception as exc:reason=str(exc)
    finally:
        if started:
            deadline=time.monotonic()+3.;last=None;count=0
            while time.monotonic()<deadline:
                try:
                    send();spin();r=latest.get('raw')
                    if r and r['stamp_ns']!=last:
                        last=r['stamp_ns'];count=count+1 if 0<=(now()-last)/1e9<=.3 and abs(r['v'])<.01 and abs(r['w'])<.01 else 0
                    if count>=5:zero=True;break
                except Exception:break
        summary=dict(synchronized_summary(streams,begin_ns=begin),trial_reason=reason,target_distance_m=a.distance_m,
                     leader_commanded=started,follower_commanded=False,leader_zero_confirmed=zero,
                     trial_success=reason=='DISTANCE_TARGET_REACHED' and zero,plan_only=a.plan_only,evaluation_pass=False)
        folder=Path(__file__).resolve().parent/'logs';folder.mkdir(exist_ok=True);out=folder/f'leader_distance_trial_{time.time_ns()}.json'
        summary['distance_sources']={}
        if started:
            for key,b in base.items():
                e=latest.get(key)
                if e is None or key=='imu':continue
                dx=e['x']-b['x'];dy=e['y']-b['y'];co,si=math.cos(b['yaw']),math.sin(b['yaw'])
                summary['distance_sources'][key]=dict(forward_m=co*dx+si*dy,lateral_m=-si*dx+co*dy,displacement_m=math.hypot(dx,dy))
        summary['clearance_diagnostic']=clearance_diagnostic
        summary['operator_clear_path']=a.operator_clear_path
        summary['scan_used_as_motion_gate']=not a.operator_clear_path
        out.write_text(json.dumps(dict(summary=summary,streams=streams,rejections=rejects,
            scans_at_failure=failure_scans,scans_after_stop=list(scan_ring) if started else []),allow_nan=False)+'\n')
        print(json.dumps(summary,ensure_ascii=False,indent=2));print('JSON='+str(out))
        if started and not zero:print('선두 정지 피드백 미확인. 실물이 멈추지 않으면 전원을 차단하세요.')
        node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
