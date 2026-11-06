#!/usr/bin/env python3
"""선두/후발 초기 위치 숫자 미세 조정. 기본은 미리보기이며 --apply만 발행한다."""
import argparse
from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import time

TARGETS={
    'leader':('base_link','/initialpose','/amcl/get_state','/'),
    'follower':('follower/base_link','/follower/initialpose','/follower/amcl/get_state','/follower'),
}


@dataclass(frozen=True)
class Pose:
    x:float
    y:float
    yaw:float

    def report(self):return dict(x_m=self.x,y_m=self.y,yaw_deg=math.degrees(self.yaw))


def plan(current,*,x=None,y=None,yaw_deg=None,set_yaw_deg=None,dx_cm=0.,dy_cm=0.,dyaw_deg=0.,offset_frame='base'):
    values=(x,y,yaw_deg);absolute=any(v is not None for v in values)
    numbers=[v for v in (*values,set_yaw_deg,dx_cm,dy_cm,dyaw_deg) if v is not None]
    if not all(math.isfinite(v) for v in numbers):raise ValueError('유한한 숫자만 입력하세요.')
    if offset_frame not in ('base','map'):raise ValueError('이동 기준은base 또는map이어야 합니다.')
    if set_yaw_deg is not None:
        if absolute or any(v!=0 for v in (dx_cm,dy_cm,dyaw_deg)):
            raise ValueError('--set-yaw-deg는 다른 위치·각도 변경 옵션과 혼용하지 마세요.')
        if current is None:raise ValueError('각도만 지정하려면 현재 map TF가 필요합니다.')
        yaw=math.radians(set_yaw_deg)
        return Pose(current.x,current.y,math.atan2(math.sin(yaw),math.cos(yaw)))
    if absolute:
        if not all(v is not None for v in values):raise ValueError('--x --y --yaw-deg를 함께 입력하세요.')
        if any(v!=0 for v in (dx_cm,dy_cm,dyaw_deg)):raise ValueError('절대 좌표와 증분 조정을 함께 사용하지 마세요.')
        if abs(x)>100 or abs(y)>100:raise ValueError('지도 좌표 범위를 확인하세요.')
        yaw=math.radians(yaw_deg)
        return Pose(x,y,math.atan2(math.sin(yaw),math.cos(yaw)))
    if current is None:raise ValueError('증분 조정에는 현재 map TF가 필요합니다.')
    if math.hypot(dx_cm,dy_cm)>20 or abs(dyaw_deg)>10:
        raise ValueError('한 번의 미세 조정은 이동20cm/회전10도 이내로 제한합니다.')
    dx,dy=dx_cm/100,dy_cm/100
    if offset_frame=='base':
        dx,dy=math.cos(current.yaw)*dx-math.sin(current.yaw)*dy,math.sin(current.yaw)*dx+math.cos(current.yaw)*dy
    yaw=current.yaw+math.radians(dyaw_deg)
    return Pose(current.x+dx,current.y+dy,math.atan2(math.sin(yaw),math.cos(yaw)))


def stopped(sample,now_ns,mono,*,leader):
    if sample is None:return False
    m,rx=sample;stamp=m.header.stamp.sec*10**9+m.header.stamp.nanosec
    expected=('odom','base_link') if leader else ('follower/odom','follower/base_link')
    if (m.header.frame_id,m.child_frame_id)!=expected:return False
    if not (stamp>0 and (-.05 if leader else 0)<=(now_ns-stamp)/1e9<=.3 and 0<=mono-rx<=.3):return False
    t=m.twist.twist;values=(t.linear.x,t.linear.y,t.linear.z,t.angular.x,t.angular.y,t.angular.z)
    return all(math.isfinite(v) and abs(v)<=.01 for v in values)


def apply_ready(motor,leader,follower,now_ns,mono):
    if motor is None:return False
    s,rx=motor
    return (0<=mono-rx<=.3 and s.get('armed') is False and s.get('fault') is None
            and s.get('io_failed') is False and s.get('feedback_fresh') is True
            and s.get('measured_rpm')==[0,0]
            and stopped(leader,now_ns,mono,leader=True) and stopped(follower,now_ns,mono,leader=False))


def fill_pose(msg,pose,stamp,xy_std,yaw_std_deg):
    if not (math.isfinite(xy_std) and .01<=xy_std<=5
            and math.isfinite(yaw_std_deg) and .1<=yaw_std_deg<=180):
        raise ValueError('표준편차 범위를 확인하세요.')
    msg.header.frame_id='map';msg.header.stamp=stamp
    p=msg.pose.pose;p.position.x=pose.x;p.position.y=pose.y;p.position.z=0.
    p.orientation.x=p.orientation.y=0.
    p.orientation.z=math.sin(pose.yaw/2);p.orientation.w=math.cos(pose.yaw/2)
    cov=[0.]*36;cov[0]=cov[7]=xy_std**2;cov[35]=math.radians(yaw_std_deg)**2
    msg.pose.covariance=cov
    return msg


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--robot',choices=TARGETS,default='follower')
    p.add_argument('--x',type=float);p.add_argument('--y',type=float);p.add_argument('--yaw-deg',type=float)
    p.add_argument('--set-yaw-deg',type=float,help='현재 위치를 유지하고 지도 기준 각도만 지정합니다.')
    p.add_argument('--dx-cm',type=float,default=0.);p.add_argument('--dy-cm',type=float,default=0.)
    p.add_argument('--dyaw-deg',type=float,default=0.)
    p.add_argument('--offset-frame',choices=['base','map'],default='base')
    p.add_argument('--xy-std-m',type=float,default=.5)
    p.add_argument('--yaw-std-deg',type=float,default=15.)
    p.add_argument('--apply',action='store_true');p.add_argument('--ack-stationary',action='store_true')
    a=p.parse_args()
    params=dict(x=a.x,y=a.y,yaw_deg=a.yaw_deg,set_yaw_deg=a.set_yaw_deg,dx_cm=a.dx_cm,dy_cm=a.dy_cm,
                dyaw_deg=a.dyaw_deg,offset_frame=a.offset_frame)
    try:plan(Pose(0.,0.,0.),**params)
    except ValueError as exc:p.error(str(exc))
    if not (.01<=a.xy_std_m<=5 and .1<=a.yaw_std_deg<=180):p.error('표준편차 범위를 확인하세요.')
    if a.apply and not a.ack_stationary:p.error('--apply에는 --ack-stationary가 필요합니다.')
    if a.apply and a.x is None and a.set_yaw_deg is None and not any((a.dx_cm,a.dy_cm,a.dyaw_deg)):
        p.error('실제 반영할 변경값을 입력하세요.')
    if os.environ.get('ROS_DOMAIN_ID')!='10':p.error('ROS_DOMAIN_ID=10이 필요합니다.')
    import rclpy
    from rclpy.time import Time
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import QoSProfile,ReliabilityPolicy
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from lifecycle_msgs.srv import GetState
    from tf2_ros import Buffer,TransformListener,TransformException
    from follower_remote_qos import latest_state_qos
    rclpy.init(args=[],signal_handler_options=SignalHandlerOptions.NO)
    node=rclpy.create_node('initial_pose_fine_adjust');buffer=Buffer()
    listener=TransformListener(buffer,node,qos=QoSProfile(depth=100,reliability=ReliabilityPolicy.BEST_EFFORT))
    base,topic,service,namespace=TARGETS[a.robot]
    samples={};motor=None;pub=None;future=None;stable_since=None;current=None
    client=node.create_client(GetState,service) if a.apply else None
    if a.apply:pub=node.create_publisher(PoseWithCovarianceStamped,topic,1)
    def odom(name,m):samples[name]=(m,time.monotonic())
    def status(m):
        nonlocal motor
        try:motor=(json.loads(m.data),time.monotonic())
        except ValueError:motor=None
    subs=[node.create_subscription(Odometry,'/odometry/filtered',lambda m:odom('leader',m),latest_state_qos()),
          node.create_subscription(Odometry,'/follower/odometry/filtered',lambda m:odom('follower',m),latest_state_qos()),
          node.create_subscription(String,'/follower/motor_candidate/status',status,1)]
    absolute=a.x is not None
    try:
        until=time.monotonic()+15
        print('현재 pose 조회 중. --apply가 없으면 초기 위치를 발행하지 않습니다.',flush=True)
        while time.monotonic()<until:
            rclpy.spin_once(node,timeout_sec=.03)
            now=node.get_clock().now().nanoseconds;mono=time.monotonic();current=None
            try:
                t=buffer.lookup_transform('map',base,Time())
                ns=t.header.stamp.sec*10**9+t.header.stamp.nanosec
                q=t.transform.rotation;v=t.transform.translation
                if (ns>0 and (-.05 if a.robot=='leader' else 0)<=(now-ns)/1e9<=.3
                        and all(math.isfinite(n) for n in (v.x,v.y,q.x,q.y,q.z,q.w))
                        and abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1)<.01):
                    current=Pose(v.x,v.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
            except TransformException:pass
            if current is None and not absolute:continue
            if not a.apply:break
            if future is None and client.service_is_ready():future=client.call_async(GetState.Request())
            if future is not None and future.done() and future.result().current_state.id!=3:
                raise RuntimeError('대상 AMCL이 active 상태가 아닙니다.')
            ready=apply_ready(motor,samples.get('leader'),samples.get('follower'),now,mono)
            stable_since=mono if ready and stable_since is None else stable_since if ready else None
            matched=any(e.node_name=='amcl' and e.node_namespace==namespace
                        for e in node.get_subscriptions_info_by_topic(topic))
            if (stable_since is not None and mono-stable_since>=.5 and matched
                    and pub.get_subscription_count()>0 and future is not None and future.done()):break
        else:raise RuntimeError('최신 map TF 또는 양쪽 정지/미무장/AMCL 연결을 확인하지 못했습니다. 발행하지 않았습니다.')
        proposed=plan(current,**params)
        report=dict(robot=a.robot,frame='map',topic=topic,
                    current=None if current is None else current.report(),proposed=proposed.report(),
                    offset_frame=a.offset_frame,xy_std_m=a.xy_std_m,yaw_std_deg=a.yaw_std_deg,
                    mode='PUBLISH_ONCE' if a.apply else 'PREVIEW_ONLY')
        print(json.dumps(report,ensure_ascii=False,indent=2),flush=True)
        if a.apply:
            msg=fill_pose(PoseWithCovarianceStamped(),proposed,node.get_clock().now().to_msg(),a.xy_std_m,a.yaw_std_deg)
            pub.publish(msg)
            until=time.monotonic()+1
            while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.03)
            directory=Path(__file__).parent/'logs';directory.mkdir(exist_ok=True)
            path=directory/f'pose_adjust_{time.time_ns()}.json';path.write_text(json.dumps(report,indent=2)+'\n')
            print('초기 위치 메시지1회 발행 완료. RViz에서 스캔과 지도 정합을 확인하세요.',flush=True)
            if a.robot=='leader':print('선두 조정을 모두 마친 뒤 선두 bridge를 한 번 재시작해 경로 이력을 초기화하세요.')
        else:print('미리보기만 완료. 실제 pose는 변경하지 않았습니다.')
        return 0
    except (RuntimeError,KeyboardInterrupt) as exc:
        print(str(exc) or '사용자 중단',flush=True);return 1
    except Exception as exc:
        print('초기 위치 도구 중단: '+str(exc),flush=True);return 1
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':raise SystemExit(main())
