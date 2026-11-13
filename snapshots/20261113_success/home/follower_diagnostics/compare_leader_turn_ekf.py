#!/usr/bin/env python3
"""저장된 선두 rosbag 입력을 Domain217에서 두 EKF로 비교. 모터·실제 TF 접근 없음."""
import argparse
import copy
import json
import math
import os
from pathlib import Path
import sqlite3
import subprocess
import time
import yaml

ROOT=Path(__file__).resolve().parent

def parameters(wheel=False):
    source=yaml.safe_load((ROOT/'leader_ekf_current.yaml').read_text())['/ekf_filter_node']['ros__parameters']
    # dump의 null 미설정 항목과 진단/QoS 내부 항목은 재설정하지 않는다.
    p={k:copy.deepcopy(v) for k,v in source.items() if v is not None and k not in ('qos_overrides','diagnostic_updater')}
    if wheel:
        p={k:v for k,v in p.items() if not k.startswith('imu0')}
        p['odom0_config'][11]=True
    return p

def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def angle(a,b):return math.degrees(math.atan2(math.sin(b-a),math.cos(b-a)))

def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('database',type=Path);ap.add_argument('--run',action='store_true');ap.add_argument('--wheel-yaw-variance',type=float,default=None);ap.add_argument('--wheel-yaw-multiplier',type=float,default=1.);ap.add_argument('--reference-wheel',action='store_true');a=ap.parse_args()
    if not math.isfinite(a.wheel_yaw_multiplier) or not .25<=a.wheel_yaw_multiplier<=4:ap.error('회전 환산 배율 범위 오류')
    if a.wheel_yaw_variance is not None and (not math.isfinite(a.wheel_yaw_variance) or a.wheel_yaw_variance<=0):ap.error('각속도 분산은 유한 양수여야 합니다')
    if not a.run:ap.print_help();return
    os.environ['ROS_DOMAIN_ID']='217';os.environ['ROS_LOCALHOST_ONLY']='1'
    import rclpy
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu
    from geometry_msgs.msg import Twist,TransformStamped
    from tf2_ros import StaticTransformBroadcaster
    from rclpy.qos import qos_profile_sensor_data
    db=sqlite3.connect('file:'+str(a.database.resolve())+'?mode=ro',uri=True)
    if db.execute('pragma quick_check(1)').fetchone()!=('ok',):raise RuntimeError('데이터베이스 검사 실패')
    topics={n:i for i,n in db.execute('select id,name from topics')}
    active=[]
    for t,b in db.execute('select timestamp,data from messages where topic_id=? order by timestamp',(topics['/cmd_vel'],)):
        m=deserialize_message(b,Twist)
        if abs(m.linear.x)+abs(m.angular.z)>1e-6:active.append(t)
    groups=[[]]
    for t in active:
        if groups[-1] and t-groups[-1][-1]>10**9:groups.append([])
        groups[-1].append(t)
    if not active:raise RuntimeError('이동 명령 없음')
    first,last=groups[-1][0],groups[-1][-1];begin=first-2*10**9;finish=last+4*10**9
    rows=[]
    for topic,kind in [('/odom',Odometry),('/imu/data',Imu)]:
        for t,b in db.execute('select timestamp,data from messages where topic_id=? and timestamp between ? and ? order by timestamp',(topics[topic],begin,finish)):
            rows.append((t,topic,deserialize_message(b,kind)))
    rows.sort(key=lambda r:r[0]);db.close()
    rclpy.init(args=[]);node=rclpy.create_node('leader_offline_turn_comparison');children=[];handles=[];outputs={'current':[],'wheel':[]}
    directory=ROOT/'logs'/f'leader_ekf_comparison_{time.time_ns()}';directory.mkdir()
    try:
        until=time.monotonic()+2
        while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.05)
        if any(n!=node.get_name() for n,ns in node.get_node_names_and_namespaces()):raise RuntimeError('Domain217이 사용 중입니다')
        raw=node.create_publisher(Odometry,'/replay/odom',100);imu=node.create_publisher(Imu,'/replay/imu',100)
        wheel_raw=node.create_publisher(Odometry,'/replay/wheel_odom',100)
        static=StaticTransformBroadcaster(node);tf=TransformStamped();tf.header.frame_id='base_link';tf.child_frame_id='imu_link';tf.transform.translation.z=.15;tf.transform.rotation.w=1.;tf.header.stamp=node.get_clock().now().to_msg();static.sendTransform(tf)
        base=[None];subs=[]
        for label in outputs:
            def receive(m,label=label):
                if base[0] is not None:outputs[label].append(dict(t=(m.header.stamp.sec*10**9+m.header.stamp.nanosec-base[0])/1e9,yaw=yaw(m.pose.pose.orientation),x=m.pose.pose.position.x,y=m.pose.pose.position.y))
            subs.append(node.create_subscription(Odometry,'/replay/'+label,receive,qos_profile_sensor_data))
            p=parameters(label=='wheel' or a.reference_wheel);p.update(odom0='/replay/odom',publish_tf=False)
            if label=='current' and not a.reference_wheel:p['imu0']='/replay/imu'
            if label=='wheel':p['odom0']='/replay/wheel_odom'
            config=directory/(label+'.yaml');config.write_text(yaml.safe_dump({'/**':{'ros__parameters':p}}))
            h=(directory/(label+'.log')).open('w');handles.append(h)
            children.append(subprocess.Popen(['/opt/ros/humble/lib/robot_localization/ekf_node','--ros-args','-r','__node:=leader_'+label+'_offline','--params-file',str(config),'-r','odometry/filtered:=/replay/'+label],stdout=h,stderr=subprocess.STDOUT,env=os.environ.copy()))
        until=time.monotonic()+3
        while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.02)
        if any(c.poll() is not None for c in children):raise RuntimeError('EKF 시작 실패: '+str(directory))
        print('격리 Domain217: 원본 메시지·공분산 유지, 시각 공통 이동. 신선한 EKF 비교이며 실물 검증 아님.',flush=True)
        start=time.monotonic();base[0]=node.get_clock().now().nanoseconds;offset=base[0]-begin
        for t,topic,m in rows:
            due=start+(t-begin)/1e9
            while time.monotonic()<due:
                if any(c.poll() is not None for c in children):raise RuntimeError('EKF 종료')
                rclpy.spin_once(node,timeout_sec=min(.01,max(0,due-time.monotonic())))
            stamp=m.header.stamp.sec*10**9+m.header.stamp.nanosec+offset
            m.header.stamp.sec=stamp//10**9;m.header.stamp.nanosec=stamp%10**9
            (raw if topic=='/odom' else imu).publish(m)
            if topic=='/odom':
                wheel_m=copy.deepcopy(m)
                wheel_m.twist.twist.angular.z *= a.wheel_yaw_multiplier
                # 변경한 축의 공분산 행·열도 같은 선형 환산을 적용한다.
                for i in range(6):
                    wheel_m.twist.covariance[5*6+i] *= a.wheel_yaw_multiplier
                    wheel_m.twist.covariance[i*6+5] *= a.wheel_yaw_multiplier
                if a.wheel_yaw_variance is not None:wheel_m.twist.covariance[35]=max(wheel_m.twist.covariance[35],a.wheel_yaw_variance)
                wheel_raw.publish(wheel_m)
            rclpy.spin_once(node,timeout_sec=0.)
        until=time.monotonic()+.3
        while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.02)
        result={}
        for label,rr in outputs.items():
            before=[r for r in rr if r['t']<2.];after=[r for r in rr if (last-begin)/1e9+2<r['t']<(last-begin)/1e9+4]
            if not before or not after:raise RuntimeError('비교 구간 출력 부족: '+label)
            result[label]=dict(samples=len(rr),yaw_delta_deg=angle(before[-1]['yaw'],after[-1]['yaw']),delta_x=after[-1]['x']-before[-1]['x'],delta_y=after[-1]['y']-before[-1]['y'])
        (directory/'result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2),flush=True)
    finally:
        for c in children:
            if c.poll() is None:
                c.terminate()
                try:c.wait(timeout=3)
                except subprocess.TimeoutExpired:c.kill();c.wait()
        for h in handles:h.close()
        (directory/'outputs.json').write_text(json.dumps(outputs)+'\n');print('RESULT_DIR='+str(directory),flush=True)
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
