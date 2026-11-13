#!/usr/bin/env python3
"""정지 상태 실제 scan으로 APF 가상 전진 계산. 모터·서비스·Twist 발행 없음."""
import argparse,json,math,os,time
from pathlib import Path
from apf_case3_candidate import Settings,APF,scan_diagnostic


def project(msg,tf):
    if msg.header.frame_id!='follower/laser' or tf.header.frame_id!='follower/base_link' or tf.child_frame_id!='follower/laser':raise ValueError('scan/TF 프레임 불일치')
    t,q=tf.transform.translation,tf.transform.rotation
    vals=(msg.angle_min,msg.angle_increment,msg.range_min,msg.range_max,t.x,t.y,t.z,q.x,q.y,q.z,q.w)
    if not all(math.isfinite(v) for v in vals) or abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1)>.01 or not 0<msg.range_min<msg.range_max or msg.angle_increment==0 or len(msg.ranges)>50000:
        raise ValueError('scan/TF 값이 유효하지 않음')
    out=[]
    for i,r in enumerate(msg.ranges):
        if not math.isfinite(r) or not msg.range_min<=r<=msg.range_max:continue
        a=msg.angle_min+i*msg.angle_increment;x=r*math.cos(a);y=r*math.sin(a)
        out.append((t.x+(1-2*(q.y*q.y+q.z*q.z))*x+2*(q.x*q.y-q.w*q.z)*y,
                    t.y+2*(q.x*q.y+q.w*q.z)*x+(1-2*(q.x*q.x+q.z*q.z))*y))
    return out


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--seconds',type=int,default=60)
    p.add_argument('--confirm-obstacle-visible',action='store_true',help='시험물 높이를 실제 스캔 평면 위로 맞춘 상태 확인')
    p.add_argument('--obstacle-center-y',type=float,required=True,help='폭2.5m 공간 중심 기준 상자 중심의 왼쪽 오프셋(m)')
    a=p.parse_args()
    if not 10<=a.seconds<=120 or not math.isfinite(a.obstacle_center_y):p.error('진단 시간/상자 위치가 유효하지 않습니다.')
    if os.environ.get('ROS_DOMAIN_ID')!='10':p.error('ROS_DOMAIN_ID=10이 필요합니다.')
    cfg=Settings(lidar_height=.25,visibility_confirmed=a.confirm_obstacle_visible)
    geometry=cfg.passage(a.obstacle_center_y)
    print('진단 전용: 실제 scan + 가상 전진0.04m/s. 주행 명령·arm 요청 없음.',flush=True)
    print(json.dumps(dict(passage=geometry,parallel_geometry_only=True),ensure_ascii=False),flush=True)
    import rclpy
    from rclpy.duration import Duration
    from rclpy.time import Time
    from sensor_msgs.msg import LaserScan
    from rclpy.qos import qos_profile_sensor_data
    from tf2_ros import Buffer,TransformListener,TransformException
    rclpy.init(args=[]);n=rclpy.create_node('apf_case3_observer');buffer=Buffer();listener=TransformListener(buffer,n,qos=qos_profile_sensor_data)
    preferred=1 if geometry['left_gap_m']>=geometry['right_gap_m'] else -1
    core=APF(cfg,preferred_side=preferred);rows=[];last=None;last_at=0.
    def receive(m):
        nonlocal last,last_at
        now=n.get_clock().now().nanoseconds;stamp=m.header.stamp.sec*10**9+m.header.stamp.nanosec
        if stamp<=0 or not 0<=(now-stamp)/1e9<=.3:r=dict(v=0.,w=0.,reason='STOP_SCAN_TIME_INVALID',phase=core.phase)
        elif not (geometry['left_possible'] or geometry['right_possible']):r=dict(v=0.,w=0.,reason='STOP_PARALLEL_PASSAGE_TOO_NARROW',phase=core.phase)
        else:
            try:
                tf=buffer.lookup_transform('follower/base_link','follower/laser',Time.from_msg(m.header.stamp),timeout=Duration(seconds=0.))
                points=project(m,tf)
                r=core.step(.04,0.,points)
                r['scan_diagnostic']=scan_diagnostic(points,cfg)
            except (ValueError,TransformException) as exc:r=dict(v=0.,w=0.,reason='STOP_SCAN_TF_INVALID',detail=str(exc),phase=core.phase)
        row=dict(source_ns=stamp,rx_ros_ns=now,synthetic_forward_probe=True,control_connected=False,**r);rows.append(row)
        key=(r['reason'],r['phase'],r.get('side'))
        if key!=last or time.monotonic()-last_at>=2:
            print(json.dumps(row,ensure_ascii=False),flush=True);last=key;last_at=time.monotonic()
    sub=n.create_subscription(LaserScan,'/follower/scan',receive,qos_profile_sensor_data)
    start=time.monotonic()
    try:
        while rclpy.ok() and time.monotonic()-start<a.seconds:rclpy.spin_once(n,timeout_sec=.05)
    except KeyboardInterrupt:pass
    finally:
        d=Path(__file__).parent/'logs';d.mkdir(exist_ok=True);path=d/f'apf_case3_observation_{time.time_ns()}.json'
        path.write_text(json.dumps(dict(settings=vars(cfg),passage=geometry,rows=rows,evaluation_pass=False,control_connected=False),ensure_ascii=False,allow_nan=False)+'\n')
        print('JSON='+str(path),flush=True);n.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
