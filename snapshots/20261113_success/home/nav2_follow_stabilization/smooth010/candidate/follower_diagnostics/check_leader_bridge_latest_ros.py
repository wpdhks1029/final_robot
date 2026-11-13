#!/usr/bin/env python3
"""Domain217 localhost의 가짜 TF/odom만 사용. 실제 로봇·모터·서비스 요청 없음."""
import argparse,os,time,subprocess,sys,json
from pathlib import Path

def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--run',action='store_true');a=ap.parse_args()
    if not a.run:ap.print_help();return
    os.environ['ROS_DOMAIN_ID']='217';os.environ['ROS_LOCALHOST_ONLY']='1'
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry,Path as RosPath
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import TransformBroadcaster
    rclpy.init(args=[]);n=rclpy.create_node('leader_bridge_offline_check');child=None;out=[];paths=[];failures=[]
    directory=Path(__file__).parent/'logs';directory.mkdir(exist_ok=True)
    log=directory/f'leader_bridge_ros_{time.time_ns()}.log';stream=log.open('x')
    try:
        end=time.monotonic()+2
        while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
        if any(name!=n.get_name() for name,ns in n.get_node_names_and_namespaces()):raise RuntimeError('Domain217 사용 중')
        raw=n.create_publisher(Odometry,'/odometry/filtered',10);tfpub=TransformBroadcaster(n)
        subs=[n.create_subscription(Odometry,'/leader/odom',lambda m:out.append((time.monotonic(),m)),qos_profile_sensor_data),
              n.create_subscription(RosPath,'/leader/path',lambda m:paths.append((time.monotonic(),m)),qos_profile_sensor_data)]
        child=subprocess.Popen([sys.executable,'-B',str(Path(__file__).with_name('leader_bridge_latest_candidate.py'))],stdout=stream,stderr=subprocess.STDOUT,env=os.environ.copy())
        end=time.monotonic()+3
        while time.monotonic()<end:
            if child.poll() is not None:raise RuntimeError('후보 노드 종료')
            rclpy.spin_once(n,timeout_sec=.02)
        print('ISOLATED Domain217. 가짜 TF/odom만 사용, 실제 모터 연결 없음.',flush=True)
        for name,tf_age,odom_age,expect in [('missing_tf',None,.02,False),('older_odom_stamp',.02,.15,'odom'),
                ('stale_odom',.02,1.,False),('odom_loss',.02,None,False),
                ('recover',.02,.15,'odom'),('tf_loss',None,.02,False),('older_tf_stamp',.15,.02,'tf')]:
            out.clear();paths.clear();stamps={'tf':set(),'odom':set()};start=time.monotonic();next_send=start
            while time.monotonic()-start<1.5:
                if child.poll() is not None:raise RuntimeError('후보 노드 종료')
                now=time.monotonic()
                if now>=next_send:
                    next_send=now+.05;ros=n.get_clock().now().nanoseconds
                    if odom_age is not None:
                        m=Odometry();ts=ros-round(odom_age*1e9);stamps['odom'].add(ts)
                        m.header.stamp.sec=ts//10**9;m.header.stamp.nanosec=ts%10**9
                        m.header.frame_id='odom';m.child_frame_id='base_link';m.pose.pose.orientation.w=1.
                        m.twist.twist.linear.x=.04;raw.publish(m)
                    if tf_age is not None:
                        m=TransformStamped();ts=ros-round(tf_age*1e9);stamps['tf'].add(ts)
                        m.header.stamp.sec=ts//10**9;m.header.stamp.nanosec=ts%10**9
                        m.header.frame_id='map';m.child_frame_id='base_link';m.transform.translation.x=1.;m.transform.rotation.w=1.
                        tfpub.sendTransform(m)
                rclpy.spin_once(n,timeout_sec=.01)
            late=[m for t,m in out if t-start>1.]
            late_path=[m for t,m in paths if t-start>1.]
            if expect:
                valid=lambda m:m.header.stamp.sec*10**9+m.header.stamp.nanosec in stamps[expect]
                good=len(late)>=2 and len(late_path)>=2 and all(valid(m) and m.header.frame_id=='map' and m.child_frame_id=='base_link' and m.twist.twist.linear.x==.04 for m in late) and all(valid(m) and len(m.poses)==1 for m in late_path)
            else:good=not late and not late_path
            print(('PASS ' if good else 'FAIL ')+name,flush=True)
            if not good:failures.append(name)
        print(f'RESULT: {7-len(failures)}/7 passed. Failures={failures}',flush=True)
        if failures:raise RuntimeError('비교 검사 실패')
    finally:
        if child and child.poll() is None:
            child.terminate()
            try:child.wait(timeout=3)
            except subprocess.TimeoutExpired:child.kill();child.wait()
        stream.close();print('LOG='+str(log),flush=True);n.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
