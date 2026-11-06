#!/usr/bin/env python3
"""Read-only follower wheel/IMU/map heading observer. No motor publishers."""
import argparse,json,math,os,time
from pathlib import Path
from follower_yaw_diagnostics import rpy,synchronized_summary


def attach(node):
    from types import SimpleNamespace
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu,LaserScan
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage
    from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
    from follower_remote_qos import latest_state_qos
    watch=SimpleNamespace(streams={},latest={},motor=None,scan=None,mount=None,rejections={},subscriptions=[],events=[])
    def pose_cb(key,msg):
        try:
            expected={'raw':'follower/odom','filtered':'follower/odom','map':'map','imu':'follower/imu_link'}[key]
            if msg.header.frame_id!=expected:raise ValueError('FRAME_MISMATCH')
            if key!='imu' and msg.child_frame_id!='follower/base_link':raise ValueError('CHILD_FRAME_MISMATCH')
            if key=='imu' and msg.orientation_covariance[0]<0:raise ValueError('NO_IMU_ORIENTATION')
            roll,pitch,yaw=rpy(msg.orientation if key=='imu' else msg.pose.pose.orientation)
            stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
            rows=watch.streams.setdefault(key,[])
            if stamp<=0 or (rows and stamp<=rows[-1]['stamp_ns']):raise ValueError('STAMP_ORDER')
            data=dict(stamp_ns=stamp,rx_mono=time.monotonic(),roll=roll,pitch=pitch,yaw=yaw)
            if key=='imu' and hasattr(msg,'angular_velocity'):
                gyro=float(msg.angular_velocity.z)
                if not math.isfinite(gyro):raise ValueError('INVALID_GYRO')
                data['gyro_z_rad_s']=gyro
                covariance=getattr(msg,'angular_velocity_covariance',[])
                if len(covariance)==9:data['gyro_z_variance']=float(covariance[8])
            if key!='imu':
                data.update(x=msg.pose.pose.position.x,y=msg.pose.pose.position.y,
                            v=msg.twist.twist.linear.x,w=msg.twist.twist.angular.z)
            rows.append(data);watch.latest[key]=data
        except (ValueError,TypeError,IndexError) as exc:watch.rejections[key]=str(exc)
    def motor_cb(msg):
        try:watch.motor=(json.loads(msg.data),time.monotonic())
        except (ValueError,TypeError):watch.motor=None
    def scan_cb(msg):
        watch.scan=(msg,time.monotonic())
        if getattr(watch,'capture_scans',False):
            stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
            if stamp-getattr(watch,'last_saved_scan_ns',0)>=180_000_000:
                from capture_scan_anomalies import pack
                rows=getattr(watch,'scan_samples',[])
                rows.append(pack(msg,node.get_clock().now().nanoseconds))
                watch.scan_samples=rows[-180:];watch.last_saved_scan_ns=stamp
    def tf_cb(msg):
        for tf in msg.transforms:
            if (tf.header.frame_id,tf.child_frame_id)==('follower/base_link','follower/laser'):
                t=tf.transform.translation;q=tf.transform.rotation
                delta=(t.x-.26,t.y,t.z-.24,q.x,q.y,q.z,abs(q.w)-1)
                watch.mount=tf if all(math.isfinite(v) and abs(v)<.001 for v in delta) else None
    for key,topic in [('raw','/follower/odom_raw'),('filtered','/follower/odometry/filtered'),('map','/follower/odom_map')]:
        watch.subscriptions.append(node.create_subscription(Odometry,topic,lambda m,k=key:pose_cb(k,m),latest_state_qos()))
    watch.subscriptions.extend([
        node.create_subscription(Imu,'/follower/imu/data',lambda m:pose_cb('imu',m),latest_state_qos()),
        node.create_subscription(String,'/follower/motor_candidate/status',motor_cb,latest_state_qos()),
        node.create_subscription(LaserScan,'/follower/scan',scan_cb,latest_state_qos()),
        node.create_subscription(TFMessage,'/tf_static',tf_cb,QoSProfile(depth=20,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL))])
    return watch


def save(watch,kind,extra=None):
    folder=Path(__file__).resolve().parent/'logs';folder.mkdir(exist_ok=True)
    result=dict(synchronized_summary(watch.streams,begin_ns=(extra or {}).get('measurement_begin_ns')),
                **(extra or {}),rejections=watch.rejections,
                clearance_diagnostic=getattr(watch,'clearance_diagnostic',None))
    path=folder/f'{kind}_{time.time_ns()}.json'
    evidence=None
    if getattr(watch,'failed_scan',None) is not None:
        from capture_scan_anomalies import pack
        evidence=pack(*watch.failed_scan)
    result['saved_motion_scans']=len(getattr(watch,'scan_samples',[]))
    mount=None
    if getattr(watch,'mount',None) is not None and hasattr(watch.mount,'transform'):
        t=watch.mount.transform.translation;q=watch.mount.transform.rotation
        mount=dict(translation=[t.x,t.y,t.z],rotation_xyzw=[q.x,q.y,q.z,q.w])
    path.write_text(json.dumps(dict(summary=result,streams=watch.streams,events=watch.events,failed_scan=evidence,
        motion_scans=getattr(watch,'scan_samples',[]),mount=mount),allow_nan=False)+'\n')
    print(json.dumps(result,ensure_ascii=False,indent=2));print('JSON='+str(path));return result


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--seconds',type=int,default=30);a=p.parse_args()
    if not 5<=a.seconds<=120 or os.environ.get('ROS_DOMAIN_ID')!='10':p.error('seconds 5..120 and ROS_DOMAIN_ID=10 required')
    import rclpy
    rclpy.init(args=[]);node=rclpy.create_node('follower_yaw_observer_readonly');watch=attach(node)
    print('조회 전용: raw odom / IMU / filtered / map 방향을 같은 시간 구간으로 비교합니다. 명령·arm 없음.',flush=True)
    begin=time.monotonic()
    try:
        while rclpy.ok() and time.monotonic()-begin<a.seconds:rclpy.spin_once(node,timeout_sec=.05)
    finally:
        save(watch,'follower_yaw_observation',dict(control_connected=False))
        node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
