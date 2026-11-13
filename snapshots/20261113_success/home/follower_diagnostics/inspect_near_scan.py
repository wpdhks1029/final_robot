#!/usr/bin/env python3
"""Read-only samples of nearest scan returns; no command publishers or masking."""
import time
import argparse
import json
import math
from pathlib import Path

from follower_obstacle_preview import ObstacleCore,BASE,LASER


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--front-only',action='store_true',help='실제 TF로 변환한 전방 x>0.27m, |y|<0.75m 점만 표시')
    parser.add_argument('--save-json',type=Path,help='비교용 전체 스캔과 실제 TF 저장. 명령 발행 없음')
    args=parser.parse_args()
    saved=[]
    import yaml
    import rclpy
    from rclpy.node import Node
    from rclpy.time import Time
    from rclpy.duration import Duration
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    from tf2_ros import Buffer,TransformListener,TransformException

    class Inspector(Node):
        def __init__(self):
            super().__init__('follower_near_scan_inspector')
            cfg=Path(__file__).with_name('follower_hardware.yaml')
            self.core=ObstacleCore(**yaml.safe_load(cfg.read_text())['follower_obstacle_preview']['ros__parameters'])
            self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
            self.samples=0;self.last=0.
            self.sub=self.create_subscription(LaserScan,'/follower/scan',self.scan_cb,qos_profile_sensor_data)

        def scan_cb(self,msg):
            now=time.monotonic()
            if now-self.last<1.:
                return
            self.last=now
            if msg.header.frame_id!=LASER:
                print(f'Unexpected scan frame: {msg.header.frame_id}',flush=True)
                return
            try:
                tf=self.buffer.lookup_transform(BASE,LASER,Time.from_msg(msg.header.stamp),
                                                timeout=Duration(seconds=.05))
            except TransformException as exc:
                print(f'Waiting for measured-time TF: {exc}',flush=True)
                return
            self.core.update_scan(msg,tf,self.get_clock().now().nanoseconds,now)
            if self.core.scan is None:
                print('Scan invalid or stale; no sample accepted',flush=True)
                return
            if args.save_json:
                t=tf.transform.translation;q=tf.transform.rotation
                saved.append(dict(stamp_ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,
                    frame=msg.header.frame_id,angle_min=msg.angle_min,angle_increment=msg.angle_increment,
                    range_min=msg.range_min,range_max=msg.range_max,
                    ranges=[float(r) if math.isfinite(r) else None for r in msg.ranges],
                    translation=[t.x,t.y,t.z],rotation=[q.x,q.y,q.z,q.w]))
            self.samples+=1
            print(f'\nSAMPLE {self.samples}: valid_rays={self.core.scan[3]}, '
                  f'min_footprint_clearance={self.core.scan[2]:.4f}m',flush=True)
            if args.front_only:
                from watch_apf_case3 import project
                from apf_case3_candidate import Settings,scan_diagnostic
                import json
                try:
                    diagnostic=scan_diagnostic(project(msg,tf),Settings())
                except ValueError as exc:
                    print('Projection rejected: '+str(exc),flush=True)
                    return
                print('FRONT_ONLY '+json.dumps(diagnostic,allow_nan=False),flush=True)
                print('front_gap_m = x_m - 0.27; left is positive y_m',flush=True)
                return
            print(' index | scan_angle(deg) | raw_range(m) | base_x(m) | base_y(m) | clearance(m)',flush=True)
            for clearance,index,angle,r,x,y in self.core.nearest_points:
                print(f'{index:6d} | {angle:15.2f} | {r:12.4f} | {x:9.4f} | {y:9.4f} | {clearance:12.4f}',flush=True)

    rclpy.init(args=[])
    node=Inspector()
    try:
        print('READ ONLY: collecting 5 scan samples, at most 12 seconds. No masks/commands.',flush=True)
        end=time.monotonic()+12.
        while rclpy.ok() and node.samples<5 and time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.1)
        print(f'Done: {node.samples}/5 samples.',flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        if args.save_json:
            args.save_json.parent.mkdir(parents=True,exist_ok=True)
            args.save_json.write_text(json.dumps(dict(samples=saved),allow_nan=False)+'\n')
            print('JSON='+str(args.save_json),flush=True)
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
