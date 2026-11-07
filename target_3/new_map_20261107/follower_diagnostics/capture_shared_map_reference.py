#!/usr/bin/env python3
"""Capture a separate stationary map reference. No publisher or service calls."""
import argparse
import hashlib
import json
import math
import os
import time
from pathlib import Path


def reference_line(follower, leader):
    if not all(math.isfinite(v) for v in (*follower, *leader)):
        raise ValueError('INVALID_REFERENCE_POSE')
    dx, dy = leader[0]-follower[0], leader[1]-follower[1]
    if math.hypot(dx, dy) < .5:
        raise ValueError('REFERENCE_ROBOTS_TOO_CLOSE')
    heading = math.atan2(dy, dx)
    for pose in (follower, leader):
        error = math.atan2(math.sin(pose[2]-heading), math.cos(pose[2]-heading))
        if abs(error) > math.radians(20):
            raise ValueError('REFERENCE_ROBOTS_NOT_ALIGNED')
    return dict(reason='REFERENCE_LINE_CAPTURED', control_connected=False,
                evaluation_pass=False, reference_only=True,
                snapshot=dict(reference_origin=list(follower[:2]), reference_heading=heading,
                              line_fit_residual_m=0., follower_map_pose=list(follower),
                              leader_map_pose=list(leader)))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID')!='10':parser.error('ROS_DOMAIN_ID=10 required')
    if args.output.exists():parser.error('Output already exists; choose a new filename')
    from follower_map_selection import selected_map
    map_path,yaml_sha,image_sha=selected_map(args.map,Path('/nonexistent/legacy-map'),'','')
    import rclpy
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from follower_remote_qos import latest_state_qos
    from follower_pursuit_preview import PreviewCore
    from check_temporary_detour import stationary_inputs
    rclpy.init(args=[]);node=rclpy.create_node('capture_shared_map_reference_readonly')
    nav=PreviewCore();motor=None
    def status(msg):
        nonlocal motor
        try:motor=(json.loads(msg.data),time.monotonic())
        except (ValueError,TypeError):motor=None
    def odom(key,msg):nav.odom_callback(key,msg,node.get_clock().now().nanoseconds,time.monotonic())
    subs=[node.create_subscription(Odometry,'/leader/odom',lambda m:odom('leader',m),latest_state_qos()),
          node.create_subscription(Odometry,'/follower/odom_map',lambda m:odom('follower',m),latest_state_qos()),
          node.create_subscription(String,'/follower/motor_candidate/status',status,latest_state_qos())]
    print('조회 전용: 양쪽 정지·후발 비무장 상태에서 새 지도 기준선 저장. 명령·arm 없음.',flush=True)
    end=time.monotonic()+15.;stable=None;baseline=None;last='WAIT_INPUTS';samples=set()
    try:
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.02)
            mono=time.monotonic();now=node.get_clock().now().nanoseconds
            try:
                stationary_inputs(nav,motor,now,mono)
                poses=tuple(nav.inputs[k][2] for k in ('follower','leader'))
                data=reference_line(*poses)
                if baseline is not None:
                    for p,b in zip(poses,baseline):
                        angle=math.atan2(math.sin(p[2]-b[2]),math.cos(p[2]-b[2]))
                        if math.dist(p[:2],b[:2])>.03 or abs(angle)>math.radians(2):
                            raise ValueError('POSE_NOT_STABLE')
                if stable is None:stable=mono;baseline=poses
                samples.add(tuple((nav.inputs[k][0].sec,nav.inputs[k][0].nanosec) for k in ('follower','leader')))
                if mono-stable<3. or len(samples)<10:continue
                # Ensure the map files did not change during capture.
                if selected_map(args.map,Path('/nonexistent/legacy-map'),'','')[1:]!=(yaml_sha,image_sha):
                    raise ValueError('MAP_CHANGED_DURING_CAPTURE')
                data.update(map_yaml=str(map_path),map_yaml_sha256=yaml_sha,map_image_sha256=image_sha,
                            captured_ros_ns=now,stationary_samples=len(samples))
                args.output.parent.mkdir(parents=True,exist_ok=True)
                with args.output.open('x') as f:f.write(json.dumps(data,ensure_ascii=False,allow_nan=False,indent=2)+'\n')
                print('REFERENCE_JSON='+str(args.output),flush=True)
                print(json.dumps(data['snapshot'],ensure_ascii=False),flush=True)
                return 0
            except (ValueError,KeyError,TypeError) as exc:
                last=str(exc);stable=None;baseline=None;samples.clear()
        print('REFERENCE_NOT_SAVED='+last,flush=True);return 1
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':raise SystemExit(main())
