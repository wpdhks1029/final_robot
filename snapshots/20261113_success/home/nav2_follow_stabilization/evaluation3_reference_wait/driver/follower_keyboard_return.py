#!/usr/bin/env python3
"""Opt-in keyboard client for the existing bounded follower driver; no serial I/O.

a: explicit arm, i: forward, j/l: pivot, k/space: zero, x: exit.
Motion intent expires 0.3s after the last motion key. No reverse translation.
"""
import argparse
import json
import math
import select
import sys
import termios
import time
import tty

from local_avoidance_drive import COMMAND_TOPIC, PROFILE
from run_local_avoidance_trial import motor_problem


class KeyIntent:
    def __init__(self):
        self.key=None
        self.at=None

    def update(self, key, mono):
        if key in ('i','j','l'):
            self.key=key;self.at=mono
        else:
            self.key=None;self.at=None

    def command(self, mono):
        if self.at is None or not 0<=mono-self.at<.3:
            return 'STOP',0.,0.
        return {'i':('DRIVE',.04,0.),'j':('TURN',0.,.10),
                'l':('TURN',0.,-.10)}[self.key]


def envelope(intent, mono, ns, epoch, source, scan, arm_ns):
    if any(type(x) is not int or x<=0 or not 0<=(ns-x)/1e9<=.8
           for x in (source,scan)):
        raise ValueError('LOCAL_SENSOR_STALE_OR_MISSING')
    motion,v,w=intent.command(mono)
    if type(arm_ns) is not int or min(source,scan)<arm_ns:
        motion,v,w='STOP',0.,0.
    return dict(version=3,controller=PROFILE,epoch=epoch,frame='follower/base_link',
        stamp_ns=ns,source_stamp_ns=min(source,scan),scan_stamp_ns=scan,
        motion=motion,v=v,w=w,clearance_m=0.,swept_clearance_m=0.)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run',action='store_true')
    parser.add_argument('--ack-clear-area-power-cut-ready',action='store_true')
    args=parser.parse_args()
    if not args.run:
        parser.print_help();return 0
    if not args.ack_clear_area_power_cut_ready or not sys.stdin.isatty():
        parser.error('Clear area/power-cut acknowledgement and interactive terminal required')
    import rclpy
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from follower_remote_qos import latest_state_qos
    from rclpy.qos import QoSProfile,ReliabilityPolicy

    rclpy.init(args=[])
    node=rclpy.create_node('follower_keyboard_return')
    motor=None;raw=None;scan=None;epoch=None;arm_at=None;arm_pending=False
    intent=KeyIntent();original_terminal=None;code=0
    def now():return node.get_clock().now().nanoseconds
    def status(msg):
        nonlocal motor
        try:
            value=json.loads(msg.data)
            motor=(value,time.monotonic()) if isinstance(value,dict) else None
        except ValueError:motor=None
    def stamp(msg):return msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
    def raw_cb(msg):
        nonlocal raw
        raw=(stamp(msg),time.monotonic()) if msg.header.frame_id=='follower/odom' and msg.child_frame_id=='follower/base_link' else None
    def scan_cb(msg):
        nonlocal scan
        scan=(stamp(msg),time.monotonic()) if msg.header.frame_id=='follower/laser' else None
    def sensors(mono):
        if any(r is None or not 0<=mono-r[1]<=.8 for r in (raw,scan)):
            raise ValueError('LOCAL_SENSOR_STALE_OR_MISSING')
        return raw[0],scan[0]
    def exclusive():
        owners=node.get_publishers_info_by_topic(COMMAND_TOPIC)
        if len(owners)!=1 or owners[0].node_name!=node.get_name():
            raise ValueError('STOP_FOLLOW_OPERATOR_FIRST')
    def call(client):
        if not client.service_is_ready():raise ValueError('DRIVER_SERVICE_UNAVAILABLE')
        future=client.call_async(Trigger.Request());end=time.monotonic()+.5
        while not future.done() and time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.01)
        if not future.done():raise ValueError('DRIVER_SERVICE_TIMEOUT')
        return future.result()

    command_pub=node.create_publisher(String,COMMAND_TOPIC,
        QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE))
    subscriptions=[node.create_subscription(String,'/follower/motor_candidate/status',status,latest_state_qos()),
        node.create_subscription(Odometry,'/follower/odom_raw',raw_cb,latest_state_qos()),
        node.create_subscription(LaserScan,'/follower/scan',scan_cb,latest_state_qos())]
    arm=node.create_client(Trigger,'/follower/motor_candidate/arm')
    disarm=node.create_client(Trigger,'/follower/motor_candidate/disarm')
    try:
        until=time.monotonic()+2.
        while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.02)
        exclusive()
        issue=motor_problem(motor,time.monotonic(),360,True,disable_clearance_stop=True)
        if issue:raise ValueError(issue)
        sensors(time.monotonic())
        print('a: arm | i: 전진 | j/l: 좌/우 제자리 회전 | k/스페이스: 정지 | x: 종료\n'
              '이동 키를 누르고 유지하세요. 후진 입력 없음. 장애물 거리 정지 해제 상태.',flush=True)
        original_terminal=termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        last_graph=0.
        while rclpy.ok():
            rclpy.spin_once(node,timeout_sec=.01)
            mono=time.monotonic()
            if mono-last_graph>.2:
                exclusive();last_graph=mono
            if select.select([sys.stdin],[],[],0)[0]:
                key=sys.stdin.read(1)
                intent.update(key,mono)
                if key in ('x','\x03',''):
                    break
                if key=='a' and epoch is None:
                    issue=motor_problem(motor,mono,360,True,disable_clearance_stop=True)
                    if issue:raise ValueError(issue)
                    sensors(mono)
                    arm_pending=True
                    response=call(arm)
                    if not response.success:raise ValueError('ARM_REJECTED: '+response.message)
                    epoch=json.loads(response.message)['epoch'];arm_at=time.monotonic()
                    intent.update('k',arm_at)
            if epoch is None:continue
            mono=time.monotonic()
            issue=motor_problem(motor,mono,360,disable_clearance_stop=True)
            if issue:raise ValueError(issue)
            state=motor[0]
            matching=state.get('armed') is True and state.get('epoch')==epoch
            if not matching and mono-arm_at>.2:raise ValueError('SESSION_CHANGED_OR_DISARMED')
            source,scan_stamp=sensors(mono)
            # Until matching arm status arrives, publish only zero commands.
            obj=envelope(intent,mono,now(),epoch,source,scan_stamp,
                         state.get('arm_ros_ns') if matching else None)
            command_pub.publish(String(data=json.dumps(obj,allow_nan=False)))
    except KeyboardInterrupt:pass
    except Exception as exc:
        print('STOP: '+str(exc),file=sys.stderr,flush=True);code=1
    finally:
        try:
            if arm_pending and disarm.service_is_ready():
                call(disarm)
        finally:
            if original_terminal is not None:termios.tcsetattr(sys.stdin,termios.TCSADRAIN,original_terminal)
            node.destroy_node()
            if rclpy.ok():rclpy.shutdown()
    return code


if __name__=='__main__':raise SystemExit(main())
