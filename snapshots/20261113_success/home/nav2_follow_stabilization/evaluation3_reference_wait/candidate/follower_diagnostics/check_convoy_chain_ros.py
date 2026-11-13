#!/usr/bin/env python3
"""Synthetic Domain217 preview->guard chain check. No hardware or real command output."""
from follower_remote_qos import latest_state_qos
import argparse
import math
import os
from pathlib import Path
import subprocess
import sys
import time


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run',action='store_true')
    args=parser.parse_args()
    if not args.run:parser.print_help();return 0
    os.environ['ROS_DOMAIN_ID']='217';os.environ['ROS_LOCALHOST_ONLY']='1'
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import Twist,PoseStamped,TransformStamped
    from nav_msgs.msg import Odometry,Path as RosPath
    from sensor_msgs.msg import LaserScan
    from tf2_ros import StaticTransformBroadcaster
    from builtin_interfaces.msg import Time as Stamp

    rclpy.init(args=[])
    node=Node('follower_synthetic_chain_check')
    children=[];samples={'raw':[],'guard':[]};failures=[]
    try:
        print('ISOLATED Domain217 localhost-only. Two preview processes, no motor connection.',flush=True)
        end=time.monotonic()+2.
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.1)
        if any(name!=node.get_name() for name,ns in node.get_node_names_and_namespaces()):
            raise RuntimeError('Other Domain217 nodes found; refusing test')
        for topic in ['/leader/path','/leader/odom','/follower/odom_map','/follower/scan',
                      '/follower/cmd_vel_test','/follower/cmd_vel_guard_test',
                      '/follower/cmd_vel','/follower/cmd_vel_raw','/tf_static']:
            if node.get_publishers_info_by_topic(topic) or node.get_subscriptions_info_by_topic(topic):
                raise RuntimeError(f'Existing Domain217 endpoint: {topic}')
        pubs={'path':node.create_publisher(RosPath,'/leader/path',10),
              'leader':node.create_publisher(Odometry,'/leader/odom',10),
              'follower':node.create_publisher(Odometry,'/follower/odom_map',10),
              'scan':node.create_publisher(LaserScan,'/follower/scan',qos_profile_sensor_data)}
        subs=[]
        for key,topic in [('raw','/follower/cmd_vel_test'),('guard','/follower/cmd_vel_guard_test')]:
            subs.append(node.create_subscription(Twist,topic,
                lambda msg,key=key:samples[key].append((time.monotonic(),msg.linear.x,msg.angular.z)),latest_state_qos()))
        broadcaster=StaticTransformBroadcaster(node)
        tf=TransformStamped();tf.header.frame_id='follower/base_link';tf.child_frame_id='follower/laser'
        tf.transform.translation.x=.26;tf.transform.translation.z=.24;tf.transform.rotation.w=1.
        tf.header.stamp=node.get_clock().now().to_msg();broadcaster.sendTransform(tf)
        root=Path(__file__).resolve().parent
        for file in ('follower_pursuit_preview.py','follower_obstacle_preview.py'):
            children.append(subprocess.Popen([sys.executable,'-B',str(root/file)],env=os.environ.copy(),
                stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True))
        end=time.monotonic()+8.
        while time.monotonic()<end and not all(samples.values()):
            if any(p.poll() is not None for p in children):raise RuntimeError('Preview exited during startup')
            rclpy.spin_once(node,timeout_sec=.1)
        if not all(samples.values()):raise RuntimeError('Both outputs were not discovered')

        def publish(config):
            ns=node.get_clock().now().nanoseconds
            stamp=Stamp(sec=ns//1_000_000_000,nanosec=ns%1_000_000_000)
            outgoing={}
            for key,child_frame in [('leader','base_link'),('follower','follower/base_link')]:
                msg=Odometry();msg.header.stamp=stamp;msg.header.frame_id='map'
                msg.child_frame_id=child_frame;msg.pose.pose.orientation.w=1.
                if key=='leader':
                    msg.pose.pose.position.x=3.;msg.twist.twist.linear.x=config.get('vx',.1)
                    if config.get('bad_leader'):msg.header.frame_id='wrong'
                outgoing[key]=msg
            path=RosPath();path.header.stamp=stamp;path.header.frame_id='map'
            for x,y in config.get('points',[(0.,0.),(3.,0.)]):
                point=PoseStamped();point.header.stamp=stamp;point.header.frame_id='map'
                point.pose.position.x=x;point.pose.position.y=y;point.pose.orientation.w=1.
                path.poses.append(point)
            outgoing['path']=path
            scan=LaserScan();scan.header.frame_id='follower/laser'
            scan_ns=ns+config.get('scan_offset_ns',0)
            scan.header.stamp=Stamp(sec=scan_ns//1_000_000_000,nanosec=scan_ns%1_000_000_000)
            scan.angle_min=config.get('angle',0.);scan.angle_max=scan.angle_min;scan.angle_increment=.01
            scan.range_min=.28;scan.range_max=16.;scan.ranges=[config.get('r',2.)]
            outgoing['scan']=scan
            for key,msg in outgoing.items():
                if key!=config.get('omit'):pubs[key].publish(msg)

        zero=lambda v,w:abs(v)<1e-9 and abs(w)<1e-9
        forward=lambda v,w:0.<v<=.1 and abs(w)<1e-9
        cases=[
            ('straight_chain',{},forward,forward),
            ('left_chain',{'points':[(0.,0.),(3.,1.)]},lambda v,w:v>0 and w>0,lambda v,w:v>0 and w>0),
            ('right_chain',{'points':[(0.,0.),(3.,-1.)]},lambda v,w:v>0 and w<0,lambda v,w:v>0 and w<0),
            ('front_slow_chain',{'r':.5},forward,lambda v,w:abs(v-.0633333333)<1e-5 and abs(w)<1e-9),
            ('front_obstacle_overrides_forward',{'r':.3},forward,zero),
            ('side_obstacle_overrides_forward',{'r':.5,'angle':math.pi/2},forward,zero),
            ('bad_scan_stamp_overrides_forward',{'scan_offset_ns':-1_000_000_000},forward,zero),
            ('recover_before_scan_loss',{},forward,forward),
            ('scan_loss_overrides_forward',{'omit':'scan'},forward,zero),
            ('recover_before_leader_loss',{},forward,forward),
            ('leader_loss_chain',{'omit':'leader'},zero,zero),
            ('recover_before_follower_loss',{},forward,forward),
            ('follower_loss_chain',{'omit':'follower'},zero,zero),
            ('recover_before_path_loss',{},forward,forward),
            ('path_loss_chain',{'omit':'path'},zero,zero),
            ('one_point_reset_chain',{'points':[(0.,0.)]},zero,zero),
            ('leader_stop_chain',{'vx':0.},zero,zero),
            ('behind_target_chain',{'points':[(-3.,0.),(-2.,0.)]},zero,zero),
            ('final_recovery',{},forward,forward),
        ]
        for name,config,raw_check,guard_check in cases:
            samples['raw'].clear();samples['guard'].clear()
            start=time.monotonic();next_pub=start
            while time.monotonic()-start<1.8:
                if any(p.poll() is not None for p in children):raise RuntimeError('A preview exited')
                now=time.monotonic()
                if now>=next_pub:publish(config);next_pub=now+.05
                rclpy.spin_once(node,timeout_sec=.01)
            late={key:[(v,w) for t,v,w in values if t-start>=1.3] for key,values in samples.items()}
            passed=all(len(late[k])>=3 and all(check(v,w) for v,w in late[k])
                       for k,check in [('raw',raw_check),('guard',guard_check)])
            print(f'{"PASS" if passed else "FAIL"} {name}: raw={late["raw"][-1] if late["raw"] else None}, '
                  f'guard={late["guard"][-1] if late["guard"] else None}',flush=True)
            if not passed:failures.append(name)
        # Explicit producer process failure: guard must expire a last nonzero Twist.
        # Kill only this synthetic child, so its graceful zero-output cleanup
        # cannot conceal a broken guard command-age timeout.
        children[0].kill()
        children[0].wait(timeout=3)
        samples['guard'].clear();start=time.monotonic();next_pub=start
        while time.monotonic()-start<1.8:
            now=time.monotonic()
            if now>=next_pub:publish({});next_pub=now+.05
            rclpy.spin_once(node,timeout_sec=.01)
        late=[(v,w) for t,v,w in samples['guard'] if t-start>=1.3]
        passed=len(late)>=3 and all(zero(v,w) for v,w in late)
        print(f'{"PASS" if passed else "FAIL"} pursuit_process_exit: guard={late[-1] if late else None}',flush=True)
        if not passed:failures.append('pursuit_process_exit')
        total=len(cases)+1
        print(f'RESULT: {total-len(failures)}/{total} passed. Failures={failures}',flush=True)
        print('Synthetic chain late-phase checks only. No physical braking or exact end-to-end stop latency certified.')
        return 1 if failures else 0
    except (Exception,KeyboardInterrupt) as exc:
        print(f'Chain test stopped: {exc}',file=sys.stderr);return 1
    finally:
        for child in children:
            if child.poll() is None:child.terminate()
            try:logs,_=child.communicate(timeout=4)
            except subprocess.TimeoutExpired:child.kill();logs,_=child.communicate()
            expected_kill=child is children[0] and child.returncode==-9
            if child.returncode not in (0,-15) and not expected_kill:
                print('Child exit/log:',child.returncode,logs[-2500:])
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':sys.exit(main())
