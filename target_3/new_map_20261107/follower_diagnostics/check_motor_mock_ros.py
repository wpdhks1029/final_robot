#!/usr/bin/env python3
"""Opt-in Domain217 ROS test of a MEMORY-ONLY motor adapter; no physical ports."""
import argparse
import json
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
    from std_msgs.msg import String
    from nav_msgs.msg import Odometry
    from std_srvs.srv import Trigger,SetBool
    from follower_motor_mock import PREFIX

    rclpy.init(args=[])
    node=Node('follower_motor_mock_ros_check')
    child=None;status=[];odoms=[];passed=[]
    try:
        print('ISOLATED Domain217 localhost. MEMORY transport only; no real port or motor commands.',flush=True)
        end=time.monotonic()+2.
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.1)
        if any(name!=node.get_name() for name,ns in node.get_node_names_and_namespaces()):
            raise RuntimeError('Other Domain217 nodes found; refusing test')
        for topic in [PREFIX+'/manual_command',PREFIX+'/status',PREFIX+'/odom',
                      '/follower/odom_raw','/follower/cmd_vel','/cmd_vel']:
            if node.get_publishers_info_by_topic(topic) or node.get_subscriptions_info_by_topic(topic):
                raise RuntimeError(f'Existing endpoint: {topic}')
        status_sub=node.create_subscription(String,PREFIX+'/status',
            lambda msg:status.append(json.loads(msg.data)),20)
        odom_sub=node.create_subscription(Odometry,PREFIX+'/odom',lambda msg:odoms.append(msg),20)
        pub=node.create_publisher(String,PREFIX+'/manual_command',10)
        arm_client=node.create_client(Trigger,PREFIX+'/arm')
        disarm_client=node.create_client(Trigger,PREFIX+'/disarm')
        feedback_client=node.create_client(SetBool,PREFIX+'/set_feedback_enabled')

        def spin(seconds):
            end=time.monotonic()+seconds
            while time.monotonic()<end:
                if child is not None and child.poll() is not None:raise RuntimeError('Mock node exited unexpectedly')
                rclpy.spin_once(node,timeout_sec=.02)

        def wait(predicate,timeout=6.):
            end=time.monotonic()+timeout
            while time.monotonic()<end:
                if status and predicate(status[-1]):return status[-1]
                spin(.02)
            raise RuntimeError(f'Status wait timed out; latest={status[-1] if status else None}')

        def call(client,request):
            if not client.wait_for_service(timeout_sec=3.):raise RuntimeError('Mock service unavailable')
            future=client.call_async(request)
            rclpy.spin_until_future_complete(node,future,timeout_sec=3.)
            if not future.done() or future.result() is None:raise RuntimeError('Mock service response timeout')
            return future.result()

        def check(name,ok):
            print(f'{"PASS" if ok else "FAIL"} {name}',flush=True)
            if not ok:raise RuntimeError(f'{name}; latest={status[-1] if status else None}')
            passed.append(name)

        def start(enabled):
            nonlocal child
            status.clear();odoms.clear()
            cmd=[sys.executable,'-B',str(Path(__file__).with_name('follower_motor_mock.py'))]
            if enabled:cmd+=['--ros-args','-p','enable_simulated_motion:=true']
            child=subprocess.Popen(cmd,env=os.environ.copy(),stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
            return wait(lambda s:s['mode']=='MEMORY_ONLY'
                        and s['simulated_motion_enabled']==enabled and s['valid_feedback']>=2)

        def stop_child():
            nonlocal child
            child.terminate()
            try:logs,_=child.communicate(timeout=4.)
            except subprocess.TimeoutExpired:child.kill();logs,_=child.communicate()
            if child.returncode not in (0,-15):raise RuntimeError(f'Mock shutdown failed: {logs[-2000:]}')
            child=None

        def arm():
            # A sent STOP/model RPM0 is not enough: wait for a new zero-RPM reply.
            wait(lambda s:s['feedback_fresh'] and s['measured_rpm']==[0,0] and not s['io_failed'])
            response=call(arm_client,Trigger.Request())
            if not response.success:raise RuntimeError(f'Arm rejected: {response.message}')
            return json.loads(response.message)['epoch']

        def envelope(epoch,v=.02,**changes):
            obj={'version':1,'epoch':epoch,'frame':'follower/base_link',
                 'stamp_ns':node.get_clock().now().nanoseconds,'v':v,'w':0.}
            obj.update(changes);return json.dumps(obj)

        def stream(epoch,seconds=.7,v=.02):
            end=time.monotonic()+seconds;next_send=0.
            while time.monotonic()<end:
                if time.monotonic()>=next_send:
                    pub.publish(String(data=envelope(epoch,v)));next_send=time.monotonic()+.05
                spin(.01)

        first=start(False)
        check('default_mode_is_memory_only_and_locked',not first['armed'] and first['last_sent_rpm']==[0,0])
        response=call(arm_client,Trigger.Request())
        check('default_arm_rejected',not response.success)
        check('no_real_command_or_odom_publishers',all(not node.get_publishers_info_by_topic(t)
              for t in ['/follower/odom_raw','/follower/cmd_vel','/cmd_vel']))
        wait(lambda s:s['feedback_fresh'])
        check('mock_odom_frames_isolated',bool(odoms) and all(m.header.frame_id=='follower_motor_mock/odom'
              and m.child_frame_id=='follower_motor_mock/base_link' for m in odoms))
        stop_child()

        start(True)
        session=arm()
        check('explicit_mock_arm_returns_epoch',isinstance(session,str) and len(session)==16)
        stream(session)
        current=wait(lambda s:s['armed'] and s['last_sent_rpm'][0]<0 and s['last_sent_rpm'][1]>0)
        check('forward_left_inversion_and_rpm_cap',max(map(abs,current['last_sent_rpm']))<=3)
        check('mock_measured_odom_advances',any(m.pose.pose.position.x>0. for m in odoms))
        timeout_state=wait(lambda s:s['fault']=='COMMAND_TIMEOUT' and not s['armed'] and s['last_sent_rpm']==[0,0])
        check('command_loss_stops_and_latches',timeout_state['model_rpm']==[0,0])
        stream(session,seconds=.4)
        check('new_data_does_not_auto_rearm',not status[-1]['armed'] and status[-1]['fault']=='COMMAND_TIMEOUT')

        session2=arm()
        check('rearm_rotates_epoch',session2!=session)
        pub.publish(String(data=envelope(session)))
        wait(lambda s:s['fault']=='INVALID_COMMAND' and not s['armed'])
        check('old_session_rejected',True)

        session=arm();pub.publish(String(data=envelope(session,frame='base_link')))
        wait(lambda s:s['fault']=='INVALID_ENVELOPE')
        check('wrong_frame_envelope_stops',not status[-1]['armed'])

        session=arm();pub.publish(String(data=envelope(session,stamp_ns=1)))
        wait(lambda s:s['fault']=='INVALID_COMMAND')
        check('expired_or_prearm_stamp_rejected',not status[-1]['armed'])

        session=arm();before=status[-1]['accepted_commands']
        repeated=envelope(session,v=0.)
        pub.publish(String(data=repeated))
        wait(lambda s:s['accepted_commands']>before,timeout=1.)
        pub.publish(String(data=repeated))
        wait(lambda s:s['fault']=='INVALID_COMMAND')
        check('repeated_stamp_rejected',not status[-1]['armed'])

        session=arm();stream(session)
        call(feedback_client,SetBool.Request(data=False))
        stream(session,seconds=.7)
        failed=wait(lambda s:s['fault'] in ('RESPONSE_TIMEOUT','FEEDBACK_STALE')
                    and not s['armed'] and s['last_sent_rpm']==[0,0])
        check('feedback_loss_stops_despite_fresh_commands',not failed['rx_enabled'])
        call(feedback_client,SetBool.Request(data=True))
        wait(lambda s:s['feedback_fresh'] and s['model_rpm']==[0,0])
        check('feedback_recovery_keeps_fault_latched',not status[-1]['armed'] and status[-1]['fault'] is not None)

        session=arm();stream(session)
        wait(lambda s:s['armed'] and s['last_sent_rpm']!=[0,0])
        check('explicit_rearm_recovers_mock_motion',True)
        response=call(disarm_client,Trigger.Request())
        wait(lambda s:not s['armed'] and s['last_sent_rpm']==[0,0])
        check('explicit_disarm_stops',response.success)
        stream(session,seconds=.4)
        check('commands_after_disarm_cannot_restart',not status[-1]['armed'] and status[-1]['model_rpm']==[0,0])
        stop_child()
        print(f'RESULT: {len(passed)}/{len(passed)} passed.',flush=True)
        print('Memory model/ROS contract only. No hardware, real stopping, exclusive port or physical watchdog validation.')
        return 0
    except (Exception,KeyboardInterrupt) as exc:
        print(f'FAIL/STOP after {len(passed)} passed checks: {exc}',file=sys.stderr)
        return 1
    finally:
        if child is not None:
            child.terminate()
            try:logs,_=child.communicate(timeout=4.)
            except subprocess.TimeoutExpired:child.kill();logs,_=child.communicate()
            if child.returncode not in (0,-15):print('Mock child log:',logs[-2500:])
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':sys.exit(main())
