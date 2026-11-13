#!/usr/bin/env python3
"""Domain217 localhost MEMORY-only integration check of the new drive node.

No physical transport and no Domain10 inputs. Model RPM is not physical braking.
"""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run',action='store_true');a=p.parse_args()
    if not a.run:p.print_help();return
    os.environ['ROS_DOMAIN_ID']='217';os.environ['ROS_LOCALHOST_ONLY']='1'
    import rclpy
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from follower_drive_node import PREFIX
    rclpy.init(args=[]);node=rclpy.create_node('follower_drive_contract_check')
    status=[];child=None;logfile=None;passed=[]
    def stop():
        nonlocal child,logfile
        if child is not None and child.poll() is None:
            child.terminate()
            try:child.wait(timeout=3)
            except subprocess.TimeoutExpired:child.kill();child.wait()
        child=None
        if logfile is not None:logfile.close();logfile=None
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            if child is not None and child.poll() is not None:raise RuntimeError('Memory child exited unexpectedly; inspect logs')
            rclpy.spin_once(node,timeout_sec=.02)
    def wait(predicate,timeout=6.):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            if status and predicate(status[-1]):return status[-1]
            spin(.02)
        raise RuntimeError('Status wait timed out: '+str(status[-1] if status else None))
    def call(client):
        if not client.wait_for_service(timeout_sec=4.):raise RuntimeError('Service unavailable')
        future=client.call_async(Trigger.Request());end=time.monotonic()+4
        while not future.done() and time.monotonic()<end:spin(.02)
        if not future.done():raise RuntimeError('Service response timeout')
        return future.result()
    def expect(name,condition):
        if not condition:raise AssertionError(name)
        passed.append(name);print('PASS '+name,flush=True)
    def launch(enabled):
        nonlocal child,logfile
        stop();status.clear()
        path=Path(__file__).parent/'logs'/f'drive_ros_memory_{time.time_ns()}.log'
        path.parent.mkdir(exist_ok=True);logfile=path.open('x')
        cmd=[sys.executable,'-B',str(Path(__file__).with_name('follower_drive_node.py')),'--run-memory']
        if enabled:cmd.append('--enable-motion-input')
        child=subprocess.Popen(cmd,env=os.environ.copy(),stdout=logfile,stderr=subprocess.STDOUT)
        return wait(lambda s:s['mode']=='MEMORY_ONLY' and s['enabled']==enabled and s['valid_feedback']>=6)
    def arm():
        response=call(arm_client)
        expect('explicit_arm',response.success)
        epoch=json.loads(response.message)['epoch']
        wait(lambda s:s['armed'] and s['epoch']==epoch)
        return epoch
    def send(epoch,**overrides):
        obj=dict(version=1,epoch=epoch,frame='follower/base_link',stamp_ns=node.get_clock().now().nanoseconds,v=.05,w=0.)
        obj.update(overrides);pub.publish(String(data=json.dumps(obj)))
    try:
        spin(2.)
        if any(n!=node.get_name() for n,ns in node.get_node_names_and_namespaces()):raise RuntimeError('Domain217 already in use')
        subscription=node.create_subscription(String,PREFIX+'/status',lambda m:status.append(json.loads(m.data)),20)
        pub=node.create_publisher(String,PREFIX+'/manual_command',10)
        arm_client=node.create_client(Trigger,PREFIX+'/arm');disarm_client=node.create_client(Trigger,PREFIX+'/disarm')
        print('ISOLATED Domain217 localhost. New drive contract, MEMORY backend only; no real motors.',flush=True)
        s=launch(False)
        expect('starts_locked_and_zero',not s['armed'] and s['measured_rpm']==[0,0])
        expect('default_arm_rejected',not call(arm_client).success)
        launch(True);epoch=arm()
        end=time.monotonic()+.7
        while time.monotonic()<end:send(epoch);spin(.08)
        s=wait(lambda s:s['accepted_commands']>=3 and s['measured_rpm']==[-3,3])
        expect('forward_signs_and_3rpm_cap',s['sent_rpm']==[-3,3])
        # Keep commands alive to distinguish the fixed trial window from TTL loss.
        end=time.monotonic()+5.2
        while time.monotonic()<end:
            if status[-1]['fault']:break
            send(epoch);spin(.08)
        s=wait(lambda s:s['fault']=='FIVE_SECOND_TRIAL_EXPIRED' and s['measured_rpm']==[0,0])
        expect('fixed_window_stops_despite_fresh_commands',not s['armed'])
        expect('fault_cannot_service_rearm',not call(arm_client).success)
        launch(True);epoch=arm()
        for _ in range(5):send(epoch);spin(.08)
        s=wait(lambda s:s['fault']=='COMMAND_TIMEOUT' and s['measured_rpm']==[0,0])
        expect('command_loss_stops_and_latches',not s['armed'])
        send(epoch);spin(.3)
        expect('fresh_data_cannot_auto_rearm',not status[-1]['armed'] and status[-1]['measured_rpm']==[0,0])
        launch(True);epoch=arm();send('0'*16)
        s=wait(lambda s:s['fault'] is not None and not s['armed'])
        expect('wrong_epoch_rejected',s['rejected_commands']>=1)
        launch(True);epoch=arm()
        for _ in range(4):send(epoch);spin(.08)
        expect('explicit_disarm',call(disarm_client).success)
        s=wait(lambda s:not s['armed'] and s['measured_rpm']==[0,0])
        send(epoch);spin(.3)
        expect('old_commands_cannot_restart_after_disarm',not status[-1]['armed'])
        print(f'RESULT: {len(passed)}/{len(passed)} passed. Memory/ROS contract only; no hardware stopping validated.',flush=True)
    finally:
        stop();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
