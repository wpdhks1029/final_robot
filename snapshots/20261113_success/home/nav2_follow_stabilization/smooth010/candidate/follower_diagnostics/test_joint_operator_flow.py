"""가짜 ROS/시계로 운용 도구 전체 흐름 검사. 실제 ROS·네트워크·포트 접근 없음."""
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace as S
import unittest
from unittest.mock import patch

import run_joint_follow_trial as joint


def twist():return S(linear=S(x=0.,y=0.,z=0.),angular=S(x=0.,y=0.,z=0.))


class Flow:
    def __init__(self,guard_loss=False,duplicate=False,trial_seconds=5,leader_speed=.05,leader_turn='straight',expect_obstacle=False,obstacle=False):
        self.now=1.;self.leader_v=0.;self.leader_w=0.;self.armed=False;self.arm_time=None
        self.guard_loss=guard_loss;self.duplicate=duplicate
        self.trial_seconds=trial_seconds
        self.leader_speed=leader_speed
        self.leader_turn=leader_turn
        self.expect_obstacle=expect_obstacle;self.obstacle=obstacle
        self.callbacks={};self.events=[];self.ok=True;self.subscription_qos={}

    def ros(self):return 100_000_000_000+int(self.now*10**9)
    def header(self):
        t=self.ros();return S(frame_id='map',stamp=S(sec=t//10**9,nanosec=t%10**9))
    def state(self):
        return dict(mode='HARDWARE_CANDIDATE',input_kind='guard',trial_max_rpm=10,
                    trial_max_seconds=self.trial_seconds,
                    enabled=True,armed=self.armed,epoch='0123456789abcdef',fault=None,
                    feedback_fresh=True,io_failed=False,measured_rpm=[-10,10] if self.armed else [0,0],
                    valid_feedback=int(self.now*100))
    def get_clock(self):return S(now=lambda:S(nanoseconds=self.ros()))
    def get_name(self):return 'joint_follow_trial_operator'
    def create_publisher(self,kind,topic,depth):
        def publish(m):
            self.leader_v=m.linear.x;self.leader_w=m.angular.z
            self.events.append(('leader_command',self.leader_v,self.now,self.leader_w))
        return S(publish=publish)
    def create_subscription(self,kind,topic,callback,qos):
        self.callbacks[topic]=callback;self.subscription_qos[topic]=qos;return callback
    def create_client(self,kind,name):
        def call(request):
            is_arm=name.endswith('/arm')
            self.armed=is_arm
            if is_arm:self.arm_time=self.now
            self.events.append(('arm' if is_arm else 'disarm',self.now))
            response=S(success=True,message=json.dumps(dict(epoch='0123456789abcdef')))
            return S(done=lambda:True,result=lambda:response)
        return S(wait_for_service=lambda **kw:True,service_is_ready=lambda:True,call_async=call)
    def get_publishers_info_by_topic(self,topic):
        if topic=='/follower/obstacle_preview/status':return [S(node_name='follower_obstacle_preview')]
        if topic=='/cmd_vel':
            return [S(node_name=self.get_name())]+([S(node_name='other')] if self.duplicate else [])
        return [S(node_name='follower_guard_gateway')]
    def get_subscriptions_info_by_topic(self,topic):
        return [S(node_name='md200t_controller' if topic=='/cmd_vel' else 'follower_drive_candidate')]
    def destroy_node(self):pass
    def spin_once(self,node,timeout_sec):
        self.now+=.02
        for topic,callback in list(self.callbacks.items()):
            h=self.header()
            if topic.endswith('/status'):
                d=self.state() if 'motor_candidate' in topic else dict(reason='FAKE')
                if 'motor_candidate' not in topic and self.obstacle and self.arm_time is not None and self.now-self.arm_time>.3:
                    d=dict(version=1,stamp_ns=self.ros(),source_stamp_ns=self.ros(),scan_source_stamp_ns=self.ros(),
                           stamped_reason='STOP_FOOTPRINT_CLEARANCE',clearance_zone='stop',clearance_m=.25,
                           output_v=0.,output_w=0.,nearest_points=[dict(base_x_m=.52,base_y_m=.1,clearance_m=.25)])
                callback(S(data=json.dumps(d)))
            elif 'guard_stamped' in topic:
                h.frame_id='follower/base_link';t=twist()
                t.linear.x=.1 if self.leader_v>0 else 0.
                if (self.guard_loss or self.obstacle) and self.arm_time is not None and self.now-self.arm_time>.3:t.linear.x=0.
                callback(S(header=h,twist=t))
            elif topic=='/follower/scan':
                h.frame_id='follower/laser'
                callback(S(header=h,range_min=.28,range_max=16.,ranges=[1.]))
            elif topic.endswith('/path'):callback(S(header=h,poses=[S(),S()]))
            else:
                t=twist();t.linear.x=self.leader_v if topic=='/leader/odom' else 0.
                t.angular.z=self.leader_w if topic=='/leader/odom' else 0.
                callback(S(header=h,child_frame_id='base_link' if topic=='/leader/odom' else 'follower/base_link',
                           twist=S(twist=t)))

    def modules(self):
        flow=self
        class Core:
            def path_callback(self,*args,**kwargs):pass
            def odom_callback(self,*args):pass
            def command(self,*args):
                return (.1,0.,'PREVIEW_TRACK') if flow.leader_v>0 else (0.,0.,'STOP_LEADER_STOPPED')
        return {
            'rclpy':S(init=lambda **kw:None,create_node=lambda name:self,spin_once=self.spin_once,
                       ok=lambda:self.ok,shutdown=lambda:setattr(self,'ok',False)),
            'rclpy.signals':S(SignalHandlerOptions=S(NO=0)),
            'rclpy.qos':S(qos_profile_sensor_data=object(),QoSProfile=lambda **kw:S(**kw),
                         ReliabilityPolicy=S(BEST_EFFORT='BEST_EFFORT'),
                         DurabilityPolicy=S(VOLATILE='VOLATILE'),HistoryPolicy=S(KEEP_LAST='KEEP_LAST')),
            'geometry_msgs':S(),'geometry_msgs.msg':S(Twist=twist,TwistStamped=object),
            'nav_msgs':S(),'nav_msgs.msg':S(Odometry=object,Path=object),
            'sensor_msgs':S(),'sensor_msgs.msg':S(LaserScan=object),
            'std_msgs':S(),'std_msgs.msg':S(String=object),
            'std_srvs':S(),'std_srvs.srv':S(Trigger=S(Request=lambda:S())),
            'follower_pursuit_preview':S(PreviewCore=Core),
        }


class Tests(unittest.TestCase):
    def run_flow(self,**kw):
        flow=Flow(**kw)
        def start(spin):spin();return True
        with tempfile.TemporaryDirectory() as tmp:
            with patch.dict(sys.modules,flow.modules()),patch.dict('os.environ',{'ROS_DOMAIN_ID':'10'}),\
                 patch('sys.argv',['joint','--run','--ack-clear-both-power-cut-ready',
                                   '--trial-seconds',str(flow.trial_seconds),'--leader-speed',str(flow.leader_speed),
                                   '--leader-turn',flow.leader_turn]+(['--expect-obstacle-stop'] if flow.expect_obstacle else [])),\
                 patch.object(joint,'__file__',tmp+'/operator.py'),\
                 patch.object(joint.time,'monotonic',side_effect=lambda:flow.now),\
                 patch.object(joint,'wait_for_start',side_effect=start):
                result=joint.main()
            log=next((Path(tmp)/'logs').glob('*.jsonl'))
            rows=[json.loads(x) for x in log.read_text().splitlines()]
        return flow,result,rows

    def test_nominal_requests_both_stops_in_same_control_time(self):
        f,result,rows=self.run_flow()
        self.assertEqual(result,0)
        self.assertEqual(sum(e[0]=='arm' for e in f.events),1)
        stop=next(e for e in f.events if e[0]=='disarm')
        zeros=[e for e in f.events if e[0]=='leader_command' and e[1]==0.]
        self.assertTrue(zeros);self.assertEqual(zeros[0][2],stop[1])
        self.assertTrue(rows[-1]['trial_success'])
        for topic in ('/leader/odom','/leader/path','/follower/odom_map','/follower/cmd_vel_guard_stamped_test','/follower/scan'):
            qos=f.subscription_qos[topic]
            self.assertEqual(qos.reliability,'BEST_EFFORT');self.assertEqual(qos.depth,1)
        self.assertEqual(f.subscription_qos['/follower/motor_candidate/status'],1)

    def test_expected_front_obstacle_stop_requires_evidence_and_both_zeros(self):
        f,result,rows=self.run_flow(expect_obstacle=True,obstacle=True)
        self.assertEqual(result,0);self.assertTrue(rows[-1]['obstacle_stop_confirmed'])
        self.assertTrue(rows[-1]['leader_zero_confirmed']);self.assertTrue(rows[-1]['follower_zero_confirmed'])
        self.assertEqual(sum(e[0]=='arm' for e in f.events),1)
        self.assertEqual(rows[-1]['reason'],'GUARD_STOP_OR_INVALID')

    def test_time_limit_without_obstacle_cannot_pass_expected_stop(self):
        f,result,rows=self.run_flow(expect_obstacle=True)
        self.assertEqual(result,1);self.assertFalse(rows[-1]['obstacle_stop_confirmed'])
        self.assertFalse(rows[-1]['trial_success'])

    def test_unexplained_zero_guard_cannot_pass_expected_stop(self):
        f,result,rows=self.run_flow(expect_obstacle=True,guard_loss=True)
        self.assertEqual(result,1);self.assertFalse(rows[-1]['obstacle_stop_confirmed'])
        self.assertTrue(rows[-1]['leader_zero_confirmed']);self.assertTrue(rows[-1]['follower_zero_confirmed'])

    def test_guard_stop_stops_both_without_rearm(self):
        f,result,rows=self.run_flow(guard_loss=True)
        self.assertEqual(result,1)
        self.assertEqual(sum(e[0]=='arm' for e in f.events),1)
        self.assertEqual(rows[-1]['reason'],'GUARD_STOP_OR_INVALID')
        self.assertTrue(rows[-1]['leader_zero_confirmed'])
        self.assertTrue(rows[-1]['follower_zero_confirmed'])

    def test_longer_window_stops_both_at_nine_point_five_seconds(self):
        f,result,rows=self.run_flow(trial_seconds=10)
        self.assertEqual(result,0)
        armed=next(e[1] for e in f.events if e[0]=='arm')
        stopped=next(e[1] for e in f.events if e[0]=='disarm')
        self.assertGreaterEqual(stopped-armed,9.5)
        self.assertLess(stopped-armed,9.6)
        self.assertTrue(rows[-1]['trial_success'])

    def test_thirty_second_left_turn_stops_both(self):
        f,result,rows=self.run_flow(trial_seconds=30,leader_speed=.04,leader_turn='left')
        self.assertEqual(result,0)
        armed=next(e[1] for e in f.events if e[0]=='arm')
        stopped=next(e[1] for e in f.events if e[0]=='disarm')
        self.assertGreaterEqual(stopped-armed,29.5)
        self.assertLess(stopped-armed,29.6)
        self.assertTrue(rows[-1]['trial_success'])

    def test_sixty_second_straight_supervisor_stops_once(self):
        f,result,rows=self.run_flow(trial_seconds=60,leader_speed=.04)
        self.assertEqual(result,0)
        armed=next(e[1] for e in f.events if e[0]=='arm')
        stopped=next(e[1] for e in f.events if e[0]=='disarm')
        self.assertGreaterEqual(stopped-armed,59.5);self.assertLess(stopped-armed,59.6)
        self.assertEqual(sum(e[0]=='arm' for e in f.events),1)
        self.assertTrue(rows[-1]['leader_zero_confirmed']);self.assertTrue(rows[-1]['follower_zero_confirmed'])
        self.assertTrue(rows[-1]['trial_success'])

    def test_lower_leader_speed_keeps_same_stop_contract(self):
        f,result,rows=self.run_flow(trial_seconds=10,leader_speed=.04)
        self.assertEqual(result,0)
        values={e[1] for e in f.events if e[0]=='leader_command'}
        self.assertEqual(values,{0.,.04})
        self.assertTrue(rows[-1]['leader_zero_confirmed'])
        self.assertTrue(rows[-1]['follower_zero_confirmed'])

    def test_left_and_right_arc_stop_linear_and_angular_together(self):
        for direction,expected in [('left',.03),('right',-.03)]:
            f,result,rows=self.run_flow(trial_seconds=10,leader_speed=.04,leader_turn=direction)
            self.assertEqual(result,0)
            commands=[e for e in f.events if e[0]=='leader_command']
            self.assertEqual({(e[1],e[3]) for e in commands},{(.04,expected),(0.,0.)})
            self.assertEqual((commands[-1][1],commands[-1][3]),(0.,0.))

    def test_duplicate_command_publisher_refuses_all_motion(self):
        f,result,rows=self.run_flow(duplicate=True)
        self.assertEqual(result,1)
        self.assertFalse(f.events)
        self.assertEqual(rows[-1]['reason'],'PREFLIGHT_NOT_READY')


if __name__=='__main__':unittest.main()
