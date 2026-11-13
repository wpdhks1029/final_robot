"""Fake ROS and clocks: no sockets, ports, motor or service traffic."""
import json,sys,tempfile,unittest
from pathlib import Path
from types import SimpleNamespace as S
from unittest.mock import patch
import run_local_avoidance_trial as runner
from local_avoidance_drive import PROFILE,COMMAND_TOPIC
from temporary_detour_candidate import Pose


def twist():return S(linear=S(x=0.,y=0.,z=0.),angular=S(x=0.,y=0.,z=0.))

class Flow:
    def __init__(self,wrong=False,stale=False,inject=False,plan_only=False,delayed_arm_status=False,source_delay=.1,legacy_driver=False,filtered_gap_ticks=0,defer_filtered=False):
        self.delayed_arm_status=delayed_arm_status
        self.legacy_driver=legacy_driver
        self.source_delay_ns=round(source_delay*1e9)
        self.filtered_seen=0;self.frozen_input=None
        self.filtered_gap_ticks=filtered_gap_ticks
        self.defer_filtered=defer_filtered;self.pending_filtered=None
        self.t=1.;self.leader_v=0.;self.armed=False;self.arm_ns=None;self.callbacks={};self.events=[];self.wrong=wrong;self.stale=stale;self.inject=inject;self.plan_only=plan_only;self.moving=False;self.zero_count=5;self.publisher_qos={}
    def ros(self):return 100_000_000_000+round(self.t*1e9)
    def get_clock(self):return S(now=lambda:S(nanoseconds=self.ros()))
    def get_name(self):return 'local_avoidance_trial_operator'
    def create_publisher(self,kind,topic,qos):
        self.publisher_qos[topic]=qos
        def publish(m):
            if topic=='/cmd_vel':self.leader_v=m.linear.x;self.events.append(('leader',self.leader_v,self.t))
            elif topic==COMMAND_TOPIC:
                obj=json.loads(m.data);self.events.append(('envelope',obj,self.t));self.moving=obj['motion']!='STOP'
                if self.moving:self.zero_count=0
            elif topic=='/follower/local_avoidance/status':
                self.events.append(('status',m,self.t))
        return S(publish=publish)
    def create_subscription(self,kind,topic,cb,qos):self.callbacks[topic]=cb;return cb
    def create_client(self,kind,name):
        def call(req):
            self.armed=name.endswith('/arm')
            if not self.armed:self.moving=False
            if self.armed:self.arm_ns=self.ros()
            self.events.append(('arm' if self.armed else 'disarm',self.t))
            return S(done=lambda:True,result=lambda:S(success=True,message=json.dumps(dict(epoch='1234567890abcdef'))))
        return S(service_is_ready=lambda:True,call_async=call)
    def get_publishers_info_by_topic(self,topic):
        if self.plan_only and topic in ('/cmd_vel',COMMAND_TOPIC):return []
        return [S(node_name=self.get_name() if topic in ('/cmd_vel',COMMAND_TOPIC) else 'source')]
    def get_subscriptions_info_by_topic(self,topic):return [S(node_name='md200t_controller' if topic=='/cmd_vel' else 'follower_drive_candidate')]
    def destroy_node(self):pass
    def state(self):return dict(mode='HARDWARE_CANDIDATE',input_kind='guard' if self.wrong else 'local_avoidance',
        motion_profile=PROFILE,input_source_timeout_s=.8,zero_stop_prearm_supported=not self.legacy_driver,zero_stop_response_retry_supported=True,zero_response_retry_version=2,zero_reply_hold_supported=True,trial_max_rpm=10,trial_max_seconds=30,enabled=True,fault=None,io_failed=False,
        feedback_fresh=True,armed=self.armed,measured_rpm=[2,2] if self.armed and self.moving else [0,0],zero_feedback_samples=self.zero_count,
        valid_feedback=round(self.t*100),epoch='1234567890abcdef',arm_ros_ns=self.arm_ns)
    def spin(self,node,timeout_sec):
        if timeout_sec==0:
            if self.pending_filtered is not None:
                self.callbacks['/follower/odometry/filtered'](self.pending_filtered);self.pending_filtered=None
            return
        self.t+=.02
        if not self.moving:self.zero_count=min(5,self.zero_count+1)
        ns=self.ros();h=lambda frame:S(frame_id=frame,stamp=S(sec=ns//10**9,nanosec=ns%10**9))
        for topic,cb in list(self.callbacks.items()):
            if topic=='/navigate_to_pose/_action/status':
                cb(S(status_list=[S(status=2 if self.t<1.4 else 4)]))
            elif topic.endswith('/status'):
                state=self.state()
                if self.delayed_arm_status and self.armed and ns-self.arm_ns<80_000_000:
                    state.update(armed=False,arm_ros_ns=self.arm_ns-10_000_000_000)
                cb(S(data=json.dumps(state)))
            elif topic=='/tf_static':cb(S(transforms=[S(header=h('follower/base_link'),child_frame_id='follower/laser',transform=S(translation=S(x=.26,y=0.,z=.24),rotation=S(x=0.,y=0.,z=0.,w=1.)))]))
            elif topic.endswith('/scan'):cb(S(header=h('follower/laser')))
            elif topic=='/follower/imu/data':cb(S(header=h('follower/imu_link'),orientation=S(x=0.,y=0.,z=0.,w=1.),orientation_covariance=[.05]*9,angular_velocity=S(z=0.)))
            elif topic.endswith('/path') or topic=='/received_global_plan':cb(S(header=h('map'),poses=[]))
            else:
                t=twist();t.linear.x=self.leader_v if topic=='/leader/odom' else 0.
                message=S(header=h('follower/odom' if topic.endswith('odom_raw') or topic=='/follower/odometry/filtered' else 'map'),
                     child_frame_id='base_link' if topic=='/leader/odom' else 'follower/base_link',
                     pose=S(pose=S(position=S(x=2. if topic=='/leader/odom' else (.4 if topic=='/follower/odometry/filtered' else 0.),y=0.),orientation=S(x=0.,y=0.,z=0.,w=1.))),twist=S(twist=t))
                if topic=='/follower/odometry/filtered' and self.defer_filtered:self.pending_filtered=message
                else:cb(message)
    def modules(self):
        flow=self
        class Nav:
            def __init__(self):self.inputs={}
            def odom_callback(self,key,m,ns,rx):self.inputs[key]=(m.header.stamp,rx,(2. if key=='leader' else 0.,0.,0.,flow.leader_v if key=='leader' else 0.,0.))
            def command(self,*args):return (.04,0.,'PREVIEW_TRACK') if flow.leader_v>.03 else (0.,0.,'STOP_LEADER_STOPPED')
            def path_callback(self,m,ns,rx,key='path',persistent=False):self.inputs[key]=(m.header.stamp,rx,((0.,0.),(3.,0.)))
        class Core:
            def __init__(self,pose_source='raw'):self.inputs={};self.scan=None;self.fsm=None;self.pose_source=pose_source;self.last_alignment_failure=None
            def raw(self,stamp,rx,data):self.inputs['raw']=(stamp,rx,data)
            def filtered(self,stamp,rx,data):self.inputs['filtered']=(stamp,rx,data);flow.filtered_seen+=1
            def imu(self,stamp,rx,data):self.inputs['imu']=(stamp,rx,data)
            def offer_scan(self,*args):self.scan=(args[0],args[1],args[2])
            def freeze(self,*args):flow.frozen_input=args[0]
            def scene(self,*args):
                if flow.stale and flow.armed:raise ValueError('RAW_STALE')
                if self.pose_source=='filtered' and flow.armed and flow.filtered_gap_ticks:
                    flow.filtered_gap_ticks-=1
                    self.last_alignment_failure=dict(kind='WAITING_FOR_NEWER_POSE',scan_ahead_of_pose_s=.08)
                    raise ValueError('FILTERED_ODOM_SCAN_BRACKET_MISSING')
                self.last_alignment_failure=None
                cloud=[[.95,y] for y in (-.05,-.03,-.01,.01,.03,.05)] if flow.inject and flow.arm_ns is not None and flow.ros()-flow.arm_ns>600_000_000 else [[2.,0.]]
                return dict(pose=Pose(0.,0.,0.),points=__import__('numpy').array(cloud),scan_stamp=flow.ros(),source=flow.ros()-flow.source_delay_ns)
            def install(self,p):self.fsm=True
            def command(self,*args,**kwargs):
                self.scene()
                follow=kwargs.get('follow_command')
                phase='TURN' if follow is None else ('DRIVE' if follow[0]>0 else 'STOP')
                v,w=(0.,.05) if follow is None else follow
                return dict(reason='LOCAL_AVOIDANCE_READY',motion=phase,v=v,w=w,
                    source_stamp_ns=flow.ros()-flow.source_delay_ns,scan_stamp_ns=flow.ros()-flow.source_delay_ns,
                    clearance_m=.4,swept_clearance_m=.35,geometry_complete=False,pose=[0.,0.,0.])
        class Message:
            def __init__(self,**kw):self.__dict__.update(kw)
        def receiver(callback):
            flow.create_subscription(object,'/follower/motor_candidate/status',callback,1)
            return S(close=lambda:True,error=None)
        def sensor_receiver(node):
            def poll():flow.spin(node,.01);flow.spin(node,0.)
            return S(poll=poll,close=lambda:True)
        modules={'sensor_executor':S(SensorExecutor=sensor_receiver),'motor_status_receiver':S(MotorStatusReceiver=receiver),'rclpy':S(init=lambda **kw:None,create_node=lambda n:flow,spin_once=flow.spin,ok=lambda:True,shutdown=lambda:None),
          'rclpy.signals':S(SignalHandlerOptions=S(NO=0)),
          'rclpy.qos':S(QoSProfile=lambda **kw:S(**kw),ReliabilityPolicy=S(RELIABLE='R',BEST_EFFORT='BE'),DurabilityPolicy=S(TRANSIENT_LOCAL='TL')),
          'geometry_msgs':S(),'geometry_msgs.msg':S(Twist=twist),'nav_msgs':S(),'nav_msgs.msg':S(Odometry=object,Path=object),
          'sensor_msgs':S(),'sensor_msgs.msg':S(LaserScan=object,Imu=object),'std_msgs':S(),'std_msgs.msg':S(String=Message),
          'action_msgs':S(),'action_msgs.msg':S(GoalStatusArray=object),
          'std_srvs':S(),'std_srvs.srv':S(Trigger=S(Request=lambda:S())), 'tf2_msgs':S(),'tf2_msgs.msg':S(TFMessage=object),
          'follower_remote_qos':S(latest_state_qos=lambda:1,measurement_history_qos=lambda:S(depth=10)),'follower_pursuit_preview':S(PreviewCore=Nav),
          'watch_apf_case3':S(project=lambda *args:[(2.,0.)]),'follower_input_guard':S(valid_odom=lambda *args:True),
          'watch_temporary_detour_shadow':S(PreparedPathReview=lambda p:S(review=lambda pts:dict(reason='LIVE_PATH_GEOMETRY_CLEAR')))}
        return modules,Core

class Tests(unittest.TestCase):
    def run_flow(self,plan_only=False,apf_pivot=False,near_return_hold=False,pose_source='raw',**kw):
        f=Flow(plan_only=plan_only,**kw);modules,core=f.modules();plan=dict(path=[[0,0,0,0],[1,0,0,0]],config={},geometry=dict(reason='LIVE_PATH_GEOMETRY_CLEAR'),attempts=[])
        calls=[0]
        def submit(*args):
            calls[0]+=1
            initial=calls[0]==1
            def result():
                if f.inject and initial:raise ValueError('NO_OBSTACLE_CLUSTER')
                return plan
            return S(done=lambda:True,result=result)
        pool=S(submit=submit,shutdown=lambda **kw:None)
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);ref=root/'reference.json';ref.write_text('{}')
            with patch.dict(sys.modules,modules),patch.dict('os.environ',{'ROS_DOMAIN_ID':'10'}),\
                 patch('sys.argv',['local','--plan-only' if plan_only else '--run','--ack-clear-both-power-cut-ready','--reference-plan',str(ref),'--pose-source',pose_source]+(['--apf-pivot'] if apf_pivot else [])+(['--near-return-hold'] if near_return_hold else [])),\
                 patch.object(runner,'ROOT',root),patch.object(runner,'LocalCore',core),\
                 patch.object(runner,'ThreadPoolExecutor',return_value=pool),\
                 patch.object(runner,'reference_from_plan',return_value=((0,0),0.,0.,0.)),\
                 patch.object(runner.time,'monotonic',side_effect=lambda:f.t),patch.object(runner,'wait_for_start',return_value=True):
                result=runner.main()
            rows=[json.loads(x) for x in next((root/'logs').glob('*.jsonl')).read_text().splitlines()]
        return f,result,rows
    def test_nominal_both_stops_and_new_envelope(self):
        f,result,rows=self.run_flow();self.assertEqual(result,0);self.assertEqual(sum(e[0]=='arm' for e in f.events),1)
        self.assertEqual(f.publisher_qos[COMMAND_TOPIC].reliability,'R');self.assertEqual(f.publisher_qos[COMMAND_TOPIC].depth,1)
        self.assertTrue(rows[-1]['motion_feedback_seen']);self.assertTrue(rows[-1]['leader_zero_confirmed']);self.assertTrue(rows[-1]['follower_zero_confirmed'])
        envelopes=[e[1] for e in f.events if e[0]=='envelope'];self.assertTrue(envelopes)
        self.assertTrue(all(e['version']==3 and e['motion'] in ('STOP','TURN') and e['v']==0 for e in envelopes))
        self.assertFalse(rows[-1]['evaluation_pass'])
    def test_retained_previous_arm_timestamp_cannot_release_old_measurements(self):
        f,result,rows=self.run_flow(delayed_arm_status=True)
        self.assertEqual(result,0)
        arm_time=next(e[1] for e in f.events if e[0]=='arm')
        arm_ns=100_000_000_000+round(arm_time*1e9)
        envelopes=[e[1] for e in f.events if e[0]=='envelope']
        self.assertTrue(envelopes)
        self.assertTrue(all(min(e['source_stamp_ns'],e['scan_stamp_ns'])>=arm_ns or
                            (e['motion']=='STOP' and e['v']==e['w']==0) for e in envelopes))
    def test_four_tenths_sensor_support_wait_keeps_zero_then_uses_post_arm_motion(self):
        f,result,rows=self.run_flow(apf_pivot=True,source_delay=.4)
        self.assertEqual(result,0)
        envelopes=[e[1] for e in f.events if e[0]=='envelope']
        arm=next(e[1] for e in f.events if e[0]=='arm');arm_ns=100_000_000_000+round(arm*1e9)
        self.assertGreater(sum(e['motion']=='STOP' for e in envelopes),10)
        self.assertTrue(any(e['motion']=='TURN' for e in envelopes))
        self.assertTrue(all(e['motion']=='STOP' and e['v']==e['w']==0 or
                            min(e['source_stamp_ns'],e['scan_stamp_ns'])>=arm_ns for e in envelopes))
    def test_apf_mode_keeps_existing_bounded_driver_envelope_and_both_stops(self):
        f,result,rows=self.run_flow(apf_pivot=True)
        self.assertEqual(result,0);self.assertEqual(rows[-1]['controller_kind'],'APF_PIVOT_STEP_V1')
        self.assertTrue(rows[-1]['leader_zero_confirmed']);self.assertTrue(rows[-1]['follower_zero_confirmed'])
        envelopes=[e[1] for e in f.events if e[0]=='envelope']
        self.assertTrue(all(e['controller']==PROFILE and e['version']==3 for e in envelopes))
    def test_near_return_hold_is_explicit_and_does_not_change_nominal_trial(self):
        f,result,rows=self.run_flow(apf_pivot=True,near_return_hold=True)
        self.assertEqual(result,0);self.assertTrue(rows[-1]['near_return_hold_enabled'])
        self.assertTrue(rows[-1]['leader_zero_confirmed']);self.assertTrue(rows[-1]['follower_zero_confirmed'])
    def test_filtered_pose_uses_filtered_anchor_and_is_reported(self):
        f,result,rows=self.run_flow(apf_pivot=True,pose_source='filtered')
        self.assertEqual(result,0);self.assertGreater(f.filtered_seen,0)
        self.assertAlmostEqual(f.frozen_input[0],.4)
        self.assertEqual(rows[-1]['pose_source'],'filtered')
    def test_short_scan_ahead_interval_sends_zero_and_recovers_without_rearm(self):
        f,result,rows=self.run_flow(apf_pivot=True,pose_source='filtered',filtered_gap_ticks=3)
        self.assertEqual(result,0)
        self.assertTrue(any(r['event']=='scan_alignment_wait' for r in rows))
        self.assertTrue(any(r['event']=='scan_alignment_recovered' for r in rows))
        self.assertEqual(sum(e[0]=='arm' for e in f.events),1)
        first_wait=next(r['ros_ns'] for r in rows if r['event']=='scan_alignment_wait')
        recovered=next(r['ros_ns'] for r in rows if r['event']=='scan_alignment_recovered')
        commands=[e[1] for e in f.events if e[0]=='envelope' and first_wait<=e[1]['stamp_ns']<recovered]
        self.assertTrue(commands);self.assertTrue(all(c['motion']=='STOP' and c['v']==c['w']==0 for c in commands))
    def test_persistent_filtered_scan_gap_ends_with_both_robots_stopped(self):
        f,result,rows=self.run_flow(apf_pivot=True,pose_source='filtered',filtered_gap_ticks=1000)
        self.assertEqual(result,1);self.assertEqual(rows[-1]['reason'],'FILTERED_SCAN_ALIGNMENT_TIMEOUT')
        self.assertTrue(rows[-1]['leader_zero_confirmed']);self.assertTrue(rows[-1]['follower_zero_confirmed'])
        self.assertFalse(any(e[0]=='envelope' and e[1]['motion']!='STOP' for e in f.events))
    def test_pending_filtered_callback_is_drained_before_pose_computation(self):
        f,result,rows=self.run_flow(apf_pivot=True,pose_source='filtered',defer_filtered=True)
        self.assertEqual(result,0);self.assertGreater(f.filtered_seen,0)
        self.assertTrue(rows[-1]['follower_zero_confirmed'])
    def test_obstacle_inserted_during_follow_brakes_then_switches_to_avoid(self):
        f,result,rows=self.run_flow(inject=True);self.assertEqual(result,0)
        self.assertEqual(sum(r['event']=='obstacle_trigger' for r in rows),1)
        commands=[e[1]['motion'] for e in f.events if e[0]=='envelope']
        first_turn=commands.index('TURN');self.assertIn('DRIVE',commands[:first_turn]);self.assertIn('STOP',commands[:first_turn])
        self.assertTrue(rows[-1]['leader_zero_confirmed']);self.assertTrue(rows[-1]['follower_zero_confirmed'])

    def test_plan_only_saves_input_without_publishers_arm_or_leader_start(self):
        f,result,rows=self.run_flow(plan_only=True);self.assertEqual(result,0)
        self.assertEqual(f.publisher_qos,{});self.assertEqual(f.events,[])
        self.assertTrue(rows[-1]['plan_only']);self.assertTrue(rows[-1]['plan_ready'])
        self.assertFalse(rows[-1]['leader_started']);self.assertFalse(rows[-1]['arm_requested'])
        self.assertTrue(any(r['event']=='planning_input' for r in rows))

    def test_wrong_driver_cannot_start_either_robot(self):
        f,result,rows=self.run_flow(wrong=True);self.assertEqual(result,1)
        self.assertFalse(rows[-1]['leader_started']);self.assertFalse(rows[-1]['arm_requested'])
    def test_old_driver_is_detected_before_arm_or_leader_start(self):
        f,result,rows=self.run_flow(legacy_driver=True)
        self.assertEqual(result,1);self.assertEqual(f.events,[])
        self.assertIn('REQUIRE_ZERO_STOP_HANDSHAKE_DRIVER_RESTART',rows[-1]['reason'])
    def test_input_loss_disarms_and_stops_leader(self):
        f,result,rows=self.run_flow(stale=True);self.assertEqual(result,1)
        self.assertEqual(sum(e[0]=='arm' for e in f.events),1);self.assertEqual(sum(e[0]=='disarm' for e in f.events),1)
        self.assertTrue(rows[-1]['leader_zero_confirmed']);self.assertTrue(rows[-1]['follower_zero_confirmed'])

if __name__=='__main__':unittest.main()
