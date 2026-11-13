import copy,math,unittest
from types import SimpleNamespace as S
from unittest.mock import patch
from follower_temporary_detour_preview import DetourCore,load_plan,PLAN
from run_joint_follow_trial import scene_reason,main
from temporary_detour_candidate import Pose,to_reference

class Tests(unittest.TestCase):
    def core(self,trial_seconds=5):
        data,digest=load_plan(PLAN);c=DetourCore(data,digest,trial_seconds)
        t=S(sec=100,nanosec=0);pose=data['snapshot']['pose'];h=c.heading
        f=(pose['x'],pose['y'],pose['yaw'],0.,0.)
        l=(f[0]+2*math.cos(h),f[1]+2*math.sin(h),h,.04,0.)
        c.nav.inputs={'leader':(t,1.,l),'follower':(t,1.,f),'path':(t,1.,((f[0],f[1]),(l[0],l[1])))}
        c.nav.leader_moving=True
        c.motor_update(dict(mode='HARDWARE_CANDIDATE',input_kind='guard',trial_max_rpm=10,
            trial_max_seconds=trial_seconds,enabled=True,armed=False,epoch='one',fault=None,
            feedback_fresh=True,io_failed=False,measured_rpm=[0,0]),1.)
        body=data['snapshot']['body_points']
        points=to_reference(body,Pose(*[pose[k] for k in ('x','y','yaw')]),c.origin,c.heading)
        c.scan_update(100_000_000_000,1.,points,body)
        return c
    def test_actual_scene_left_command_and_limits(self):
        c=self.core();r=c.evaluate(100_100_000_000,1.1)
        self.assertEqual(r['reason'],'DETOUR_READY',r)
        self.assertGreater(r['output_v'],0);self.assertGreater(r['output_w'],0)
        self.assertGreaterEqual(r['output_v'],abs(r['output_w'])*.28)
        self.assertLessEqual(r['max_wheel_rpm'],10.)
        self.assertEqual(r['source_stamp_ns'],99_950_000_000)
    def test_each_stale_input_stops(self):
        for key in ('leader','path','follower'):
            c=self.core();stamp,rx,data=c.nav.inputs[key]
            c.nav.inputs[key]=(S(sec=99,nanosec=0),rx,data)
            r=c.evaluate(100_100_000_000,1.1)
            self.assertEqual((r['output_v'],r['output_w']),(0.,0.));self.assertIn('STALE',r['reason'])
        c=self.core();c.scan=(99_000_000_000,*c.scan[1:])
        self.assertEqual(c.evaluate(100_100_000_000,1.1)['reason'],'STOP_SCAN_OR_SOURCE_STALE')
    def test_stopped_leader_and_wrong_driver(self):
        c=self.core();c.nav.leader_moving=False
        self.assertEqual(c.evaluate(100_100_000_000,1.1)['reason'],'STOP_LEADER_STOPPED')
        c=self.core();c.motor[0]['trial_max_seconds']=10
        self.assertEqual(c.evaluate(100_100_000_000,1.1)['output_v'],0.)
    def test_all_scan_points_retained(self):
        c=self.core();stamp,rx,pts,d=c.scan;c.scan=(stamp,rx,pts+[(.4,.4)],d)
        r=c.evaluate(100_100_000_000,1.1)
        self.assertEqual(r['reason'],'STOP_LIVE_PATH_BLOCKED');self.assertEqual(r['output_v'],0.)
    def test_initial_mismatch_and_pose_jump(self):
        c=self.core();stamp,rx,f=c.nav.inputs['follower'];c.nav.inputs['follower']=(stamp,rx,(f[0]+.2,*f[1:]))
        self.assertEqual(c.evaluate(100_100_000_000,1.1)['reason'],'STOP_INITIAL_PLAN_POSE_MISMATCH')
        c=self.core();c.evaluate(100_100_000_000,1.1)
        stamp,rx,f=c.nav.inputs['follower'];c.nav.inputs['follower']=(stamp,rx,(f[0]+.2,*f[1:]))
        self.assertEqual(c.evaluate(100_100_000_000,1.1)['reason'],'STOP_LOCALIZATION_JUMP')
    def test_arm_before_ready_and_session_end(self):
        c=self.core();state=copy.deepcopy(c.motor[0]);state['armed']=True;c.motor_update(state,1.1)
        self.assertEqual(c.retired,'STOP_ARMED_BEFORE_PLAN_READY')
        c=self.core();c.evaluate(100_100_000_000,1.1)
        state=copy.deepcopy(c.motor[0]);state['armed']=True;c.motor_update(state,1.1)
        state=copy.deepcopy(state);state['armed']=False;c.motor_update(state,1.2)
        self.assertEqual(c.evaluate(100_100_000_000,1.2)['reason'],'STOP_SESSION_FINISHED')
    def test_thirty_second_deadline_and_driver_match(self):
        c=self.core(30);r=c.evaluate(100_100_000_000,1.1)
        self.assertEqual(r['reason'],'DETOUR_READY');self.assertEqual(r['trial_max_seconds'],30)
        state=copy.deepcopy(c.motor[0]);state['armed']=True;c.motor_update(state,1.1)
        self.assertEqual(c.evaluate(100_100_000_000,30.86)['reason'],'STOP_LOCAL_TRIAL_TIME_LIMIT')
        c=self.core(30);c.motor[0]['trial_max_seconds']=5
        self.assertEqual(c.evaluate(100_100_000_000,1.1)['output_v'],0.)

    def test_local_deadline(self):
        c=self.core();c.evaluate(100_100_000_000,1.1)
        state=copy.deepcopy(c.motor[0]);state['armed']=True;c.motor_update(state,1.1)
        self.assertEqual(c.evaluate(100_100_000_000,5.86)['reason'],'STOP_LOCAL_TRIAL_TIME_LIMIT')
    def test_supervisor_retains_guard_gate_with_detour_steering(self):
        c=S(command=lambda *a:(0.,0.,'STOP_ALIGN_REQUIRED'),speed_gate=lambda *a:(.04,0.,'FORWARD_ALLOWED'))
        g=S(header=S(frame_id='follower/base_link',stamp=S(sec=100,nanosec=0)),
            twist=S(linear=S(x=.01,y=0.,z=0.),angular=S(x=0.,y=0.,z=.02)))
        self.assertEqual(scene_reason(c,(g,1.),100_100_000_000,1.1),'STOP_ALIGN_REQUIRED')
        self.assertEqual(scene_reason(c,(g,1.),100_100_000_000,1.1,temporary_detour=True),'READY')
        g.twist.linear.x=0
        self.assertEqual(scene_reason(c,(g,1.),100_100_000_000,1.1,temporary_detour=True),'GUARD_STOP_OR_INVALID')
    def test_longer_trial_and_turning_leader_rejected_before_ros(self):
        for extra in (['--trial-seconds','10'],['--leader-turn','left'],['--leader-speed','.05']):
            with patch('sys.argv',['trial','--temporary-detour','--leader-speed','.04']+extra):
                with self.assertRaises(SystemExit):main()
if __name__=='__main__':unittest.main()

class SupervisorFlowTests(unittest.TestCase):
    def flow(self,wrong_identity=False,guard_loss=False,trial_seconds=5,retired=False):
        import json,sys,tempfile
        from pathlib import Path
        from test_joint_operator_flow import Flow
        import run_joint_follow_trial as joint
        from follower_temporary_detour_preview import KIND,NODE,TOPICS,STATUS
        _,digest=load_plan(PLAN)
        class DetourFlow(Flow):
            def create_subscription(self,kind,topic,callback,qos):
                if topic==STATUS:
                    original=callback
                    def callback(m):
                        original(S(data=json.dumps(dict(controller_kind='WRONG' if wrong_identity else KIND,
                            plan_sha256=digest,trial_max_seconds=trial_seconds,stamp_ns=self.ros(),
                            reason='STOP_SESSION_FINISHED' if retired else ('DETOUR_READY' if self.leader_v>0 else 'STOP_LEADER_STOPPED')))))
                return super().create_subscription(kind,topic,callback,qos)
            def get_publishers_info_by_topic(self,topic):
                if topic in TOPICS+(STATUS,):return [S(node_name=NODE)]
                return super().get_publishers_info_by_topic(topic)
            def modules(self):
                modules=super().modules();cls=modules['follower_pursuit_preview'].PreviewCore
                cls.speed_gate=cls.command
                return modules
        f=DetourFlow(leader_speed=.04,guard_loss=guard_loss,trial_seconds=trial_seconds)
        with tempfile.TemporaryDirectory() as tmp:
            with patch.dict(sys.modules,f.modules()),patch.dict('os.environ',{'ROS_DOMAIN_ID':'10'}),\
                patch('sys.argv',['joint','--run','--ack-clear-both-power-cut-ready','--temporary-detour',
                                  '--trial-seconds',str(trial_seconds),'--leader-speed','.04']),\
                patch.object(joint,'__file__',tmp+'/operator.py'),\
                patch.object(joint.time,'monotonic',side_effect=lambda:f.now),\
                patch.object(joint,'wait_for_start',side_effect=lambda spin:True):
                result=joint.main()
            rows=[json.loads(x) for x in next((Path(tmp)/'logs').glob('*.jsonl')).read_text().splitlines()]
        return f,result,rows
    def test_five_second_session_stops_both_and_advertises_mode(self):
        f,result,rows=self.flow()
        self.assertEqual(result,0);self.assertEqual(sum(e[0]=='arm' for e in f.events),1)
        self.assertTrue(rows[-1]['leader_zero_confirmed']);self.assertTrue(rows[-1]['follower_zero_confirmed'])
        self.assertFalse(rows[-1]['evaluation_pass'])
    def test_wrong_controller_rejected_without_motion(self):
        f,result,rows=self.flow(wrong_identity=True)
        self.assertEqual(result,1);self.assertFalse(rows[-1]['leader_started'])
        self.assertFalse(any(e[0]=='arm' for e in f.events))
    def test_retired_session_rejected_before_leader_start(self):
        f,result,rows=self.flow(retired=True)
        self.assertEqual(result,1);self.assertEqual(rows[-1]['reason'],'STOP_SESSION_FINISHED')
        self.assertFalse(rows[-1]['leader_started']);self.assertFalse(rows[-1]['arm_requested'])

    def test_guard_loss_stops_without_rearm(self):
        f,result,rows=self.flow(guard_loss=True)
        self.assertEqual(result,1);self.assertEqual(sum(e[0]=='arm' for e in f.events),1)
        self.assertTrue(rows[-1]['leader_zero_confirmed']);self.assertTrue(rows[-1]['follower_zero_confirmed'])

    def test_thirty_second_supervisor_stops_both_at_limit(self):
        f,result,rows=self.flow(trial_seconds=30)
        self.assertEqual(result,0)
        armed=next(e[1] for e in f.events if e[0]=='arm')
        stopped=next(e[1] for e in f.events if e[0]=='disarm')
        self.assertGreaterEqual(stopped-armed,29.5)
        self.assertLess(stopped-armed,29.6)
        self.assertTrue(rows[-1]['leader_zero_confirmed']);self.assertTrue(rows[-1]['follower_zero_confirmed'])

class DeferredScanTests(unittest.TestCase):
    core=Tests.core
    def test_future_tf_wait_preserves_original_checked_scan_time(self):
        from follower_temporary_detour_preview import DeferredScan,AwaitTransform
        c=self.core();queue=DeferredScan(c);old=c.scan
        queue.offer(100_050_000_000,1.05,'scan',100_100_000_000)
        def unavailable(m):raise AwaitTransform('extrapolation into the future')
        queue.retry(100_100_000_000,1.1,unavailable)
        self.assertIs(c.scan,old);self.assertIsNotNone(queue.pending)
        self.assertEqual(c.evaluate(100_100_000_000,1.1)['source_stamp_ns'],99_950_000_000)
        queue.retry(100_150_000_000,1.15,lambda m:(old[2],c.data['snapshot']['body_points']))
        self.assertEqual(c.scan[:2],(100_050_000_000,1.05));self.assertIsNone(queue.pending)
    def test_expired_pending_scan_clears_old_geometry(self):
        from follower_temporary_detour_preview import DeferredScan
        c=self.core();queue=DeferredScan(c)
        queue.offer(100_050_000_000,1.05,'scan',100_100_000_000)
        queue.retry(100_351_000_000,1.351,lambda m:self.fail('Expired input must not be transformed'))
        self.assertIsNone(c.scan);self.assertEqual(queue.detail,'STOP_SCAN_TF_WAIT_EXPIRED')
    def test_invalid_transform_does_not_keep_old_geometry(self):
        from follower_temporary_detour_preview import DeferredScan
        c=self.core();queue=DeferredScan(c);queue.offer(100_050_000_000,1.05,'scan',100_100_000_000)
        def invalid(m):raise ValueError('STOP_MAP_TF_INVALID')
        queue.retry(100_100_000_000,1.1,invalid)
        self.assertIsNone(c.scan);self.assertEqual(queue.detail,'STOP_MAP_TF_INVALID')
    def test_retired_session_is_preflight_error(self):
        from run_joint_follow_trial import detour_preflight_problem
        self.assertEqual(detour_preflight_problem({'reason':'STOP_SESSION_FINISHED'}),'STOP_SESSION_FINISHED')
        self.assertIsNone(detour_preflight_problem({'reason':'STOP_LEADER_STOPPED'}))


class TransformHistoryTests(unittest.TestCase):
    def test_tf_retains_multiple_frames_and_sensor_inputs_remain_latest_only(self):
        import sys
        from follower_temporary_detour_preview import transform_history_qos
        fake=S(QoSProfile=lambda **kw:S(**kw),ReliabilityPolicy=S(BEST_EFFORT='BE'))
        with patch.dict(sys.modules,{'rclpy':S(),'rclpy.qos':fake}):
            qos=transform_history_qos()
        self.assertEqual(qos.depth,100);self.assertEqual(qos.reliability,'BE')
        import ast
        source=__import__('pathlib').Path(__file__).with_name('follower_temporary_detour_preview.py').read_text()
        tree=ast.parse(source)
        listeners=[n for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Name) and n.func.id=='TransformListener']
        self.assertEqual(len(listeners),1)
        value=next(k.value for k in listeners[0].keywords if k.arg=='qos')
        self.assertEqual(value.func.id,'transform_history_qos')
