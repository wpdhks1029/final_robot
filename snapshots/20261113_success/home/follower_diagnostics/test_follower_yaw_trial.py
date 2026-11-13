"""Fake ROS/clocks only: verify bounded pivot wiring and stop cleanup."""
import json,math,sys,unittest
from types import SimpleNamespace as S
from unittest.mock import patch
import run_follower_yaw_trial as runner
from follower_yaw_diagnostics import synchronized_summary
from local_avoidance_drive import PROFILE,COMMAND_TOPIC

class Flow:
    def __init__(self,ratio=1.3,stale_imu=False):
        self.t=1.;self.raw=0.;self.imu=0.;self.w=0.;self.ratio=ratio;self.armed=False;self.arm_ns=None;self.zero=5
        self.publishers=[];self.commands=[];self.services=[];self.summary=None
        self.stale_imu=stale_imu
        self.watch=S(streams={},latest={},motor=None,scan=None,mount=object(),events=[],rejections={})
    def ros(self):return 100_000_000_000+round(self.t*1e9)
    def get_name(self):return 'follower_yaw_trial_operator'
    def get_clock(self):return S(now=lambda:S(nanoseconds=self.ros()))
    def create_publisher(self,kind,topic,qos):
        self.publishers.append(topic)
        def publish(m):
            obj=json.loads(m.data);self.commands.append(obj)
            self.w=obj['w'] if self.armed else 0.
            if self.w:self.zero=0
        return S(publish=publish)
    def create_client(self,kind,name):
        def call(req):
            self.services.append(name);self.armed=name.endswith('/arm')
            if self.armed:self.arm_ns=self.ros()
            else:self.w=0.
            return S(done=lambda:True,result=lambda:S(success=True,message=json.dumps(dict(epoch='1234567890abcdef'))))
        return S(service_is_ready=lambda:True,call_async=call)
    def get_publishers_info_by_topic(self,topic):return [S(node_name=self.get_name())] if self.publishers else []
    def get_subscriptions_info_by_topic(self,topic):return [S(node_name='follower_drive_candidate')]
    def destroy_node(self):pass
    def spin(self,node,timeout_sec):
        self.t+=.02;self.raw+=self.w*.02;self.imu+=self.w*.02*self.ratio
        if not self.w:self.zero+=1
        for k,y in [('raw',self.raw),('imu',self.imu),('filtered',self.imu),('map',self.imu)]:
            row=dict(stamp_ns=self.ros(),rx_mono=self.t,yaw=y,roll=0.,pitch=0.,x=0.,y=0.,v=0.,w=self.w)
            if k=='imu' and self.stale_imu and self.armed:row['rx_mono']-=.9
            self.watch.latest[k]=row;self.watch.streams.setdefault(k,[]).append(row)
        stamp=S(sec=self.ros()//10**9,nanosec=self.ros()%10**9)
        self.watch.scan=(S(header=S(stamp=stamp,frame_id='follower/laser'),ranges=[2.]*360,angle_min=-math.pi,angle_max=math.pi),self.t)
        self.watch.motor=(dict(mode='HARDWARE_CANDIDATE',input_kind='local_avoidance',motion_profile=PROFILE,
            zero_stop_prearm_supported=True,zero_stop_response_retry_supported=True,zero_response_retry_version=2,zero_reply_hold_supported=True,input_source_timeout_s=.8,trial_max_rpm=10,trial_max_seconds=360,
            enabled=True,fault=None,io_failed=False,feedback_fresh=True,armed=self.armed,
            measured_rpm=[5,5] if self.w else [0,0],zero_feedback_samples=self.zero,
            epoch='1234567890abcdef',arm_ros_ns=self.arm_ns),self.t)
    def modules(self):
        return {'rclpy':S(init=lambda **kw:None,create_node=lambda n:self,spin_once=self.spin,ok=lambda:True,shutdown=lambda:None),
                'rclpy.signals':S(SignalHandlerOptions=S(NO=0)),
                'rclpy.qos':S(QoSProfile=lambda **kw:S(**kw),ReliabilityPolicy=S(RELIABLE='R')),
                'std_msgs':S(),'std_msgs.msg':S(String=lambda **kw:S(**kw)),
                'std_srvs':S(),'std_srvs.srv':S(Trigger=S(Request=lambda:S())),
                'watch_apf_case3':S(project=lambda *args:[(2*math.cos(i*math.pi/90),2*math.sin(i*math.pi/90)) for i in range(180)])}

class Tests(unittest.TestCase):
    def run_flow(self,plan_only=False,ratio=1.3,stale_imu=False,angle_source='raw',target=20):
        f=Flow(ratio,stale_imu)
        def save(watch,kind,extra):f.summary=dict(synchronized_summary(watch.streams,begin_ns=extra.get('measurement_begin_ns')),**extra)
        args=['yaw','--plan-only' if plan_only else '--run','--ack-clear-pivot-area-power-cut-ready','--target-deg',str(target),'--angle-source',angle_source]
        with patch.dict(sys.modules,f.modules()),patch.dict('os.environ',{'ROS_DOMAIN_ID':'10'}),\
             patch('sys.argv',args),patch.object(runner,'attach',return_value=f.watch),patch.object(runner,'save',side_effect=save),\
             patch.object(runner.time,'monotonic',side_effect=lambda:f.t),\
             patch('leader_short_forward_trial.wait_for_start',return_value=True):
            code=runner.main()
        return f,code
    def test_nominal_raw20_vs_imu26_and_no_forward_or_leader_command(self):
        f,code=self.run_flow();self.assertEqual(code,0);self.assertTrue(f.summary['follower_zero_confirmed'])
        self.assertEqual(f.publishers,[COMMAND_TOPIC]);self.assertTrue(all(c['v']==0 for c in f.commands))
        self.assertAlmostEqual(f.summary['imu_over_raw_ratio'],1.3)
        self.assertFalse(f.summary['automatic_calibration_applied'])
        self.assertTrue(all(c['motion']=='STOP' or min(c['source_stamp_ns'],c['scan_stamp_ns'])>=f.arm_ns for c in f.commands))
    def test_imu_limit_stops_a_large_wheel_heading_underestimate(self):
        f,code=self.run_flow(ratio=3.)
        self.assertEqual(code,1);self.assertEqual(f.summary['reason'],'STOP_IMU_45_DEG_LIMIT')
        self.assertTrue(f.summary['follower_zero_confirmed']);self.assertFalse(f.armed)
    def test_plan_only_has_no_motor_publisher_or_arm_service(self):
        f,code=self.run_flow(plan_only=True);self.assertEqual(code,0)
        self.assertEqual(f.publishers,[]);self.assertEqual(f.services,[]);self.assertEqual(f.commands,[])
    def test_stale_imu_cancels_before_nonzero_and_confirms_stop(self):
        f,code=self.run_flow(stale_imu=True)
        self.assertEqual(code,1);self.assertEqual(f.summary['reason'],'IMU_MISSING_OR_STALE')
        self.assertFalse(any(c['w'] for c in f.commands));self.assertTrue(f.summary['follower_zero_confirmed'])
    def test_imu_target_stops_at_thirty_without_changing_raw_geometry(self):
        for target in (30,-30):
            f,code=self.run_flow(angle_source='imu',target=target)
            self.assertEqual(code,0);self.assertEqual(f.summary['reason'],'IMU_TARGET_REACHED')
            self.assertAlmostEqual(f.summary['sources']['imu']['delta_deg'],target,delta=.3)
            self.assertAlmostEqual(f.summary['sources']['raw']['delta_deg'],target/1.3,delta=.3)
            self.assertIsNone(f.summary['target_raw_deg']);self.assertTrue(f.summary['follower_zero_confirmed'])

if __name__=='__main__':unittest.main()
