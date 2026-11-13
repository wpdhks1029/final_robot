"""Execute real operator wait/resume flow with fake ROS and clocks only."""
import contextlib,io,json,sys,tempfile,unittest
from pathlib import Path
from types import SimpleNamespace as S
from unittest.mock import patch
ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'candidate/follower_diagnostics'))
import run_local_avoidance_trial as runner
from test_local_avoidance_operator import Flow
from evaluation3_clearance_policy import PROFILE as POLICY
from directed_path_tracker import DirectedPathTracker

class OperatorTests(unittest.TestCase):
    def flow(self,loss_end=35.,local_fault=False,alignment_end=None):
        class WaitFlow(Flow):
            def state(self):
                return dict(super().state(),continuous20=True,continuous_stop_clearance_m=.05,
                    continuous_clearance_profile=POLICY,reference_wait_supported=True,trial_max_seconds=0)
            def create_publisher(self,kind,topic,qos):
                if topic=='/follower/motor_candidate/continuous20_command':topic=runner.COMMAND_TOPIC
                return super().create_publisher(kind,topic,qos)
            def create_subscription(self,kind,topic,callback,qos):
                original=callback
                if topic in ('/leader/odom','/leader/path'):
                    def callback(m):
                        if self.armed and 10.<=self.t<loss_end:
                            ns=self.ros()-2_000_000_000;m.header.stamp=S(sec=ns//10**9,nanosec=ns%10**9)
                        original(m)
                return super().create_subscription(kind,topic,callback,qos)
            def get_publishers_info_by_topic(self,topic):
                return [S(node_name=self.get_name() if topic in ('/cmd_vel','/follower/motor_candidate/continuous20_command') else 'source')]
            def get_subscriptions_info_by_topic(self,topic):
                return [S(node_name='md200t_controller' if topic=='/cmd_vel' else 'follower_drive_candidate')]
        f=WaitFlow();modules,Base=f.modules();Nav=modules['follower_pursuit_preview'].PreviewCore
        class Tracked(Nav):
            def __init__(self,lookahead=.35):super().__init__();self.path_tracker=DirectedPathTracker();self.last_selection=None
        class Continuous(Base):
            def scene(self,*args,**kw):
                if local_fault and f.armed and f.t>15.:raise ValueError('IMU_STALE')
                if alignment_end is not None and f.armed and 10.<=f.t<alignment_end:
                    self.last_alignment_failure=dict(kind='POSE_HISTORY_GAP')
                    raise ValueError('FILTERED_ODOM_SCAN_BRACKET_MISSING')
                return super().scene(*args)
            def command(self,*args,**kw):
                row=super().command(*args,**kw)
                row.update(reason='CONTINUOUS_READY',v=min(.025,row['v']))
                return row
        modules['tracked_pursuit_candidate']=S(TrackedPreviewCore=Tracked)
        modules['continuous_local_core']=S(ContinuousCore=Continuous,build_plan=lambda *a:None)
        modules['rclpy'].ok=lambda:f.t<50. or not f.armed
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);ref=root/'reference.json';ref.write_text('{}')
            args=['eval3','--run','--ack-clear-both-power-cut-ready','--continuous20','--dynamic-box',
                  '--apf-pivot','--gentle-pivot','--map-rejoin','--continue-follow-after-rejoin',
                  '--pose-source','filtered','--reference-plan',str(ref),'--seconds','0','--reference-wait']
            with patch.dict(sys.modules,modules),patch.dict('os.environ',{'ROS_DOMAIN_ID':'10'}),patch('sys.argv',args),\
                 patch.object(runner,'ROOT',root),patch.object(runner.time,'monotonic',side_effect=lambda:f.t),\
                 patch.object(runner,'reference_from_plan',return_value=((0.,0.),0.,0.,0.)),\
                 patch.object(runner,'wait_for_start',return_value=True),contextlib.redirect_stdout(io.StringIO()):
                runner.main()
            rows=[json.loads(l) for l in next((root/'logs').glob('*.jsonl')).read_text().splitlines()]
        return f,rows
    def test_twenty_five_second_loss_waits_zero_then_same_arm_resumes(self):
        f,rows=self.flow()
        start=next(r for r in rows if r['event']=='reference_wait_started')
        resumed=next(r for r in rows if r['event']=='reference_wait_resumed')
        self.assertGreater(resumed['waited_s'],25.)
        held=[e[1] for e in f.events if e[0]=='envelope' and start['ros_ns']<=e[1]['stamp_ns']<resumed['ros_ns']]
        self.assertTrue(held);self.assertTrue(all(r['motion']=='STOP' and r['v']==r['w']==0 for r in held))
        self.assertTrue(any(e[0]=='envelope' and e[1]['stamp_ns']>resumed['ros_ns'] and e[1]['v']>0 for e in f.events))
        self.assertEqual(sum(e[0]=='arm' for e in f.events),1)
        self.assertEqual(rows[-1]['reason'],'ROS_SHUTDOWN')
        self.assertTrue(rows[-1]['leader_zero_confirmed']);self.assertTrue(rows[-1]['follower_zero_confirmed'])
    def test_permanent_loss_never_times_out_before_operator_shutdown(self):
        f,rows=self.flow(loss_end=1000.)
        self.assertEqual(rows[-1]['reason'],'ROS_SHUTDOWN')
        self.assertFalse(any(r['event']=='reference_wait_resumed' for r in rows))
        self.assertEqual(sum(e[0]=='arm' for e in f.events),1)
        self.assertTrue(rows[-1]['follower_zero_confirmed'])
    def test_local_imu_fault_during_wait_still_exits(self):
        f,rows=self.flow(local_fault=True)
        self.assertEqual(rows[-1]['reason'],'IMU_STALE')
        self.assertTrue(rows[-1]['follower_zero_confirmed'])

    def test_long_scan_alignment_gap_waits_then_resumes_same_arm(self):
        f,rows=self.flow(loss_end=0.,alignment_end=20.)
        start=next(r for r in rows if r['event']=='scan_alignment_wait')
        resumed=next(r for r in rows if r['event']=='scan_alignment_recovered')
        self.assertGreater(resumed['waited_s'],10.)
        held=[e[1] for e in f.events if e[0]=='envelope' and start['ros_ns']<=e[1]['stamp_ns']<resumed['ros_ns']]
        self.assertTrue(held);self.assertTrue(all(r['motion']=='STOP' and r['v']==r['w']==0 for r in held))
        self.assertTrue(any(e[0]=='envelope' and e[1]['stamp_ns']>resumed['ros_ns'] and e[1]['v']>0 for e in f.events))
        self.assertEqual(sum(e[0]=='arm' for e in f.events),1)
        self.assertEqual(rows[-1]['reason'],'ROS_SHUTDOWN')
    def test_remote_loss_and_alignment_gap_can_overlap(self):
        f,rows=self.flow(loss_end=25.,alignment_end=30.)
        self.assertEqual(rows[-1]['reason'],'ROS_SHUTDOWN')
        self.assertTrue(any(r['event']=='scan_alignment_recovered' for r in rows))
        self.assertEqual(sum(e[0]=='arm' for e in f.events),1)

    def test_permanent_alignment_gap_has_no_wait_timeout(self):
        f,rows=self.flow(loss_end=0.,alignment_end=1000.)
        self.assertEqual(rows[-1]['reason'],'ROS_SHUTDOWN')
        self.assertFalse(any(r['event']=='scan_alignment_recovered' for r in rows))
        self.assertTrue(rows[-1]['follower_zero_confirmed'])
    def test_alignment_wait_does_not_mask_local_sensor_fault(self):
        f,rows=self.flow(loss_end=0.,alignment_end=1000.,local_fault=True)
        self.assertEqual(rows[-1]['reason'],'IMU_STALE')
        self.assertTrue(rows[-1]['follower_zero_confirmed'])

if __name__=='__main__':unittest.main()
