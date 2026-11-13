import json,sys,unittest
from types import SimpleNamespace as S
from unittest.mock import patch
import run_follower_distance_trial as runner
from test_follower_yaw_trial import Flow
from run_follower_distance_trial import COMMAND_TOPIC

class DistanceFlow(Flow):
    def __init__(self,stale=False):super().__init__(stale_imu=stale);self.v=0.;self.x=0.
    def get_name(self):return 'follower_distance_trial_operator'
    def create_publisher(self,kind,topic,qos):
        self.publishers.append(topic)
        def publish(m):
            obj=json.loads(m.data);self.commands.append(obj);self.v=obj['v'] if self.armed else 0.
            if self.v:self.zero=0
        return S(publish=publish)
    def spin(self,node,timeout_sec):
        if not self.armed:self.v=0.
        self.x+=self.v*.02;super().spin(node,timeout_sec)
        for key,row in self.watch.latest.items():
            if key!='imu':row.update(x=self.x,v=self.v)
        if self.v:self.zero=0
        self.watch.motor[0].update(distance_test_only=True,measured_rpm=[6,6] if self.v else [0,0],zero_feedback_samples=self.zero)

class Tests(unittest.TestCase):
    def flow(self,plan=False,stale=False):
        f=DistanceFlow(stale)
        def save(w,kind,extra):f.summary=extra
        with patch.dict(sys.modules,f.modules()),patch.dict('os.environ',{'ROS_DOMAIN_ID':'10'}),patch('sys.argv',['distance','--plan-only' if plan else '--run','--ack-clear-path-power-cut-ready']),patch.object(runner,'attach',return_value=f.watch),patch.object(runner,'save',side_effect=save),patch.object(runner.time,'monotonic',side_effect=lambda:f.t),patch('leader_short_forward_trial.wait_for_start',return_value=True):
            code=runner.main()
        return f,code
    def test_one_metre_only_follower_forward_and_confirmed_disarm(self):
        f,code=self.flow();self.assertEqual(code,0);self.assertEqual(f.publishers,[COMMAND_TOPIC])
        self.assertTrue(f.summary['trial_success']);self.assertTrue(f.summary['follower_zero_confirmed'])
        self.assertFalse(f.summary['leader_commanded']);self.assertTrue(all(c['w']==0 and 0<=c['v']<=.04 for c in f.commands))
        self.assertAlmostEqual(f.summary['distance_sources']['raw']['forward_m'],1.,delta=.005)
    def test_plan_only_no_motion_or_arm(self):
        f,code=self.flow(plan=True);self.assertEqual(code,0);self.assertEqual(f.publishers,[]);self.assertEqual(f.services,[])
    def test_imu_loss_stops_and_disarms(self):
        f,code=self.flow(stale=True);self.assertEqual(code,1);self.assertFalse(f.armed);self.assertTrue(f.summary['follower_zero_confirmed'])
    def test_limits(self):
        for vals in [(0,.16,0,0,1),(0,0,.3,0,1),(-.04,0,0,0,1),(0,0,0,0,45)]:self.assertEqual(runner.distance_request(*vals)[0],0.)

if __name__=='__main__':unittest.main()
