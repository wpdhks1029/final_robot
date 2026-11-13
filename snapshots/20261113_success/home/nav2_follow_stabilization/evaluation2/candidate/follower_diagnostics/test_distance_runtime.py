"""Fake ROS only: execute the real distance operator without opening hardware."""
import contextlib
import io
import json
import math
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace as S
import unittest
from unittest.mock import patch
import run_leader_distance_trial as runner


def twist():return S(linear=S(x=0.,y=0.,z=0.),angular=S(x=0.,y=0.,z=0.))


class Flow:
    def __init__(self,duplicate=False,stale=False,near=False,interrupt=False,body_echo=False,only_body=False):
        self.t=1.;self.x=0.;self.v=0.;self.commands=[];self.callbacks={}
        self.duplicate=duplicate;self.stale=stale;self.near=near;self.interrupt=interrupt
        self.topics=[];self.body_echo=body_echo;self.only_body=only_body
    def ns(self):return 100_000_000_000+round(self.t*1e9)
    def get_clock(self):return S(now=lambda:S(nanoseconds=self.ns()))
    def get_name(self):return 'leader_distance_trial_operator'
    def create_publisher(self,kind,topic,qos):
        self.topics.append(topic)
        def publish(m):self.commands.append(m.linear.x);self.v=m.linear.x
        return S(publish=publish)
    def create_subscription(self,kind,topic,callback,qos):self.callbacks[topic]=callback;return S()
    def get_publishers_info_by_topic(self,topic):
        names=([self.get_name(),'teleop'] if self.duplicate else [self.get_name()]) if topic=='/cmd_vel' else ['sensor']
        return [S(node_name=n) for n in names]
    def get_subscriptions_info_by_topic(self,topic):return [S(node_name='md200t_controller')]
    def destroy_node(self):pass
    def spin(self,node,timeout_sec):
        self.t+=.05;self.x+=self.v*.05
        if self.interrupt and abs(self.x)>.03:
            self.interrupt=False;raise KeyboardInterrupt
        ns=self.ns()
        if self.stale and abs(self.x)>.03:ns-=1_000_000_000
        stamp=S(sec=ns//10**9,nanosec=ns%10**9)
        q=S(x=0.,y=0.,z=0.,w=1.)
        for topic,cb in self.callbacks.items():
            if topic=='/scan':
                ranges=[.2 if self.only_body else (.4 if self.near else 5.)]*360
                if self.body_echo:ranges[180]=.2
                cb(S(header=S(stamp=stamp,frame_id='laser'),ranges=ranges,
                     range_min=.05,range_max=10.,angle_min=-math.pi,angle_max=math.pi,
                     angle_increment=2*math.pi/359))
            elif topic=='/imu/data':cb(S(header=S(stamp=stamp,frame_id='imu'),orientation=q,orientation_covariance=[.01]*9))
            else:
                t=twist();t.linear.x=self.v
                cb(S(header=S(stamp=stamp,frame_id='map' if topic=='/leader/odom' else 'odom'),
                     child_frame_id='base_link',pose=S(pose=S(position=S(x=self.x,y=0.),orientation=q)),twist=S(twist=t)))
    def modules(self):
        tf=S(transform=S(translation=S(x=0.,y=0.,z=0.),rotation=S(x=0.,y=0.,z=0.,w=1.)))
        return {'rclpy':S(init=lambda **kw:None,create_node=lambda name:self,spin_once=self.spin,shutdown=lambda:None,
                          time=S(Time=S(from_msg=lambda m:None))),
                'rclpy.signals':S(SignalHandlerOptions=S(NO=0)),
                'rclpy.qos':S(QoSProfile=lambda **kw:None,ReliabilityPolicy=S(BEST_EFFORT=0)),
                'geometry_msgs.msg':S(Twist=twist),'nav_msgs.msg':S(Odometry=object),
                'sensor_msgs.msg':S(Imu=object,LaserScan=object),
                'tf2_ros':S(Buffer=lambda:S(lookup_transform=lambda *a:tf),TransformListener=lambda *a:None)}


class DistanceTests(unittest.TestCase):
    def run_flow(self,direction='forward',scale=1.,speed=.1,**kw):
        f=Flow(**kw)
        with tempfile.TemporaryDirectory() as d:
            fake_file=Path(d)/'run_leader_distance_trial.py'
            with patch.dict(sys.modules,f.modules()),patch.dict('os.environ',{'ROS_DOMAIN_ID':'10'}),\
                 patch('sys.argv',['distance','--run','--ack-clear-path-power-cut-ready','--direction',direction,'--distance-scale',str(scale),'--speed',str(speed)]),\
                 patch.object(runner,'__file__',str(fake_file)),patch.object(runner,'wait_for_start',return_value=True),\
                 patch.object(runner.time,'monotonic',side_effect=lambda:f.t),contextlib.redirect_stdout(io.StringIO()):
                code=runner.main()
            report=json.loads(next((Path(d)/'logs').glob('*.json')).read_text())['summary']
        return f,code,report
    def test_forward_three_meters_and_stop(self):
        f,code,r=self.run_flow()
        self.assertEqual(code,0);self.assertTrue(r['leader_zero_confirmed'])
        self.assertGreaterEqual(f.x,3.);self.assertLess(f.x,3.01)
        self.assertTrue(all(0<=v<=.1 for v in f.commands));self.assertEqual(f.commands[-1],0.)
        self.assertEqual(f.topics,['/cmd_vel']);self.assertFalse(r['follower_commanded'])
    def test_forward_measurement_compensation_does_not_change_command_speed(self):
        scale=2.8/3.0060330220273124
        f,code,r=self.run_flow(scale=scale)
        self.assertEqual(code,0)
        self.assertGreaterEqual(f.x*scale,3.)
        self.assertLess(f.x*scale,3.01)
        self.assertAlmostEqual(r['raw_odom_target_m'],3.2207496664578357)
        self.assertTrue(all(0<=v<=.1 for v in f.commands))
        self.assertTrue(r['leader_zero_confirmed'])
    def test_additional_forward_ten_cm_calibration(self):
        previous=.93146016011216
        scale=previous*2.9/3.
        f,code,r=self.run_flow(scale=scale,body_echo=True)
        self.assertEqual(code,0);self.assertTrue(r['leader_zero_confirmed'])
        self.assertGreaterEqual(f.x*scale,3.)
        self.assertLess(f.x*scale,3.01)
        self.assertAlmostEqual(r['raw_odom_target_m']/(3/previous),3/2.9)
        self.assertLessEqual(max(f.commands),.1)
    def test_reduce_forward_after_reported_310cm_travel(self):
        previous=.900411488108421
        scale=previous*3.1/3.
        f,code,r=self.run_flow(scale=scale,body_echo=True)
        self.assertEqual(code,0);self.assertTrue(r['leader_zero_confirmed'])
        self.assertGreaterEqual(f.x*scale,3.)
        self.assertLess(f.x*scale,3.01)
        self.assertAlmostEqual(r['raw_odom_target_m']/(3/previous),3/3.1)
        self.assertLessEqual(max(f.commands),.1)
    def test_reverse_three_meters_and_stop(self):
        f,code,r=self.run_flow('reverse')
        self.assertEqual(code,0);self.assertTrue(r['leader_zero_confirmed'])
        self.assertLessEqual(f.x,-3.);self.assertGreater(f.x,-3.01)
        self.assertTrue(all(-.1<=v<=0 for v in f.commands));self.assertEqual(f.commands[-1],0.)
    def test_speed030_forward_and_reverse_reach_and_stop(self):
        for direction,scale in [('forward',2.8/3.0060330220273124),('reverse',1.)]:
            with self.subTest(direction=direction):
                f,code,r=self.run_flow(direction,scale=scale,speed=.3,body_echo=True)
                self.assertEqual(code,0);self.assertTrue(r['leader_zero_confirmed'])
                velocities=[abs(v) for v in f.commands]
                self.assertAlmostEqual(max(velocities),.3)
                self.assertLessEqual(velocities[0],.011)
                self.assertTrue(all(b-a<=.010001 for a,b in zip(velocities,velocities[1:])))
                sign=1 if direction=='forward' else -1
                self.assertGreaterEqual(sign*f.x*scale,3.)
                self.assertLess(sign*f.x*scale,3.02)
                self.assertEqual(f.commands[-1],0.)
    def test_reverse050_ramps_reaches_target_and_stops(self):
        f,code,r=self.run_flow('reverse',speed=.5,body_echo=True)
        self.assertEqual(code,0);self.assertTrue(r['leader_zero_confirmed'])
        velocities=[abs(v) for v in f.commands]
        self.assertAlmostEqual(max(velocities),.5)
        self.assertTrue(all(v<=0. for v in f.commands))
        self.assertTrue(all(b-a<=.010001 for a,b in zip(velocities,velocities[1:])))
        self.assertLessEqual(f.x,-3.);self.assertGreater(f.x,-3.02)
        self.assertEqual(f.commands[-1],0.)
    def test_forward050_is_rejected_before_ros(self):
        with patch('sys.argv',['distance','--run','--speed','.5','--direction','forward']),contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit):runner.main()
    def test_second_command_writer_blocks_before_motion(self):
        f,code,r=self.run_flow(duplicate=True)
        self.assertEqual(code,1);self.assertIn('OTHER_CMD_VEL_PUBLISHER',r['trial_reason']);self.assertFalse(f.commands)
    def test_stale_input_aborts_motion_and_sends_zero(self):
        f,code,r=self.run_flow(stale=True)
        self.assertEqual(code,1);self.assertIn('MISSING_OR_STALE',r['trial_reason'])
        self.assertEqual(f.commands[-1],0.);self.assertLess(f.x,1.)
    def test_near_scan_still_blocks_default_distance_test(self):
        f,code,r=self.run_flow(near=True)
        self.assertEqual(code,1);self.assertIn('NOT_CLEAR',r['trial_reason']);self.assertFalse(f.commands)
    def test_body_echo_does_not_stop_calibrated_forward(self):
        f,code,r=self.run_flow(scale=2.8/3.0060330220273124,body_echo=True)
        self.assertEqual(code,0);self.assertTrue(r['exclude_body_internal_scan'])
        self.assertEqual(r['last_scan_evidence']['excluded_body_internal_count'],1)
        self.assertGreaterEqual(f.x*r['distance_scale'],3.)
        self.assertTrue(r['leader_zero_confirmed'])
    def test_body_echo_does_not_stop_reverse(self):
        f,code,r=self.run_flow('reverse',body_echo=True)
        self.assertEqual(code,0);self.assertLessEqual(f.x,-3.)
        self.assertTrue(r['leader_zero_confirmed'])
    def test_only_internal_points_cannot_prove_free_space(self):
        f,code,r=self.run_flow(only_body=True)
        self.assertEqual(code,1);self.assertIn('SCAN_POINTS_INSUFFICIENT',r['trial_reason'])
        self.assertFalse(f.commands)
    def test_exact_body_boundary_has_no_extra_mask_margin(self):
        self.assertTrue(runner.inside_body(.27,-.30))
        self.assertFalse(runner.inside_body(.270001,0.))
        self.assertFalse(runner.inside_body(0.,-.300001))
    def test_logged_echo_is_excluded_after_mount_transform(self):
        # Reproduce the user's rear-facing laser mounting and actual range.
        angle=math.radians(85.34741408556633)
        m=S(ranges=[.2549999952316284],range_min=.05,range_max=10.,angle_min=angle,angle_increment=.01)
        tf=S(transform=S(translation=S(x=.26,y=0.,z=.24),rotation=S(x=0.,y=0.,z=1.,w=0.)))
        e=runner.scan_evidence(m,tf)
        self.assertEqual(e['excluded_body_internal_count'],1)
        self.assertEqual(e['inside_count'],0)
        self.assertEqual(e['external_valid_count'],0)
        self.assertAlmostEqual(e['excluded_body_internal_points'][0]['base_x_m'],.23931599887146987)
    def test_interrupt_sends_zero(self):
        f,code,r=self.run_flow(interrupt=True)
        self.assertEqual(code,1);self.assertEqual(r['trial_reason'],'INTERRUPTED');self.assertTrue(r['leader_zero_confirmed'])
        self.assertEqual(f.commands[-1],0.)
    def test_heading_projection_and_reverse(self):
        self.assertAlmostEqual(runner.directed_progress(0,3,math.pi/2,'forward')[0],3.)
        self.assertAlmostEqual(runner.directed_progress(0,-3,math.pi/2,'reverse')[0],3.)
        self.assertEqual(runner.directed_progress(-1,.1,0,'forward'),(-1.,.1))
    def test_missing_run_does_not_import_ros(self):
        with patch('sys.argv',['distance']),contextlib.redirect_stdout(io.StringIO()):self.assertEqual(runner.main(),0)


if __name__=='__main__':unittest.main()
