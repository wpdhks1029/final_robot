import json
import unittest
from types import SimpleNamespace as S
from unittest.mock import Mock,patch

from run_joint_follow_trial import Window,fresh_sample,motor_reason,scene_reason,main,relative_scene,leader_turn_rate
from follower_drive_candidate import BoundedDrivePort,DriveContract
from follower_motor_mock import MemoryTransport
from follower_motor_candidate import velocity_packet
from test_motor_candidate import Clock


class Tests(unittest.TestCase):
    def test_default_no_ros_or_hardware(self):
        with patch('sys.argv',['joint']):self.assertEqual(main(),0)

    def test_run_requires_ack_before_ros(self):
        with patch('sys.argv',['joint','--run']):
            with self.assertRaises(SystemExit):main()

    def test_window_keeps_wait_and_follow_limits(self):
        w=Window();w.start(1.)
        self.assertIsNone(w.expired(4.99))
        self.assertEqual(w.expired(5.),'FORWARD_SCENE_TIMEOUT')
        w.follow(3.)
        self.assertIsNone(w.expired(7.49))
        self.assertEqual(w.expired(7.5),'TRIAL_TIME_LIMIT')
        self.assertEqual(w.expired(9.),'LEADER_TIME_LIMIT')

    def test_ten_second_window_and_matching_driver_required(self):
        w=Window(10);w.start(1.);w.follow(3.)
        self.assertIsNone(w.expired(12.49))
        self.assertEqual(w.expired(12.5),'TRIAL_TIME_LIMIT')
        self.assertEqual(w.expired(15.),'LEADER_TIME_LIMIT')
        s=self.state();s['trial_max_seconds']=5
        self.assertEqual(motor_reason((s,1.),1.1,stationary=True,trial_seconds=10),'DRIVER_TRIAL_SECONDS_MISMATCH')
        s['trial_max_seconds']=10
        self.assertIsNone(motor_reason((s,1.),1.1,stationary=True,trial_seconds=10))

    def test_sixty_second_window_requires_matching_driver(self):
        w=Window(60);w.start(1.);w.follow(3.)
        self.assertIsNone(w.expired(62.49))
        self.assertEqual(w.expired(62.5),'TRIAL_TIME_LIMIT')
        self.assertEqual(w.expired(65.),'LEADER_TIME_LIMIT')
        state=self.state();state['trial_max_seconds']=30
        self.assertEqual(motor_reason((state,1.),1.1,stationary=True,trial_seconds=60),'DRIVER_TRIAL_SECONDS_MISMATCH')
        state['trial_max_seconds']=60
        self.assertIsNone(motor_reason((state,1.),1.1,stationary=True,trial_seconds=60))

    def test_sixty_second_hard_stop_and_independent_port_deadline(self):
        c=Clock();transport=MemoryTransport();port=BoundedDrivePort(transport,c,max_rpm=10)
        k=DriveContract(port,enabled=True,clock=c,max_rpm=10,trial_seconds=60)
        for _ in range(70):k.tick(c.ros);c.now+=.01
        self.assertIsNotNone(k.arm(c.ros));deadline=k.deadline;c.now+=.01
        while c.now<deadline-.02:
            self.assertTrue(k.command(json.dumps(dict(version=1,epoch=k.core.epoch,frame='follower/base_link',stamp_ns=c.ros,v=.1,w=0.)),c.ros))
            k.tick(c.ros);c.now+=.01
        self.assertTrue(k.core.armed);self.assertEqual(transport.rpm,(-10,10))
        c.now=deadline
        with self.assertRaises(ValueError):port.write(velocity_packet(-10,10))
        k.tick(c.ros)
        for _ in range(30):c.now+=.01;k.tick(c.ros)
        self.assertEqual(k.core.fault,'SIXTY_SECOND_TRIAL_EXPIRED')
        self.assertEqual(transport.rpm,(0,0));self.assertIsNone(k.arm(c.ros))

    def test_command_loss_at_45s_still_latches_in_sixty_second_mode(self):
        c=Clock();transport=MemoryTransport();port=BoundedDrivePort(transport,c,max_rpm=10)
        k=DriveContract(port,enabled=True,clock=c,max_rpm=10,trial_seconds=60)
        for _ in range(70):k.tick(c.ros);c.now+=.01
        self.assertIsNotNone(k.arm(c.ros));start=c.now;c.now+=.01
        while c.now-start<45.:
            self.assertTrue(k.command(json.dumps(dict(version=1,epoch=k.core.epoch,frame='follower/base_link',stamp_ns=c.ros,v=.1,w=0.)),c.ros))
            k.tick(c.ros);c.now+=.01
        for _ in range(50):k.tick(c.ros);c.now+=.01
        self.assertEqual(k.core.fault,'COMMAND_TIMEOUT');self.assertFalse(k.core.armed)
        self.assertEqual(transport.rpm,(0,0));self.assertIsNone(k.arm(c.ros))

    def test_relative_scene_retains_actual_gap_and_lateral_error(self):
        c=S(inputs={'leader':(None,None,(3.,.2,0.,.05,0.)),
                    'follower':(None,None,(1.,0.,0.,.04,0.))})
        r=relative_scene(c)
        self.assertAlmostEqual(r['ahead_m'],2.);self.assertAlmostEqual(r['left_m'],.2)
        self.assertAlmostEqual(r['distance_m'],(4.04)**.5)
        self.assertIsNone(relative_scene(S(inputs={})))

    def test_arc_command_keeps_both_leader_wheels_forward(self):
        for direction in ('left','right'):
            w=leader_turn_rate(direction)
            wheels=(.04-w*.466/2,.04+w*.466/2)
            self.assertTrue(all(v>0 for v in wheels))
        self.assertEqual(leader_turn_rate('straight'),0.)

    def test_remote_skew_only_and_stale_receipt_rejected(self):
        m=S(header=S(stamp=S(sec=1,nanosec=0)))
        self.assertTrue(fresh_sample((m,1.),980_000_000,1.1,remote=True))
        self.assertFalse(fresh_sample((m,1.),980_000_000,1.1))
        self.assertFalse(fresh_sample((m,1.),1_100_000_000,1.31,remote=True))

    def state(self):
        return dict(mode='HARDWARE_CANDIDATE',input_kind='guard',trial_max_rpm=10,
                    enabled=True,armed=False,fault=None,io_failed=False,
                    feedback_fresh=True,measured_rpm=[0,0])

    def test_preflight_requires_explicit_ten_rpm_and_healthy_zero(self):
        s=self.state();self.assertIsNone(motor_reason((s,1.),1.1,stationary=True))
        for key,value in [('trial_max_rpm',5),('enabled',False),('armed',True),
                          ('fault','SERIAL_WRITE_ERROR'),('feedback_fresh',False),('measured_rpm',[1,0])]:
            s=self.state();s[key]=value
            self.assertIsNotNone(motor_reason((s,1.),1.1,stationary=True))

    def test_running_motor_may_have_nonzero_feedback(self):
        s=self.state();s.update(armed=True,measured_rpm=[-10,10])
        self.assertIsNone(motor_reason((s,1.),1.1,stationary=False))

    def guard(self,v=.1):
        return S(header=S(frame_id='follower/base_link',stamp=S(sec=1,nanosec=0)),
                 twist=S(linear=S(x=v,y=0.,z=0.),angular=S(x=0.,y=0.,z=0.)))

    def test_guard_zero_or_invalid_does_not_continue(self):
        core=Mock();core.command.return_value=(.1,0.,'PREVIEW_TRACK')
        self.assertEqual(scene_reason(core,(self.guard(),1.),1_100_000_000,1.1),'READY')
        self.assertNotEqual(scene_reason(core,(self.guard(0.),1.),1_100_000_000,1.1),'READY')
        self.assertNotEqual(scene_reason(core,(self.guard(),1.),1_400_000_000,1.1),'READY')
        core.command.return_value=(0.,0.,'STOP_LEADER_STOPPED')
        self.assertEqual(scene_reason(core,(self.guard(),1.),1_100_000_000,1.1),'STOP_LEADER_STOPPED')

    def test_ten_rpm_at_core_and_byte_boundary_then_disarm(self):
        c=Clock();d=MemoryTransport();p=BoundedDrivePort(d,c,max_rpm=10)
        k=DriveContract(p,enabled=True,clock=c,max_rpm=10)
        for _ in range(70):k.tick(c.ros);c.now+=.01
        self.assertIsNotNone(k.arm(c.ros));c.now+=.01
        for _ in range(20):
            obj=dict(version=1,epoch=k.core.epoch,frame='follower/base_link',stamp_ns=c.ros,v=.1,w=0.)
            self.assertTrue(k.command(json.dumps(obj),c.ros));k.tick(c.ros);c.now+=.01
        self.assertEqual(d.rpm,(-10,10))
        for pair in [(-11,10),(-10,11),(10,10),(-10,-10)]:
            with self.assertRaises(ValueError):p.write(velocity_packet(*pair))
        k.disarm()
        for _ in range(50):k.tick(c.ros);c.now+=.01
        self.assertEqual(d.rpm,(0,0));self.assertFalse(k.core.armed)
        with self.assertRaises(ValueError):p.write(velocity_packet(-10,10))

    def test_ten_rpm_still_has_five_second_hard_stop(self):
        c=Clock();d=MemoryTransport();p=BoundedDrivePort(d,c,max_rpm=10)
        k=DriveContract(p,enabled=True,clock=c,max_rpm=10)
        for _ in range(70):k.tick(c.ros);c.now+=.01
        k.arm(c.ros);c.now+=.01
        for _ in range(600):
            if k.core.armed:
                obj=dict(version=1,epoch=k.core.epoch,frame='follower/base_link',stamp_ns=c.ros,v=.1,w=0.)
                k.command(json.dumps(obj),c.ros)
            k.tick(c.ros);c.now+=.01
        self.assertEqual(k.core.fault,'FIVE_SECOND_TRIAL_EXPIRED')
        self.assertEqual(d.rpm,(0,0));self.assertIsNone(k.arm(c.ros))

    def test_explicit_ten_second_driver_still_has_hard_limit(self):
        c=Clock();d=MemoryTransport();p=BoundedDrivePort(d,c,max_rpm=10)
        k=DriveContract(p,enabled=True,clock=c,max_rpm=10,trial_seconds=10)
        for _ in range(70):k.tick(c.ros);c.now+=.01
        k.arm(c.ros);c.now+=.01
        for _ in range(990):
            obj=dict(version=1,epoch=k.core.epoch,frame='follower/base_link',stamp_ns=c.ros,v=.1,w=0.)
            self.assertTrue(k.command(json.dumps(obj),c.ros));k.tick(c.ros);c.now+=.01
        self.assertTrue(k.core.armed);self.assertEqual(d.rpm,(-10,10))
        for _ in range(30):
            if k.core.armed:
                obj=dict(version=1,epoch=k.core.epoch,frame='follower/base_link',stamp_ns=c.ros,v=.1,w=0.)
                k.command(json.dumps(obj),c.ros)
            k.tick(c.ros);c.now+=.01
        self.assertEqual(k.core.fault,'TEN_SECOND_TRIAL_EXPIRED')
        self.assertEqual(d.rpm,(0,0));self.assertIsNone(k.arm(c.ros))
    def test_explicit_thirty_second_driver_still_has_hard_limit(self):
        c=Clock();d=MemoryTransport();p=BoundedDrivePort(d,c,max_rpm=10)
        k=DriveContract(p,enabled=True,clock=c,max_rpm=10,trial_seconds=30)
        for _ in range(70):k.tick(c.ros);c.now+=.01
        k.arm(c.ros);c.now+=.01
        for _ in range(2990):
            obj=dict(version=1,epoch=k.core.epoch,frame='follower/base_link',stamp_ns=c.ros,v=.1,w=0.)
            self.assertTrue(k.command(json.dumps(obj),c.ros));k.tick(c.ros);c.now+=.01
        self.assertTrue(k.core.armed);self.assertEqual(d.rpm,(-10,10))
        for _ in range(30):
            if k.core.armed:
                obj=dict(version=1,epoch=k.core.epoch,frame='follower/base_link',stamp_ns=c.ros,v=.1,w=0.)
                k.command(json.dumps(obj),c.ros)
            k.tick(c.ros);c.now+=.01
        self.assertEqual(k.core.fault,'THIRTY_SECOND_TRIAL_EXPIRED')
        self.assertEqual(d.rpm,(0,0));self.assertIsNone(k.arm(c.ros))


class RecorderGraphTests(unittest.TestCase):
    def test_recorder_is_opt_in_and_unknown_readers_rejected(self):
        from run_joint_follow_trial import command_readers_ok
        from types import SimpleNamespace as S
        controller=S(node_name='md200t_controller',node_namespace='/')
        recorder=S(node_name='rosbag2_recorder',node_namespace='/')
        self.assertTrue(command_readers_ok([controller]))
        self.assertFalse(command_readers_ok([controller,recorder]))
        self.assertTrue(command_readers_ok([controller,recorder],True))
        for readers in ([recorder],[controller,controller],[controller,recorder,recorder],
                        [controller,S(node_name='other')],
                        [controller,S(node_name='rosbag2_recorder',node_namespace='/other')]):
            self.assertFalse(command_readers_ok(readers,True))


if __name__=='__main__':unittest.main()
