import json,unittest
from local_avoidance_drive import LocalAvoidanceContract,PivotDrivePort,PROFILE
from follower_motor_mock import MemoryTransport
from follower_motor_candidate import velocity_packet
from test_motor_candidate import Clock

class Tests(unittest.TestCase):
    def test_explicit_clearance_disable_accepts_actual_low_clearance(self):
        c,t,p,k=self.contract();k.clearance_stop_enabled=False
        obj=self.envelope(c,k,'DRIVE',.04,0.)
        obj.update(clearance_m=-.02,swept_clearance_m=-.03)
        self.assertTrue(k.command(json.dumps(obj),c.ros),k.last_command_rejection)
        self.assertFalse(k.status()['clearance_stop_enabled'])

    def test_clearance_disable_preserves_stale_source_and_wheel_limits(self):
        for v,age in ((.04,1.),(.07,0.)):
            c,t,p,k=self.contract();k.clearance_stop_enabled=False
            obj=self.envelope(c,k,'DRIVE',v,0.)
            obj.update(clearance_m=-.02,swept_clearance_m=-.03,
                source_stamp_ns=c.ros-round(age*1e9))
            self.assertFalse(k.command(json.dumps(obj),c.ros))
            self.assertFalse(k.core.armed)

    def contract(self,seconds=30):
        c=Clock();t=MemoryTransport();p=PivotDrivePort(t,c,max_rpm=10)
        k=LocalAvoidanceContract(p,enabled=True,clock=c,max_rpm=10,trial_seconds=seconds)
        for _ in range(75):k.tick(c.ros);c.now+=.01
        epoch=k.arm(c.ros);self.assertIsNotNone(epoch);c.now+=.01
        return c,t,p,k
    def envelope(self,c,k,motion='TURN',v=0.,w=.12):
        return dict(version=3,controller=PROFILE,epoch=k.core.epoch,frame='follower/base_link',
                    stamp_ns=c.ros,source_stamp_ns=c.ros,scan_stamp_ns=c.ros,motion=motion,
                    v=v,w=w,clearance_m=.4,swept_clearance_m=.35)
    def test_waiting_for_post_arm_support_allows_only_fresh_zero_stop(self):
        c,t,p,k=self.contract();old=k.core.arm_ros_ns-100_000_000
        for _ in range(45):
            obj=self.envelope(c,k,'STOP',0.,0.);obj.update(source_stamp_ns=old,scan_stamp_ns=old)
            self.assertTrue(k.command(json.dumps(obj),c.ros),k.last_command_rejection)
            k.tick(c.ros);self.assertIsNone(k.core.fault);self.assertEqual(t.rpm,(0,0));c.now+=.01
        self.assertTrue(k.command(json.dumps(self.envelope(c,k)),c.ros))
    def test_pre_arm_source_still_forbidden_for_nonzero_and_stale_stop(self):
        for motion,v,w,age in [('TURN',0.,.12,.1),('DRIVE',.04,0.,.1),('STOP',0.,.01,.1),('STOP',0.,0.,.9)]:
            c,t,p,k=self.contract();obj=self.envelope(c,k,motion,v,w)
            old=k.core.arm_ros_ns-round(age*1e9);obj.update(source_stamp_ns=old,scan_stamp_ns=old)
            self.assertFalse(k.command(json.dumps(obj),c.ros));self.assertFalse(k.core.armed)
    def test_zero_wait_commands_do_not_disable_command_loss_watchdog(self):
        c,t,p,k=self.contract();obj=self.envelope(c,k,'STOP',0.,0.)
        obj.update(source_stamp_ns=k.core.arm_ros_ns-100_000_000,scan_stamp_ns=k.core.arm_ros_ns-100_000_000)
        self.assertTrue(k.command(json.dumps(obj),c.ros))
        for _ in range(35):k.tick(c.ros);c.now+=.01
        self.assertEqual(k.core.fault,'COMMAND_TIMEOUT');self.assertEqual(t.rpm,(0,0))
    def test_turn_accepts_bounded_opposite_physical_wheels(self):
        c,t,p,k=self.contract()
        for _ in range(40):
            self.assertTrue(k.command(json.dumps(self.envelope(c,k)),c.ros));k.tick(c.ros);c.now+=.01
        self.assertGreater(t.rpm[0],0);self.assertEqual(t.rpm[0],t.rpm[1]);self.assertLessEqual(abs(t.rpm[0]),10)
        self.assertGreater(k.core.odom_yaw,0.)
    def test_turn_cannot_be_used_for_reverse_translation(self):
        c,t,p,k=self.contract();p.motion_kind='TURN'
        with self.assertRaises(ValueError):p.write(velocity_packet(3,4))
        with self.assertRaises(ValueError):p.write(velocity_packet(11,11))
        obj=self.envelope(c,k,v=.01)
        self.assertFalse(k.command(json.dumps(obj),c.ros));self.assertFalse(k.core.armed)
    def test_phase_change_requires_zero_feedback(self):
        c,t,p,k=self.contract()
        for _ in range(30):
            self.assertTrue(k.command(json.dumps(self.envelope(c,k)),c.ros));k.tick(c.ros);c.now+=.01
        self.assertTrue(k.command(json.dumps(self.envelope(c,k,'DRIVE',.04,0.)),c.ros))
        self.assertIsNone(k.core.fault);self.assertTrue(k.phase_waiting_for_zero)
        self.assertEqual(k.core.command[:2],(0.,0.));self.assertEqual(p.motion_kind,'STOP')
    def test_opposite_pivot_direction_requires_stop(self):
        c,t,p,k=self.contract()
        for _ in range(30):
            k.command(json.dumps(self.envelope(c,k)),c.ros);k.tick(c.ros);c.now+=.01
        self.assertTrue(k.command(json.dumps(self.envelope(c,k,w=-.12)),c.ros))
        self.assertIsNone(k.core.fault);self.assertTrue(k.phase_waiting_for_zero)
        self.assertEqual(k.core.command[:2],(0.,0.));self.assertEqual(k.last_turn_sign,1)

    def test_stop_then_zero_samples_permits_next_phase(self):
        c,t,p,k=self.contract()
        for _ in range(30):
            k.command(json.dumps(self.envelope(c,k)),c.ros);k.tick(c.ros);c.now+=.01
        for _ in range(55):
            self.assertTrue(k.command(json.dumps(self.envelope(c,k,'STOP',0.,0.)),c.ros));k.tick(c.ros);c.now+=.01
        self.assertGreaterEqual(k.zero_samples,3)
        self.assertTrue(k.command(json.dumps(self.envelope(c,k,'DRIVE',.04,0.)),c.ros))
        self.assertEqual(k.zero_samples,0)
    def test_stale_zero_status_transition_holds_then_uses_actual_feedback(self):
        c,t,p,k=self.contract()
        for _ in range(30):
            k.command(json.dumps(self.envelope(c,k)),c.ros);k.tick(c.ros);c.now+=.01
        waiting=False;resumed=False
        for _ in range(90):
            before=k.zero_samples
            self.assertTrue(k.command(json.dumps(self.envelope(c,k,'DRIVE',.04,0.)),c.ros))
            if k.phase_waiting_for_zero:
                waiting=True;self.assertEqual(k.core.command[:2],(0.,0.))
                self.assertEqual(k.last_nonzero_motion,'TURN')
            elif k.last_nonzero_motion=='DRIVE':
                if not resumed:self.assertGreaterEqual(before,3)
                resumed=True
            k.tick(c.ros);self.assertIsNone(k.core.fault);c.now+=.01
        self.assertTrue(waiting);self.assertTrue(resumed)
        self.assertLess(t.rpm[0],0);self.assertGreater(t.rpm[1],0)
    def test_invalid_epoch_while_waiting_still_faults(self):
        c,t,p,k=self.contract()
        for _ in range(30):
            k.command(json.dumps(self.envelope(c,k)),c.ros);k.tick(c.ros);c.now+=.01
        obj=self.envelope(c,k,'DRIVE',.04,0.);obj['epoch']='0'*16
        self.assertFalse(k.command(json.dumps(obj),c.ros))
        self.assertFalse(k.core.armed);self.assertIsNotNone(k.core.fault)
    def test_stale_source_and_missing_clearance_latch(self):
        for field,value in [('source_stamp_ns',1),('scan_stamp_ns',1),('clearance_m',.3),('swept_clearance_m',.29)]:
            c,t,p,k=self.contract();obj=self.envelope(c,k);obj[field]=value
            self.assertFalse(k.command(json.dumps(obj),c.ros));self.assertIsNone(p.deadline)
    def test_old_guard_protocol_and_wrong_epoch_rejected(self):
        for change in ({'version':2},{'epoch':'0'*16}):
            c,t,p,k=self.contract();obj=self.envelope(c,k);obj.update(change)
            self.assertFalse(k.command(json.dumps(obj),c.ros))
    def test_port_deadline_independent_of_contract_tick(self):
        c,t,p,k=self.contract(360);p.motion_kind='TURN';c.now=k.deadline
        with self.assertRaises(ValueError):p.write(velocity_packet(5,5))
        k.tick(c.ros);self.assertEqual(k.core.fault,'LOCAL_AVOIDANCE_TRIAL_EXPIRED')
    def test_command_loss_still_times_out(self):
        c,t,p,k=self.contract();k.command(json.dumps(self.envelope(c,k)),c.ros)
        for _ in range(50):k.tick(c.ros);c.now+=.01
        self.assertFalse(k.core.armed);self.assertEqual(k.core.fault,'COMMAND_TIMEOUT')
        self.assertEqual(t.rpm,(0,0))
    def test_normal_port_still_forbids_pivot(self):
        from follower_drive_candidate import BoundedDrivePort
        c=Clock();p=BoundedDrivePort(MemoryTransport(),c,max_rpm=10);p.open_gate(c.now+1.)
        with self.assertRaises(ValueError):p.write(velocity_packet(5,5))

if __name__=='__main__':unittest.main()

class CLI(unittest.TestCase):
    def test_new_mode_requires_ack_and_long_mode_is_not_available_to_old_guard(self):
        from follower_drive_node import main
        from unittest.mock import patch
        for args in (['--input-kind','local_avoidance','--trial-max-rpm','10','--trial-seconds','30'],
                     ['--input-kind','guard','--trial-seconds','360']):
            with patch('sys.argv',['driver','--run-hardware','--ack-hardware-zero-rpm-tx']+args):
                with self.assertRaises(SystemExit):main()

class SourceLifetimeTests(unittest.TestCase):
    contract=Tests.contract
    envelope=Tests.envelope
    def test_eight_tenths_source_window_and_three_tenths_device_watchdogs(self):
        c,t,p,k=self.contract();source=c.ros
        for _ in range(75):
            obj=self.envelope(c,k,'STOP',0.,0.);obj['source_stamp_ns']=source;obj['scan_stamp_ns']=source
            self.assertTrue(k.command(json.dumps(obj),c.ros),k.last_command_rejection)
            k.tick(c.ros);c.now+=.01
        obj=self.envelope(c,k);obj['source_stamp_ns']=source;obj['scan_stamp_ns']=source
        self.assertTrue(k.command(json.dumps(obj),c.ros))
        self.assertEqual(k.status()['input_source_timeout_s'],.8)
        self.assertEqual(k.core.config.command_timeout,.3);self.assertEqual(k.core.config.feedback_timeout,.3);self.assertEqual(k.core.config.response_timeout,.3)
        c.now+=.06;obj=self.envelope(c,k);obj['source_stamp_ns']=source;obj['scan_stamp_ns']=source
        self.assertFalse(k.command(json.dumps(obj),c.ros));self.assertIsNone(p.deadline)
