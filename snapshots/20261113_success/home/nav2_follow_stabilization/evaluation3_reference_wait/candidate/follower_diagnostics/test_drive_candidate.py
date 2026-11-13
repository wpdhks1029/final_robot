import json
import unittest
from follower_drive_candidate import BoundedDrivePort,DriveContract
from follower_motor_candidate import velocity_packet
from follower_motor_mock import MemoryTransport
from test_motor_candidate import Clock,frame


class Tests(unittest.TestCase):
    def make(self,enabled=True):
        c=Clock();d=MemoryTransport();p=BoundedDrivePort(d,c)
        return c,d,DriveContract(p,enabled=enabled,clock=c)

    def tick(self,c,k,n=30):
        for _ in range(n):k.tick(c.ros);c.now+=.01

    def arm(self,c,k):
        self.tick(c,k,70);self.assertIsNotNone(k.arm(c.ros));c.now+=.01

    def send(self,c,k,v=.02,w=0,**kw):
        obj=dict(version=1,epoch=k.core.epoch,frame='follower/base_link',stamp_ns=c.ros,v=v,w=w)
        obj.update(kw);return k.command(json.dumps(obj),c.ros)

    def test_default_enabled_false_rejects_arm(self):
        c,d,k=self.make(False);self.tick(c,k)
        self.assertIsNone(k.arm(c.ros));self.assertEqual(d.rpm,(0,0))

    def test_forward_cap_and_inversion(self):
        c,d,k=self.make();self.arm(c,k);self.assertTrue(self.send(c,k,.1))
        self.tick(c,k,15);self.assertEqual(d.rpm,(-3,3))

    def test_curved_forward_only(self):
        c,d,k=self.make();self.arm(c,k);self.assertTrue(self.send(c,k,.1,.2))
        self.tick(c,k,15);self.assertTrue(-3<=d.rpm[0]<=0);self.assertEqual(d.rpm[1],3)

    def test_reverse_and_inplace_rejected(self):
        for v,w in [(-.01,0),(0,.05),(.001,.1),(.11,0),(0,.21)]:
            c,d,k=self.make();self.arm(c,k)
            self.assertFalse(self.send(c,k,v,w));self.tick(c,k)
            self.assertEqual(d.rpm,(0,0));self.assertFalse(k.core.armed)

    def test_invalid_session_stamp_replay_latches(self):
        c,d,k=self.make();self.arm(c,k);self.assertTrue(self.send(c,k));self.tick(c,k,15)
        old=k.core.last_command_stamp
        self.assertFalse(self.send(c,k,stamp_ns=old));self.tick(c,k)
        self.assertEqual(d.rpm,(0,0));self.assertIsNone(k.arm(c.ros))

    def test_command_loss_stops_no_auto_rearm(self):
        c,d,k=self.make();self.arm(c,k);self.send(c,k);self.tick(c,k,80)
        self.assertEqual(d.rpm,(0,0));self.assertEqual(k.core.fault,'COMMAND_TIMEOUT')
        self.assertIsNone(k.arm(c.ros))

    def test_feedback_loss_with_fresh_commands(self):
        c,d,k=self.make();self.arm(c,k);self.send(c,k);self.tick(c,k,15)
        d.feedback_enabled=False
        for _ in range(70):
            if k.core.armed:self.send(c,k)
            self.tick(c,k,1)
        self.assertFalse(k.core.armed);self.assertEqual(d.rpm,(0,0))

    def test_fixed_five_second_limit_even_with_commands(self):
        c,d,k=self.make();self.arm(c,k)
        for _ in range(600):
            if k.core.armed:self.send(c,k)
            self.tick(c,k,1)
        self.assertEqual(k.core.fault,'FIVE_SECOND_TRIAL_EXPIRED')
        self.assertEqual(d.rpm,(0,0));self.assertIsNone(k.arm(c.ros))

    def test_port_gate_and_packet_limits_independent(self):
        c,d,k=self.make()
        with self.assertRaises(ValueError):k.port.write(velocity_packet(-3,3))
        k.port.open_gate(c.now+1)
        for packet in [velocity_packet(-4,3),velocity_packet(3,3),velocity_packet(-3,-3)]:
            with self.assertRaises(ValueError):k.port.write(packet)
        c.now+=1
        with self.assertRaises(ValueError):k.port.write(velocity_packet(-3,3))
        k.port.write(velocity_packet(0,0));self.assertEqual(d.rpm,(0,0))

    def test_device_status_fault_before_nonzero_transmit(self):
        c,d,k=self.make();self.arm(c,k)
        data=dict(m1_rpm=0,m2_rpm=0,m1_position=0,m2_position=0,m1_status=1,m2_status=0)
        k.core._record_feedback(data,c.now)
        self.assertEqual(k.core.fault,'DEVICE_STATUS_NONZERO');self.assertFalse(k.core.armed)

    def test_disarm_cannot_restart_old_commands(self):
        c,d,k=self.make();self.arm(c,k);epoch=k.core.epoch;self.send(c,k);self.tick(c,k,15)
        k.disarm();self.tick(c,k);self.assertEqual(d.rpm,(0,0))
        self.assertFalse(self.send(c,k,epoch=epoch))

    def test_late_stale_guard_after_disarm_does_not_poison_stopped_state(self):
        c,d,k=self.make();self.arm(c,k)
        old=dict(version=2,epoch=k.core.epoch,frame='follower/base_link',stamp_ns=c.ros,
                 source_stamp_ns=c.ros,v=.02,w=0.)
        k.disarm();self.tick(c,k,40)
        self.assertFalse(k.command(json.dumps(old),c.ros,require_source=True))
        self.assertFalse(k.command('malformed late packet',c.ros,require_source=True))
        self.assertIsNone(k.core.fault);self.assertFalse(k.core.armed)
        self.assertEqual(d.rpm,(0,0));self.assertIsNone(k.core.command)

    def test_stale_guard_while_armed_still_latches_and_keeps_evidence(self):
        c,d,k=self.make();self.arm(c,k)
        bad=dict(version=2,epoch=k.core.epoch,frame='follower/base_link',stamp_ns=c.ros,
                 source_stamp_ns=c.ros-1_000_000_000,v=.02,w=0.)
        self.assertFalse(k.command(json.dumps(bad),c.ros,require_source=True))
        self.assertEqual(k.core.fault,'INVALID_DRIVE_ENVELOPE')
        self.assertTrue(k.last_command_rejection['armed_at_rejection'])
        self.assertEqual(k.last_command_rejection['source_stamp_ns_age_s'],1.)
        trace=k.last_command_rejection.copy()
        self.assertFalse(k.command('late junk',c.ros,require_source=True))
        self.assertEqual(k.last_command_rejection,trace)
        self.assertEqual(k.core.fault,'INVALID_DRIVE_ENVELOPE');self.assertIsNone(k.arm(c.ros))

    def test_huge_source_stamp_rejection_trace_does_not_crash(self):
        c,d,k=self.make();self.arm(c,k)
        obj=dict(version=2,epoch=k.core.epoch,frame='follower/base_link',stamp_ns=c.ros,
                 source_stamp_ns=10**400,v=.02,w=0.)
        self.assertFalse(k.command(json.dumps(obj),c.ros,require_source=True))
        self.assertEqual(k.core.fault,'INVALID_DRIVE_ENVELOPE')
        json.dumps(k.status(),allow_nan=False)

    def test_old_session_after_explicit_rearm_remains_rejected(self):
        c,d,k=self.make();self.arm(c,k);old=k.core.epoch
        k.disarm();self.tick(c,k,70);self.assertIsNotNone(k.arm(c.ros));c.now+=.01
        obj=dict(version=2,epoch=old,frame='follower/base_link',stamp_ns=c.ros,
                 source_stamp_ns=c.ros,v=.02,w=0.)
        self.assertFalse(k.command(json.dumps(obj),c.ros,require_source=True))
        self.assertFalse(k.core.armed);self.assertEqual(k.core.fault,'INVALID_COMMAND')
        self.assertFalse(k.last_command_rejection['epoch_matches'])

    def test_explicit_five_rpm_has_independent_port_cap(self):
        c=Clock();d=MemoryTransport();p=BoundedDrivePort(d,c,max_rpm=5)
        k=DriveContract(p,enabled=True,clock=c,max_rpm=5)
        self.arm(c,k);self.assertTrue(self.send(c,k,.1));self.tick(c,k,15)
        self.assertEqual(d.rpm,(-5,5));self.assertEqual(k.status()['trial_max_rpm'],5)
        for packet in (velocity_packet(-6,5),velocity_packet(-5,6),velocity_packet(5,5)):
            with self.assertRaises(ValueError):p.write(packet)
        k.disarm();self.tick(c,k);self.assertEqual(d.rpm,(0,0))
        with self.assertRaises(ValueError):p.write(velocity_packet(-5,5))

    def test_rpm_cap_configuration_mismatch_rejected(self):
        c=Clock();d=MemoryTransport()
        for value in (0,4,6,True):
            with self.assertRaises(ValueError):BoundedDrivePort(d,c,max_rpm=value)
        p=BoundedDrivePort(d,c,max_rpm=5)
        with self.assertRaises(ValueError):DriveContract(p,enabled=True,clock=c)

    def test_five_rpm_still_stops_at_five_seconds(self):
        c=Clock();d=MemoryTransport();p=BoundedDrivePort(d,c,max_rpm=5)
        k=DriveContract(p,enabled=True,clock=c,max_rpm=5)
        self.arm(c,k)
        for _ in range(600):
            if k.core.armed:self.send(c,k,.1)
            self.tick(c,k,1)
        self.assertEqual(k.core.fault,'FIVE_SECOND_TRIAL_EXPIRED')
        self.assertEqual(d.rpm,(0,0));self.assertIsNone(k.arm(c.ros))


if __name__=='__main__':unittest.main()
