import contextlib
import io
import json
import unittest
from unittest.mock import patch

from local_avoidance_drive import LocalAvoidanceContract, PivotDrivePort, PROFILE
from follower_motor_mock import MemoryTransport
from follower_motor_candidate import velocity_packet
from test_motor_candidate import Clock
import follower_drive_node as driver
import run_local_avoidance_trial as operator


class UnlimitedTests(unittest.TestCase):
    def make(self, seconds=0):
        clock=Clock();device=MemoryTransport();port=PivotDrivePort(device,clock,max_rpm=15)
        c=LocalAvoidanceContract(port,enabled=True,clock=clock,max_rpm=15,
            trial_seconds=seconds,extended_follow=True)
        for _ in range(75):c.tick(clock.ros);clock.now+=.01
        self.assertIsNotNone(c.arm(clock.ros));clock.now+=.01
        return clock,device,c

    def send(self, clock, c):
        obj=dict(version=3,controller=PROFILE,epoch=c.core.epoch,frame='follower/base_link',
            stamp_ns=clock.ros,source_stamp_ns=clock.ros,scan_stamp_ns=clock.ros,
            motion='DRIVE',v=.04,w=0.,clearance_m=.5,swept_clearance_m=.4)
        self.assertTrue(c.command(json.dumps(obj),clock.ros),c.last_command_rejection)

    def test_healthy_session_runs_beyond_six_minutes_then_disarms(self):
        clock,device,c=self.make()
        start=clock.now
        for _ in range(18300):
            self.send(clock,c);c.tick(clock.ros);clock.now+=.02
            self.assertIsNone(c.core.fault)
        self.assertGreater(clock.now-start,360.)
        self.assertTrue(c.core.armed)
        self.assertLessEqual(c.port.deadline-clock.now,.3)
        self.assertIsNone(c.deadline)
        status=c.status()
        self.assertTrue(status['session_unlimited']);self.assertEqual(status['trial_max_seconds'],0)
        self.assertIsNone(status['trial_remaining_s'])
        c.disarm();c.tick(clock.ros)
        self.assertEqual(device.rpm,(0,0));self.assertFalse(c.core.armed)

    def test_command_loss_still_faults_and_stops(self):
        clock,device,c=self.make();self.send(clock,c)
        for _ in range(30):c.tick(clock.ros);clock.now+=.02
        self.assertEqual(c.core.fault,'COMMAND_TIMEOUT')
        self.assertFalse(c.core.armed);self.assertEqual(device.rpm,(0,0))

    def test_feedback_loss_still_faults_and_stops(self):
        clock,device,c=self.make();device.feedback_enabled=False
        for _ in range(30):
            if c.core.armed:self.send(clock,c)
            c.tick(clock.ros);clock.now+=.02
        self.assertIn(c.core.fault,('FEEDBACK_STALE','RESPONSE_TIMEOUT'))
        self.assertFalse(c.core.armed);self.assertEqual(device.rpm,(0,0))

    def test_port_lease_expires_without_ticks(self):
        clock,device,c=self.make();c.port.motion_kind='DRIVE'
        clock.now+=.31
        with self.assertRaises(ValueError):c.port.write(velocity_packet(-3,3))

    def test_finite_limit_is_preserved(self):
        clock,device,c=self.make(seconds=30)
        clock.now=c.deadline+.01;c.tick(clock.ros)
        self.assertEqual(c.core.fault,'LOCAL_AVOIDANCE_TRIAL_EXPIRED')

    def test_unlimited_policy_mismatch_cannot_arm_through_operator(self):
        clock,device,c=self.make()
        status=dict(c.status(),mode='HARDWARE_CANDIDATE',input_kind='local_avoidance')
        self.assertIsNone(operator.motor_problem((status,clock.now),clock.now,0,expected_max_rpm=15))
        self.assertEqual(operator.motor_problem((status,clock.now),clock.now,360,expected_max_rpm=15),
                         'REQUIRE_LOCAL_AVOIDANCE_DRIVER_MATCH')

    def test_unlimited_rejected_outside_automatic_follow_mode_before_ros(self):
        with patch('sys.argv',['follow','--run','--seconds','0']),contextlib.redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):
            operator.main()
        with patch('sys.argv',['driver','--run-hardware','--trial-seconds','0']),contextlib.redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):
            driver.main()


if __name__=='__main__':unittest.main()
