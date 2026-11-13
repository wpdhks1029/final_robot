import contextlib
import io
import json
import math
import sys
import unittest
from unittest.mock import patch

import follower_drive_node as driver
import run_local_avoidance_trial as operator
from follower_drive_candidate import BoundedDrivePort
from follower_motor_mock import MemoryTransport
from local_avoidance_drive import LocalAvoidanceContract, PivotDrivePort, PROFILE
from nav2_follow_safety import bounded_pursuit_result
from nav2_convoy_gate_core import Nav2ConvoyGateCore
from test_motor_candidate import Clock
from test_nav2_follow_safety import FollowSafetyTests


class ExtendedSpeedTests(unittest.TestCase):
    def contract(self, extended=True):
        clock=Clock();device=MemoryTransport()
        rpm=15 if extended else 10
        port=PivotDrivePort(device,clock,max_rpm=rpm)
        contract=LocalAvoidanceContract(port,clock=clock,enabled=True,max_rpm=rpm,
                                       trial_seconds=360,extended_follow=extended)
        for _ in range(75):contract.tick(clock.ros);clock.now+=.01
        self.assertIsNotNone(contract.arm(clock.ros));clock.now+=.01
        return clock,device,contract

    def command(self, clock, contract, v, w=0., age=0.):
        return json.dumps(dict(version=3,controller=PROFILE,epoch=contract.core.epoch,
            frame='follower/base_link',stamp_ns=clock.ros,
            source_stamp_ns=clock.ros-round(age*1e9),scan_stamp_ns=clock.ros,
            motion='DRIVE',v=v,w=w,clearance_m=.5,swept_clearance_m=.4))

    def test_explicit_profile_required_in_contract(self):
        device=MemoryTransport();port=PivotDrivePort(device,max_rpm=15)
        with self.assertRaises(ValueError):
            LocalAvoidanceContract(port,max_rpm=15)

    def test_driver_ack_required_before_ros_or_port_open(self):
        base=['driver','--run-hardware','--ack-hardware-zero-rpm-tx',
              '--input-kind','local_avoidance','--ack-bounded-pivot-candidate',
              '--trial-max-rpm','15','--trial-seconds','360']
        with patch.object(BoundedDrivePort,'open') as opened,contextlib.redirect_stderr(io.StringIO()):
            for argv in (base,base+['--ack-extended-follow-speed','--ack-continuous20-candidate']):
                with patch('sys.argv',argv),self.assertRaises(SystemExit):driver.main()
            opened.assert_not_called()

    def test_operator_ack_required_before_ros(self):
        argv=['operator','--run','--ack-clear-both-power-cut-ready',
              '--nav2-leader','--follow-only','--follow-max-linear','0.1']
        with patch('sys.argv',argv),contextlib.redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):
            operator.main()

    def test_baseline_still_rejects_high_speed(self):
        clock,device,contract=self.contract(False)
        self.assertFalse(contract.command(self.command(clock,contract,.1),clock.ros))
        self.assertEqual(contract.core.fault,'INVALID_LOCAL_AVOIDANCE_ENVELOPE')

    def test_explicit_profile_accepts_straight_and_curved_commands_with_15rpm_cap(self):
        for w in (-.2,0.,.2):
            clock,device,contract=self.contract()
            scene=FollowSafetyTests().scene()
            r=bounded_pursuit_result(scene,.1,w,'TRACKED_PURSUIT',max_linear=.1,max_rpm=15.)
            for _ in range(50):
                self.assertTrue(contract.command(self.command(clock,contract,r['v'],r['w']),clock.ros),contract.last_command_rejection)
                contract.tick(clock.ros);clock.now+=.01
            self.assertIsNone(contract.core.fault)
            self.assertTrue(-15<=device.rpm[0]<=0<=device.rpm[1]<=15)
            if w==0.:self.assertEqual(device.rpm,(-15,15))

    def test_extended_profile_keeps_stale_and_speed_rejection(self):
        for v,age in ((.101,0.),(.1,1.)):
            clock,device,contract=self.contract()
            self.assertFalse(contract.command(self.command(clock,contract,v,age=age),clock.ros))
            self.assertFalse(contract.core.armed)
            contract.tick(clock.ros)
            self.assertEqual(device.rpm,(0,0))

    def test_operator_rejects_mismatched_driver_limit(self):
        clock,device,contract=self.contract()
        status=contract.status()
        status.update(mode='HARDWARE_CANDIDATE',input_kind='local_avoidance')
        self.assertIsNone(operator.motor_problem((status,clock.now),clock.now,360,expected_max_rpm=15))
        self.assertEqual(operator.motor_problem((status,clock.now),clock.now,360),
                         'REQUIRE_LOCAL_AVOIDANCE_DRIVER_MATCH')

    def test_extended_profile_keeps_command_loss_stop(self):
        clock,device,contract=self.contract()
        self.assertTrue(contract.command(self.command(clock,contract,.1),clock.ros))
        for _ in range(60):contract.tick(clock.ros);clock.now+=.01
        self.assertEqual(contract.core.fault,'COMMAND_TIMEOUT')
        self.assertFalse(contract.core.armed)
        self.assertEqual(device.rpm,(0,0))

    def test_gate_high_speed_retains_slow_start_and_zero_wait(self):
        gate=Nav2ConvoyGateCore(max_linear=.1);gate.graph_ok=True
        gate.update_nav(.15,0.,1.)
        gate.update_motor(dict(armed=True,fault=None,io_failed=False,feedback_fresh=True),1.)
        gate.update_leader_pose(2.,0.,1.);gate.update_follower_pose(0.,0.,1.)
        gate.update_follower(dict(trial_phase='FOLLOW',reason='FOLLOW_WAITING_LEADER',
            follower_follow_reason='STOP_LEADER_STOPPED',follower_requested_v=0.),1.)
        self.assertAlmostEqual(gate.evaluate(1.).linear_x,.04)
        gate.update_follower(dict(trial_phase='FOLLOW',reason='TRACKED_PURSUIT',
            follower_follow_reason='TRACKED_PURSUIT',follower_requested_v=.1),1.)
        self.assertAlmostEqual(gate.evaluate(1.).linear_x,.1)
        gate.update_follower(dict(trial_phase='FOLLOW',reason='STOP_FOLLOW_REFERENCE_WAIT'),1.)
        self.assertEqual(gate.evaluate(1.).linear_x,0.)


if __name__=='__main__':unittest.main()
