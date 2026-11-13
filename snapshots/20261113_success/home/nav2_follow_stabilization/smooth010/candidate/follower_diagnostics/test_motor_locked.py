"""Locked wrapper policy/CLI gates. No ROS initialization or device opening."""
import contextlib
import io
import sys
from types import SimpleNamespace as S
import unittest
from unittest.mock import Mock,patch

import follower_motor_locked as locked


class LockedTests(unittest.TestCase):
    def test_zero_measured_feedback_not_faulted(self):
        core=S(feedback={'m1_rpm':0,'m2_rpm':0},latch_fault=Mock())
        locked.check_locked_feedback(core);core.latch_fault.assert_not_called()

    def test_measured_motion_is_fault(self):
        for left,right in [(1,0),(0,-1),(-3,3)]:
            core=S(feedback={'m1_rpm':left,'m2_rpm':right},latch_fault=Mock())
            locked.check_locked_feedback(core)
            core.latch_fault.assert_called_once_with('UNEXPECTED_MOTION_WHILE_LOCKED')
            self.assertEqual((core.feedback['m1_rpm'],core.feedback['m2_rpm']),(left,right))

    def test_no_feedback_not_manufactured(self):
        core=S(feedback=None,latch_fault=Mock())
        locked.check_locked_feedback(core);self.assertIsNone(core.feedback)

    def test_default_cli_starts_nothing(self):
        with patch.object(sys,'argv',['locked']),contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(locked.main(),0)

    def test_hardware_request_requires_tx_ack(self):
        with patch.object(sys,'argv',['locked','--run-hardware-locked']),contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit) as result:locked.main()
        self.assertEqual(result.exception.code,2)


if __name__=='__main__':unittest.main(verbosity=2)
