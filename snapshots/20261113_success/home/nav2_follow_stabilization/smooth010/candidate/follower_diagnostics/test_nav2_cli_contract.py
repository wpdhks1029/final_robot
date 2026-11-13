import contextlib
import io
import os
import sys
import unittest
from unittest.mock import patch

import run_local_avoidance_trial as runner


class Nav2CliContractTests(unittest.TestCase):
    def test_nav2_leader_rejects_missing_continuous_dynamic_handoff_contract(self):
        argv = ['run', '--run', '--ack-clear-both-power-cut-ready',
                '--nav2-leader', '--continuous20']
        stderr = io.StringIO()
        with patch.object(sys, 'argv', argv), \
             patch.dict(os.environ, {'ROS_DOMAIN_ID': '10'}), \
             contextlib.redirect_stderr(stderr), \
             self.assertRaises(SystemExit) as caught:
            runner.main()
        self.assertEqual(caught.exception.code, 2)
        self.assertIn('--nav2-leader requires', stderr.getvalue())

    def test_nav2_leader_is_never_available_in_plan_only_mode(self):
        argv = ['run', '--plan-only', '--nav2-leader', '--continuous20',
                '--dynamic-box', '--map-rejoin', '--continue-follow-after-rejoin']
        stderr = io.StringIO()
        with patch.object(sys, 'argv', argv), \
             patch.dict(os.environ, {'ROS_DOMAIN_ID': '10'}), \
             contextlib.redirect_stderr(stderr), \
             self.assertRaises(SystemExit) as caught:
            runner.main()
        self.assertEqual(caught.exception.code, 2)
        self.assertIn('--nav2-leader requires', stderr.getvalue())


if __name__ == '__main__':
    unittest.main()
