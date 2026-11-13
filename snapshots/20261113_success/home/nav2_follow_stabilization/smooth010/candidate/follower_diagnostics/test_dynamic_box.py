"""Synthetic inputs only; no ROS sockets or hardware commands."""
import json
import math
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace as S
from unittest.mock import Mock, patch

from dynamic_box_detection import front_obstacle_count
from test_local_avoidance_operator import Flow
import run_local_avoidance_trial as runner


class Tests(unittest.TestCase):
    def test_front_bumper_range_and_rotated_frame(self):
        cloud = [(1.0, .1), (.9, .1), (.95, -.1), (-.5, 0), (.8, .7)]
        self.assertEqual(front_obstacle_count(cloud, (0, 0, 0)), 2)
        rotated = [(2-y, 3+x) for x, y in cloud]
        self.assertEqual(front_obstacle_count(rotated, (2, 3, math.pi/2)), 2)
        self.assertEqual(front_obstacle_count([], (0, 0, 0)), 0)

    def test_dynamic_preview_does_not_require_box_or_call_planner(self):
        flow = Flow(plan_only=True)
        modules, core = flow.modules()
        original_state = flow.state
        flow.state = lambda: dict(original_state(), continuous20=True, continuous_stop_clearance_m=.1)
        from local_avoidance_drive import COMMAND_TOPIC
        original_publishers = flow.get_publishers_info_by_topic
        original_subscribers = flow.get_subscriptions_info_by_topic
        translate = lambda topic: COMMAND_TOPIC if topic.endswith('/continuous20_command') else topic
        flow.get_publishers_info_by_topic = lambda topic: original_publishers(translate(topic))
        flow.get_subscriptions_info_by_topic = lambda topic: original_subscribers(translate(topic))
        planner = Mock(side_effect=AssertionError('Startup must not plan a box route'))
        modules['continuous_local_core'] = S(ContinuousCore=core, build_plan=planner)
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            reference = root/'reference.json'
            reference.write_text('{}')
            argv = ['trial', '--plan-only', '--continuous20', '--dynamic-box', '--apf-pivot',
                    '--gentle-pivot', '--map-rejoin', '--continue-follow-after-rejoin',
                    '--pose-source', 'filtered', '--reference-plan', str(reference)]
            with patch.dict(sys.modules, modules), patch.dict('os.environ', {'ROS_DOMAIN_ID':'10'}), \
                 patch('sys.argv', argv), patch.object(runner, 'ROOT', root), \
                 patch.object(runner, 'reference_from_plan', return_value=((0,0),0.,0.,0.)), \
                 patch.object(runner.time, 'monotonic', side_effect=lambda:flow.t):
                runner.main()
            rows = [json.loads(line) for line in next((root/'logs').glob('*.jsonl')).read_text().splitlines()]
        planner.assert_not_called()
        self.assertEqual(rows[-1]['reason'], 'PLAN_ONLY_READY')
        self.assertTrue(rows[-1]['dynamic_box'])
        self.assertFalse(rows[-1]['persistent_replan'])
        self.assertFalse(any(event[0] == 'arm' for event in flow.events))

    def test_continuous_planner_does_not_require_pivot_clearance(self):
        from continuous_local_core import build_plan
        from apf_pivot_candidate import APFPivot
        points = [(1., y/100) for y in range(-10, 11, 2)]
        with patch.object(APFPivot, 'choose', side_effect=AssertionError('No pivot in curve mode')), \
             patch('continuous_local_core.search', return_value=dict(reason='SIM_REJOIN_PLUS_1M',
                   offset_m=.65, approach_m=.65, rejoin_m=1., minimum_m=.25)):
            plan = build_plan(points, (0,0,0))
        self.assertIn('continuous_path', plan)
        self.assertGreater(plan['geometry']['min_path_clearance_m'], .2)


if __name__ == '__main__':
    unittest.main()
