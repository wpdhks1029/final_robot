import unittest

from nav2_follow_mode import nav2_leader_mode_issue


class Nav2LeaderModeTests(unittest.TestCase):
    def check(self, **overrides):
        values=dict(run=True,continuous20=False,dynamic_box=False,
                    map_rejoin=True,continue_follow_after_rejoin=False,
                    follow_only=True)
        values.update(overrides)
        return nav2_leader_mode_issue(**values)

    def test_follow_only_nav2_mode_needs_no_avoidance_or_rejoin_flags(self):
        self.assertIsNone(self.check())

    def test_follow_only_rejects_continuous_avoidance_flags(self):
        self.assertIsNotNone(self.check(continuous20=True,dynamic_box=True))

    def test_follow_only_works_without_map_rejoin_flag(self):
        self.assertIsNone(self.check(map_rejoin=False))

    def test_run_is_required(self):
        self.assertIn('--run',self.check(run=False))

    def test_legacy_nav2_avoidance_mode_remains_supported(self):
        self.assertIsNone(self.check(continuous20=True,dynamic_box=True,
            continue_follow_after_rejoin=True,follow_only=False))

    def test_incomplete_legacy_avoidance_mode_is_rejected(self):
        self.assertIsNotNone(self.check(follow_only=False))


if __name__ == '__main__':
    unittest.main()
