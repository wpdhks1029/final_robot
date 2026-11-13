import unittest
from types import SimpleNamespace as S

from nav2_goal_state import has_active_goal


class Nav2GoalStateTests(unittest.TestCase):
    def test_waits_until_accepted_or_executing_goal(self):
        for state in (1, 2):
            with self.subTest(state=state):
                self.assertTrue(has_active_goal([S(status=state)]))

    def test_terminal_or_empty_status_does_not_arm(self):
        for states in ([], [S(status=3)], [S(status=4)], [S(status=5)], [S(status=6)],
                       [S(status=4), S(status=5)]):
            with self.subTest(states=states):
                self.assertFalse(has_active_goal(states))


if __name__ == '__main__':
    unittest.main()
