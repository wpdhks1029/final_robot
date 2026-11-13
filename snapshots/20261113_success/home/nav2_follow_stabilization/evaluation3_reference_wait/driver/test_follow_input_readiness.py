import unittest
from follow_input_readiness import InputReadiness


class ReadinessTests(unittest.TestCase):
    def test_requires_continuous_five_seconds(self):
        r = InputReadiness()
        for i in range(50):
            self.assertFalse(r.update(True, i / 10))
        self.assertTrue(r.update(True, 5.0))

    def test_invalid_input_resets_ready(self):
        r = InputReadiness()
        for i in range(51):
            r.update(True, i / 10)
        self.assertFalse(r.update(False, 5.1))
        self.assertFalse(r.update(True, 5.2))

    def test_observer_gap_cannot_count_as_stability(self):
        r = InputReadiness()
        r.update(True, 0)
        self.assertFalse(r.update(True, 20))

    def test_backward_clock_resets_window(self):
        r = InputReadiness()
        r.update(True, 10)
        self.assertFalse(r.update(True, 9))


if __name__ == '__main__':
    unittest.main()
