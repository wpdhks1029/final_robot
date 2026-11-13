import unittest
from io import StringIO
from types import SimpleNamespace as S
from unittest.mock import patch
from leader_short_forward_trial import feedback_ok, main,wait_for_start


class Tests(unittest.TestCase):
    def sample(self):
        return (S(header=S(frame_id='map', stamp=S(sec=10, nanosec=0)),
                  twist=S(twist=S(linear=S(x=.05), angular=S(z=0.)))), 1.)

    def test_fresh(self):
        self.assertTrue(feedback_ok(self.sample(), 10_100_000_000, 1.1))

    def test_stale_future_missing(self):
        self.assertFalse(feedback_ok(None, 10_100_000_000, 1.1))
        self.assertFalse(feedback_ok(self.sample(), 10_400_000_000, 1.1))
        self.assertFalse(feedback_ok(self.sample(), 9_900_000_000, 1.1))
        self.assertFalse(feedback_ok(self.sample(), 10_100_000_000, 1.4))

    def test_motion_bounds(self):
        for field, value in [('linear', .13), ('angular', .11), ('linear', float('nan'))]:
            sample = self.sample()
            vector = getattr(sample[0].twist.twist, field)
            setattr(vector, 'x' if field == 'linear' else 'z', value)
            self.assertFalse(feedback_ok(sample, 10_100_000_000, 1.1))

    def test_default_no_ros(self):
        with patch('sys.argv', ['trial']):
            self.assertEqual(main(), 0)

    def test_wait_keeps_receiving_before_start(self):
        spins=[];ready=iter([False,False,True])
        self.assertTrue(wait_for_start(lambda:spins.append(1),StringIO('START\n'),lambda:next(ready)))
        self.assertEqual(len(spins),3)

    def test_eof_and_other_input_do_not_start(self):
        for text in ('','STOP\n'):
            self.assertFalse(wait_for_start(lambda:None,StringIO(text),lambda:True))


if __name__ == '__main__':
    unittest.main()
