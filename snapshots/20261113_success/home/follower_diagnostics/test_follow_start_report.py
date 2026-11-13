import unittest
from check_follow_start import speed_summary


class Tests(unittest.TestCase):
    def test_threshold_requires_strictly_above(self):
        report=speed_summary([0.,.03,.031,.05,0.])
        self.assertEqual(report['maximum'],.05)
        self.assertEqual(report['above_stop_threshold'],2)

    def test_empty_and_invalid(self):
        self.assertIsNone(speed_summary([])['maximum'])
        self.assertEqual(speed_summary([float('nan'),float('inf')])['samples'],0)


if __name__=='__main__':unittest.main()
