import unittest
from check_timing_pipeline import summarize


class Tests(unittest.TestCase):
    def test_empty(self):self.assertEqual(summarize([]),{'received':0})
    def test_old_and_future_are_not_hidden(self):
        r=summarize([(-.06,1.,10),(.1,1.1,20),(1.9,1.3,20)])
        self.assertEqual(r['age_over_0_3s'],1)
        self.assertEqual(r['future_over_0_05s'],1)
        self.assertEqual(r['repeated_stamp'],1)
        self.assertAlmostEqual(r['max_rx_gap_s'],.2)


if __name__=='__main__':unittest.main()
