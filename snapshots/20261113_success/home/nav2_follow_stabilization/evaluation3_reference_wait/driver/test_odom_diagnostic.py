import unittest
from record_odom_diagnostic import summarize


class Tests(unittest.TestCase):
    def test_no_data(self):self.assertEqual(summarize([]),'NO DATA')
    def test_delta_and_gap(self):
        rows=[dict(rx_s=0.,x=1.,y=2.,vx=0.,age_s=.02),
              dict(rx_s=.1,x=1.003,y=2.,vx=.01,age_s=.03)]
        result=summarize(rows)
        self.assertIn('delta_xy=(0.003000,0.000000)',result)
        self.assertIn('max_rx_gap=0.1000s',result)
    def test_status_no_header(self):self.assertIn('n=1',summarize([dict(rx_s=1.,status='{}')]))


if __name__=='__main__':unittest.main()
