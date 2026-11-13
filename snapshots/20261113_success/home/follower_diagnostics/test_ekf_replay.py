import unittest
from replay_ekf_diagnostic import fill_stationary_gaps


def raw(t,v=0):
    return dict(topic='/follower/odom_raw',rx_s=str(t),stamp_ns=str(10**9+int(t*10**9)),
                age_s='.01',vx=str(v),vy='0',wz='0')


class Tests(unittest.TestCase):
    def test_stationary_gap_only(self):
        rows=[raw(0),raw(1)]
        result,count=fill_stationary_gaps(rows)
        self.assertEqual(count,9);self.assertEqual(len(rows),2)
        self.assertEqual(result[0]['rx_s'],'0');self.assertEqual(result[-1]['rx_s'],'1')

    def test_nonzero_gap_refused(self):
        with self.assertRaises(ValueError):fill_stationary_gaps([raw(0,.01),raw(1)])

    def test_regular_samples_preserved(self):
        rows=[raw(0),raw(.1),raw(.2)]
        result,count=fill_stationary_gaps(rows)
        self.assertEqual(count,0);self.assertEqual(result,rows)


if __name__=='__main__':unittest.main()
