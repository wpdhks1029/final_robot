import unittest
from test_continuous20 import CoreTests

class Tests(unittest.TestCase):
    def test_gap_stays_invalid_but_new_measurement_pair_recovers(self):
        c,ns=CoreTests().core()
        later=ns+310_000_000
        c.raw(later,1.31,(0.,0.,0.,0.,0.));c.filtered(later,1.31,(0.,0.,0.,0.,0.));c.imu(later,1.31,(0.,0.,0.,0.,0.))
        c.offer_scan(ns+150_000_000,1.31,[(2.,0.)])
        with self.assertRaisesRegex(ValueError,'FILTERED_ODOM_SCAN_BRACKET_MISSING'):c.scene(later,1.31)
        self.assertEqual(c.last_alignment_failure['kind'],'POSE_HISTORY_GAP')
        c.offer_scan(later,1.32,[(2.,0.)])
        scene=c.scene(later+10_000_000,1.32)
        self.assertEqual(scene['scan_stamp'],later)
        self.assertIsNone(c.last_alignment_failure)
    def test_new_scan_does_not_make_stale_odometry_fresh(self):
        c,ns=CoreTests().core();c.offer_scan(ns+900_000_000,1.9,[(2.,0.)])
        with self.assertRaises(ValueError):c.scene(ns+900_000_000,1.9)

if __name__=='__main__':unittest.main()
