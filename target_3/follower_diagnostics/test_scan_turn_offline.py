import unittest
import numpy as np
from check_scan_turn_offline import fit

class Tests(unittest.TestCase):
    def test_known_positive_and_negative_relative_rotation(self):
        rng=np.random.default_rng(27);dst=rng.uniform(-2,2,(180,2))
        for theta in (.025,-.025):
            r=np.array([[np.cos(theta),-np.sin(theta)],[np.sin(theta),np.cos(theta)]])
            t=np.array([.015,-.01]);src=(dst-t)@r
            measured,error,n=fit(src,dst)
            self.assertAlmostEqual(measured,theta,places=6)
            self.assertLess(error,1e-6)
    def test_identical_scans_do_not_invent_rotation(self):
        rng=np.random.default_rng(8);p=rng.uniform(-3,3,(150,2))
        theta,error,n=fit(p,p)
        self.assertAlmostEqual(theta,0.,places=9)
        self.assertLess(error,1e-8)

if __name__=='__main__':unittest.main()
