import unittest
import numpy as np
from check_scan_turn_point_plane_offline import fit

class Tests(unittest.TestCase):
    def test_known_room_rotation_and_translation(self):
        angles=np.linspace(-np.pi,np.pi,500,endpoint=False)
        radius=np.minimum(4/np.maximum(abs(np.cos(angles)),1e-9),3/np.maximum(abs(np.sin(angles)),1e-9))
        dst=np.column_stack((radius*np.cos(angles),radius*np.sin(angles)))
        for theta in (.03,-.03,0.):
            rotation=np.array([[np.cos(theta),-np.sin(theta)],[np.sin(theta),np.cos(theta)]])
            src=(dst-np.array([.02,-.015]))@rotation
            measured,error,n=fit(src,dst)
            self.assertAlmostEqual(measured,theta,places=5)
            self.assertLess(error,.0001)

if __name__=='__main__':unittest.main()
