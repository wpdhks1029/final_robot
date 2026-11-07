import math,unittest
from live_follow_reference import live_follow_reference

class Tests(unittest.TestCase):
    def test_follow_check_uses_current_segment_despite_old_line_displacement(self):
        points=[(0.,0.),(1.,0.),(2.,.2),(3.,.4),(4.,.6)]
        origin,heading,cross,selection=live_follow_reference(points,(2.5,.32,.2,0.,0.),(3.8,.56,.2,0.,0.))
        self.assertAlmostEqual(heading,math.atan2(.2,1.));self.assertLess(abs(cross),.03)
        self.assertEqual(selection['path_index'],2)
    def test_misaligned_far_or_reverse_poses_rejected(self):
        for follower,leader in [((1.,.3,0.),(3.,0.,0.)),((1.,0.,1.),(3.,0.,0.)),((1.,0.,0.),(.5,0.,0.))]:
            with self.assertRaises(ValueError):live_follow_reference([(0.,0.),(4.,0.)],follower,leader)
    def test_single_point_or_reverse_path_not_accepted(self):
        for path in ([(0.,0.)],[(0.,0.),(1.,0.),(0.,0.)]):
            with self.assertRaises(ValueError):live_follow_reference(path,(0.,0.,0.),(3.,0.,0.))

if __name__=='__main__':unittest.main()
