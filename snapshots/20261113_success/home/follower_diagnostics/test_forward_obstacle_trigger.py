import math,unittest
import numpy as np
from forward_obstacle_trigger import front_obstacle_count

class Tests(unittest.TestCase):
    def test_distance_is_measured_from_front_edge(self):
        self.assertEqual(front_obstacle_count([(0.96,0.),(0.98,0.),(.26,0.),(-.5,0.)],(0.,0.,0.)),1)
    def test_rotated_and_translated_robot_uses_body_forward_direction(self):
        self.assertEqual(front_obstacle_count([(2.,3.96),(2.,3.98)],(2.,3.,math.pi/2)),1)
    def test_side_corridor_and_empty_cloud(self):
        self.assertEqual(front_obstacle_count([(.9,.66),(.9,.64)],(0.,0.,0.)),1)
        self.assertEqual(front_obstacle_count([],(0.,0.,0.)),0)
    def test_cloud_keeps_far_obstacle_for_independent_collision_check(self):
        cloud=np.array([(0.96,0.),(1.5,.6)]);before=cloud.copy()
        self.assertEqual(front_obstacle_count(cloud,(0.,0.,0.)),1)
        np.testing.assert_array_equal(cloud,before)

if __name__=='__main__':unittest.main()
