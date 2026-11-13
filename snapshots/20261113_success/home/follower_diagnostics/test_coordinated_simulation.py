import math,unittest
from replay_recorded_detour import coordinated_speed


class Tests(unittest.TestCase):
    def test_leader_ahead_slows_instead_of_running_away(self):
        self.assertEqual(coordinated_speed(3.,0.,0.,.03,1.5),.012)
    def test_small_gap_never_accelerates_above_validated_leader_speed(self):
        self.assertEqual(coordinated_speed(.8,0.,0.,.06,1.5),.04)
    def test_lateral_motion_does_not_count_as_forward_progress(self):
        self.assertAlmostEqual(coordinated_speed(1.5,0.,math.pi/2,.04,1.5),.012)
    def test_stop_overrides_positive_speed_floor(self):
        self.assertEqual(coordinated_speed(1.5,0.,0.,.04,1.5,enabled=False),0.)
    def test_nonfinite_state_rejected(self):
        with self.assertRaises(ValueError):coordinated_speed(float('nan'),0.,0.,.04,1.5)
