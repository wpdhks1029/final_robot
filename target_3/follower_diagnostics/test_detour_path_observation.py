import unittest
from directed_path_tracker import DirectedPathTracker

class Tests(unittest.TestCase):
    def test_detour_observations_prevent_sparse_selection_false_jump(self):
        t=DirectedPathTracker();path=[(i*.1,0.) for i in range(51)]
        first=t.select(path,(0.,0.,0.),.35)
        # During avoidance there is no Pure Pursuit selection for over a metre.
        for i in range(1,31):t.observe_position((i*.04,min(.3,i*.03),0.))
        last=t.select(path,(1.2,.3,0.),.35)
        self.assertGreater(last['path_progress_m'],1.)
        self.assertEqual(first['path_session'],last['path_session'])
    def test_real_single_update_jump_is_still_rejected(self):
        t=DirectedPathTracker();t.observe_position((0.,0.,0.))
        with self.assertRaisesRegex(ValueError,'PATH_FOLLOWER_MAP_JUMP'):t.observe_position((.3,0.,0.))
    def test_unobserved_jump_and_path_reset_still_rejected(self):
        t=DirectedPathTracker();path=[(0.,0.),(2.,0.)];t.select(path,(0.,0.,0.))
        with self.assertRaisesRegex(ValueError,'PATH_FOLLOWER_MAP_JUMP'):t.select(path,(1.,0.,0.))
        t.observe_position((.1,0.,0.))
        with self.assertRaisesRegex(ValueError,'PATH_SESSION_CHANGED'):t.select([(0.,0.),(3.,0.)],(.1,0.,0.))

if __name__=='__main__':unittest.main()
