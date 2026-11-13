import json,math,unittest
from pathlib import Path
from directed_path_tracker import DirectedPathTracker

class Tests(unittest.TestCase):
    def test_recorded_reverse_prefix_is_rejected_not_silently_trimmed(self):
        rows=map(json.loads,(Path(__file__).parent/'logs/local_avoidance_trial_1793784890865692866.jsonl').read_text().splitlines())
        r=next(r for r in rows if r['event']=='map_path_verification' and len(r.get('live_path_points',[]))>=5)
        with self.assertRaisesRegex(ValueError,'SHARP_REVERSAL'):
            DirectedPathTracker().select(r['live_path_points'],r['map_pose'])
    def test_arc_length_target_and_progress_remain_forward(self):
        c=DirectedPathTracker();pts=[(0,0),(.4,0),(.8,0),(1.2,0)]
        r=c.select(pts,(.1,.05,0),.5);self.assertAlmostEqual(r['target_map'][0],.6)
        r=c.select(pts,(.09,.05,0),.5);self.assertAlmostEqual(r['path_progress_m'],.1)
        r=c.select(pts+[(1.6,0)],(.15,.05,0),.5);self.assertGreater(r['path_progress_m'],.1)
    def test_path_reset_requires_new_session(self):
        c=DirectedPathTracker();c.select([(0,0),(1,0)],(0,0,0))
        with self.assertRaisesRegex(ValueError,'SESSION_CHANGED'):c.select([(0,.1),(1,.1)],(0,0,0))
    def test_smooth_u_curve_kept_despite_global_direction_reversal(self):
        pts=[(math.sin(i*math.pi/40),1-math.cos(i*math.pi/40)) for i in range(41)]
        c=DirectedPathTracker();r=c.select(pts,(0,0,0),.5)
        self.assertGreater(r['target_map'][0],0)
        for i in range(1,35):
            r=c.select(pts,(*pts[i],i*math.pi/40),.25)
        self.assertGreater(r['path_progress_m'],2.)
    def test_intersection_cannot_jump_to_far_future_branch(self):
        c=DirectedPathTracker();pts=[(-1,0),(0,0),(1,0),(1,1),(0,1),(0,0),(0,-1)]
        r=c.select(pts,(-.1,0,0),.2)
        r=c.select(pts,(0,0,0),.2)
        self.assertEqual(r['path_index'],0);self.assertLess(r['path_target_arc_m'],1.3)
    def test_map_jump_is_not_progress(self):
        c=DirectedPathTracker();c.select([(0,0),(5,0)],(0,0,0))
        with self.assertRaisesRegex(ValueError,'MAP_JUMP'):c.select([(0,0),(5,0)],(1,0,0))

if __name__=='__main__':unittest.main()
