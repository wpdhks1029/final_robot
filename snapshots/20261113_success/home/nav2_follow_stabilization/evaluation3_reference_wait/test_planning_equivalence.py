"""Pure offline checks. The original recorded evaluation remains untouched."""
import math
from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parent/'candidate/follower_diagnostics'))
import continuous_avoidance_candidate as fast
import continuous_avoidance_reference as old


class EquivalenceTests(unittest.TestCase):
    def test_switch_threshold_matches_original_on_both_sides(self):
        for angle in np.linspace(-math.pi,math.pi,81):
            for radius in (.119,.12-1e-13,np.nextafter(.12,0),.12,np.nextafter(.12,1),.12+1e-13,.121):
                dx,dy=radius*math.cos(angle),radius*math.sin(angle)
                self.assertEqual(fast.target_is_near(dx,dy),np.linalg.norm((dx,dy))<.12)
    def test_collision_failure_is_identical(self):
        path=[(i*.01,0.,0.) for i in range(50)]
        for cloud in ([(.4,0.)],[(.4,.49)],[(.4,.51)],[(10.,10.)]):
            with self.subTest(cloud=cloud):
                self.assertEqual(fast.replay(cloud,(0.,0.,0.),path,limit=5),old.replay(cloud,(0.,0.,0.),path,limit=5))
    def test_invalid_targets_and_curvature_rejections_are_identical(self):
        for path in ([(-1.,0.,0.)]*3,[(.01,.12,0.)]*3):
            self.assertEqual(fast.replay([(10.,10.)],(0.,0.,0.),path),old.replay([(10.,10.)],(0.,0.,0.),path))
    def test_deadline_failure_is_identical(self):
        path=[(i*.01,0.,0.) for i in range(50)]
        self.assertEqual(fast.replay([(10.,10.)],(0.,0.,0.),path,deadline=0),old.replay([(10.,10.)],(0.,0.,0.),path,deadline=0))
    def test_saved_baseline_is_preserved(self):
        import hashlib,json
        root=Path(__file__).resolve().parent
        source=Path('/home/sunuk/final_robot/target_3/new_map_20261107/follower_diagnostics')
        for name,digest in json.loads((root/'baseline_manifest.json').read_text()).items():
            self.assertEqual(hashlib.sha256((source/name).read_bytes()).hexdigest(),digest,name)


if __name__=='__main__':unittest.main()
