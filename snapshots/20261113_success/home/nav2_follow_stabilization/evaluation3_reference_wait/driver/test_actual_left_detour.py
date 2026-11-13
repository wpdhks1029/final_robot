import json,math,unittest
from pathlib import Path
import numpy as np
from temporary_detour_candidate import plan,Settings
from watch_temporary_detour_shadow import review_path


class Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.fixture=json.loads((Path(__file__).parent/'fixtures/left_detour_actual_20261103.json').read_text())
        f=cls.fixture
        cls.result=plan(f['points'],Settings(),side=1,initial_cross=f['initial_cross'],initial_yaw=f['initial_yaw'],objective='time',nominal_speed=.06)
    def test_current_recorded_scene_has_left_path_without_translation(self):
        r=self.result
        self.assertEqual(r['reason'],'DETOUR_PLANNED')
        self.assertEqual(r['side'],1)
        self.assertAlmostEqual(r['obstacle_near_x_m'],.9748647780867736)
        self.assertAlmostEqual(r['observed_center_y_m'],-.03447800641139423)
        self.assertGreater(r['path'][-1][0]-r['join_x_m'],1.)
    def test_whole_recorded_scan_retained_for_clearance_review(self):
        review=review_path(self.result['path'],self.fixture['points'])
        self.assertEqual(review['reason'],'LIVE_PATH_GEOMETRY_CLEAR')
        self.assertGreater(review['min_path_clearance_m'],.32)
    def test_independent_intermediate_poses_keep_thirty_centimeters(self):
        p=self.result['path'];points=np.asarray(self.fixture['points']);minimum=999.
        for i in range(1,len(p)):
            # 끝점 사이 자세를 독립적으로 보간해 모든 관측점을 검사한다.
            for fraction in (.25,.5,.75):
                x=p[i-1][0]+fraction*(p[i][0]-p[i-1][0]);y=p[i-1][1]+fraction*(p[i][1]-p[i-1][1])
                a=p[i-1][2]+fraction*(p[i][2]-p[i-1][2])
                dx=points[:,0]-x;dy=points[:,1]-y;co,si=math.cos(a),math.sin(a)
                d=np.hypot(np.maximum(abs(co*dx+si*dy)-.27,0),np.maximum(abs(-si*dx+co*dy)-.3,0)).min()
                minimum=min(minimum,float(d))
        self.assertGreater(minimum,.30)
