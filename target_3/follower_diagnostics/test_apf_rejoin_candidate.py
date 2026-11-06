import math,unittest
from apf_case3_candidate import Settings
from apf_rejoin_candidate import StraightRejoin
from apf_guard_connection import APFGuardCore


class Tests(unittest.TestCase):
    def test_separate_objects_do_not_merge_width(self):
        m=self.make()
        points=[(1.3,-.05+i*.02) for i in range(11)]+[(1.65,.45+i*.02) for i in range(11)]
        r=m.command(.04,0.,points,(0.,0.,0.),[(0.,0.),(3.,0.)],1)
        self.assertEqual(r['reason'],'APF_REJOIN_CANDIDATE')
    def test_continuous_wide_object_still_refused(self):
        r=self.make().command(.04,0.,[(1.3,-.3+i*.02) for i in range(31)],(0.,0.,0.),[(0.,0.),(3.,0.)],1)
        self.assertEqual(r['reason'],'STOP_APF_OBSTACLE_WIDTH')
    def make(self):return StraightRejoin(Settings(influence=1.5,lidar_height=.25,visibility_confirmed=True),-1)
    def points(self):return [(1.6,.05+i*.02) for i in range(11)]
    def start(self,m):return m.command(.04,0.,self.points(),(0.,0.,0.),[(0.,0.),(3.,0.)],1)
    def test_early_detection_right_forward(self):
        r=self.start(self.make());self.assertEqual(r['phase'],'AVOID');self.assertLess(r['w'],0)
        self.assertGreaterEqual(r['v'],abs(r['w'])*.560/2)
    def test_sparse_cluster_refused(self):
        r=self.make().command(.04,0.,[(1.,0.)],(0.,0.,0.),[(0.,0.),(3.,0.)],1)
        self.assertEqual(r['reason'],'STOP_APF_OBSTACLE_CLUSTER_INSUFFICIENT')
    def test_curve_refused(self):
        r=self.make().command(.04,0.,self.points(),(0.,0.,0.),[(0.,0.),(1.,0.),(2.,1.)],1)
        self.assertEqual(r['reason'],'STOP_APF_CURVED_REFERENCE')
    def test_pose_jump_latches(self):
        m=self.make();self.start(m)
        r=m.command(.04,0.,self.points(),(.3,0.,0.),[(0.,0.),(3.,0.)],2)
        self.assertEqual(r['reason'],'STOP_APF_POSE_JUMP')
        self.assertEqual(m.command(.04,0.,self.points(),(0.,0.,0.),[(0.,0.),(5.,0.)],3)['reason'],'STOP_APF_POSE_JUMP')
    def test_same_pose_not_integrated_twice(self):
        m=self.make();a=self.start(m);b=self.start(m)
        self.assertEqual(a,b)
    def test_backwards_pose_refused(self):
        m=self.make();self.start(m)
        self.assertEqual(m.command(.04,0.,self.points(),(0.,0.,0.),[(0.,0.),(5.,0.)],0)['reason'],'STOP_APF_REFERENCE_INVALID')
    def test_pass_requires_turn_clearance_then_rejoin_and_one_meter(self):
        m=self.make();self.start(m);stamp=1
        # 위치를 0.1m씩 진행시켜 기하 판정만 검사한다.
        for i in range(1,29):
            stamp+=1;x=i*.1;y=-min(.6,i*.1)
            r=m.command(.04,0.,[(0.,2.)],(x,y,0.),[(0.,0.),(5.,0.)],stamp)
            if i==21:self.assertEqual(r['phase'],'PASS')
        self.assertEqual(r['phase'],'REJOIN')
        for i in range(1,7):
            stamp+=1;r=m.command(.04,0.,[(0.,2.)],(2.8+i*.05,-.6+i*.1,0.),[(0.,0.),(5.,0.)],stamp)
        self.assertEqual(r['phase'],'FOLLOW_AFTER_REJOIN')
        for i in range(1,12):
            stamp+=1;r=m.command(.04,0.,[(0.,2.)],(3.1+i*.1,0.,0.),[(0.,0.),(5.,0.)],stamp)
        self.assertEqual(r['phase'],'GEOMETRIC_COMPLETE_CANDIDATE')
        self.assertFalse(r['evaluation_pass'])
    def test_live_guard_requires_navigation(self):
        c=APFGuardCore(settings=Settings(influence=1.5,lidar_height=.25,visibility_confirmed=True),preferred_side=-1,enable_rejoin=True)
        c.scan=(1_000_000_000,1.,.8,10);c.raw=(1.,.04,0.);c.points=self.points()
        self.assertEqual(c.command(1_100_000_000,1.1)[2],'STOP_APF_NAVIGATION_MISSING')
