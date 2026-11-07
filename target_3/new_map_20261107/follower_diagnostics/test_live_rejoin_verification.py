import unittest
from types import SimpleNamespace as S
from live_rejoin_verification import LiveRejoinVerification

class Tests(unittest.TestCase):
    def offer(self,c,i,x,y=0.,local_y=0.,path_y=0.,stamp=None):
        ns=100_000_000_000+i*100_000_000;s=ns if stamp is None else stamp
        t=S(sec=s//10**9,nanosec=s%10**9)
        inputs={'follower':(t,i*.1,(x,y,0.,0.,0.)),'path':(t,i*.1,[(0.,path_y),(5.,path_y)])}
        return c.check(inputs,ns,i*.1,(x,local_y,0.),(0.,0.),0.,True)
    def test_local_false_success_different_map_position_rejected(self):
        r=self.offer(LiveRejoinVerification(),1,3.,y=.5)
        self.assertFalse(r['verified']);self.assertEqual(r['reason'],'MAP_LOCAL_POSE_MISMATCH')
    def test_matching_pose_but_wrong_leader_path_is_not_rejoin(self):
        r=self.offer(LiveRejoinVerification(),1,3.,path_y=.4)
        self.assertFalse(r['verified']);self.assertEqual(r['reason'],'NOT_REJOINED_LIVE_PATH')
    def test_one_metre_of_measured_map_progress_required(self):
        c=LiveRejoinVerification()
        for i in range(51):r=self.offer(c,i,i*.02)
        self.assertTrue(r['verified']);self.assertAlmostEqual(r['forward_on_live_path_m'],1.)
    def test_two_metre_target_does_not_pass_at_one_metre(self):
        c=LiveRejoinVerification(required_distance_m=2.)
        for i in range(101):
            r=self.offer(c,i,i*.02)
            if i<100:self.assertFalse(r['verified'])
        self.assertTrue(r['verified'])
        self.assertAlmostEqual(r['forward_on_live_path_m'],2.)
        self.assertEqual(r['required_distance_m'],2.)
    def test_two_metre_target_resets_on_path_departure(self):
        c=LiveRejoinVerification(required_distance_m=2.)
        for i in range(51):self.offer(c,i,i*.02)
        self.offer(c,51,1.02,y=.2,local_y=.2)
        self.assertEqual(c.distance,0.)
    def test_map_jump_and_missing_inputs_reset_progress(self):
        c=LiveRejoinVerification();self.offer(c,0,0.)
        self.assertEqual(self.offer(c,1,.4)['reason'],'MAP_PROGRESS_DISCONTINUITY')
        self.assertEqual(c.distance,0.)
        self.assertFalse(c.check({},101_000_000_000,1.,(0,0,0),(0,0),0,True)['verified'])
    def test_repeated_old_map_sample_does_not_add_distance(self):
        c=LiveRejoinVerification();self.offer(c,0,0.)
        for i in range(1,5):r=self.offer(c,i,.05,stamp=100_000_000_000)
        self.assertEqual(r['forward_on_live_path_m'],0.)

if __name__=='__main__':unittest.main()
