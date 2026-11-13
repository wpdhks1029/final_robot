import unittest
from apf_case3_candidate import Settings
from apf_guard_connection import APFGuardCore,swept_clear
from follower_stamped_preview import StampedGuard


class Tests(unittest.TestCase):
    def core(self,points):
        c=APFGuardCore(settings=Settings(lidar_height=.25,visibility_confirmed=True),preferred_side=-1)
        c.scan=(1_000_000_000,1.,min(c.clearance(*p) for p in points),len(points))
        c.raw=(1.,.04,0.);c.points=points
        return c

    def test_avoidance_then_existing_slowdown(self):
        c=self.core([(1.04,.15),(-.62,0.)])
        v,w,r=c.command(1_100_000_000,1.1)
        self.assertGreater(v,0);self.assertLess(v,.024);self.assertLess(w,0)
        self.assertEqual(c.raw,(1.,.04,0.))
        self.assertEqual(r,'PREVIEW_SLOW')

    def test_hard_stop_all_directions(self):
        c=self.core([(1.,0.),(-.5,0.)])
        self.assertEqual(c.command(1_100_000_000,1.1),(0.,0.,'STOP_FOOTPRINT_CLEARANCE'))

    def test_stale_scan_and_upstream_stop(self):
        c=self.core([(1.,0.)]);self.assertEqual(c.command(2_000_000_000,2.)[:2],(0.,0.))
        c.raw=(1.,0.,0.);self.assertEqual(c.command(1_100_000_000,1.1)[2],'STOP_UPSTREAM')

    def test_stamped_timeout_not_refreshed_by_apf(self):
        c=self.core([(1.,0.)]);g=StampedGuard(c);g.source=(1_000_000_000,1.)
        self.assertEqual(g.command(1_310_000_000,1.31)[3],'STOP_SOURCE_STALE')

    def test_pass_stops_until_rejoin_connected(self):
        c=self.core([(1.,0.)]);c.command(1_100_000_000,1.1)
        c.points=[(0.,.8)];c.scan=(1_000_000_000,1.,.5,1)
        self.assertEqual(c.command(1_100_000_000,1.1)[2],'STOP_APF_REJOIN_NOT_CONNECTED')

    def test_swept_rectangle_catches_future_front_collision(self):
        self.assertFalse(swept_clear([(.59,0.)],.04,0.,Settings()))
        self.assertTrue(swept_clear([(2.,0.)],.04,0.,Settings()))

    def test_recorded_roundoff_boundary_has_strict_forward_margin(self):
        from unittest.mock import patch
        import json
        c=self.core([(1.,0.)])
        # 실물 실패 로그의 감속 후 경계값을 그대로 재현한다.
        bad_v=.019646671394219436;bad_w=-.0701666835507837
        self.assertLess(bad_v-abs(bad_w)*.560/2,0.)
        with patch('apf_guard_connection.ObstacleCore.command',return_value=(bad_v,bad_w,'PREVIEW_SLOW')):
            v,w,_=c.command(1_100_000_000,1.1)
        v,w=json.loads(json.dumps([v,w]))
        self.assertGreater(v-abs(w)*.560/2,0.)
