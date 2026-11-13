import unittest
from follower_obstacle_preview import ObstacleCore,guard_status,status_event_key


class Tests(unittest.TestCase):
    def test_stamped_cause_not_replaced_by_plain_cause(self):
        c=ObstacleCore();c.scan=(1_000_000_000,1.,.58,300)
        s=guard_status(c,1_200_000_000,'PREVIEW_SLOW','STOP_UPSTREAM',1_100_000_000,0.,0.)
        self.assertEqual(s['plain_reason'],'PREVIEW_SLOW')
        self.assertEqual(s['stamped_reason'],'STOP_UPSTREAM')
        self.assertAlmostEqual(s['scan_age_s'],.2)
        self.assertEqual(s['clearance_m'],.58)

    def test_missing_sources_are_null_not_fresh(self):
        s=guard_status(ObstacleCore(),1_200_000_000,'STOP_SCAN_OR_TF_INVALID','STOP_SCAN_STALE_OR_INVALID',0,0.,0.)
        for key in ('clearance_m','scan_age_s','source_age_s'):
            self.assertIsNone(s[key])

    def test_inside_point_is_recorded_even_with_upstream_stopped(self):
        c=ObstacleCore();c.scan=(1_000_000_000,1.,.55,400)
        before=status_event_key(c,'STOP_UPSTREAM','STOP_UPSTREAM',1_000_000_000,0.,0.)
        c.scan=(1_100_000_000,1.1,0.,400)
        c.nearest_points=[(0.,56,-141.,.315,.015,-.198)]
        after=status_event_key(c,'STOP_UPSTREAM','STOP_UPSTREAM',1_100_000_000,0.,0.)
        self.assertNotEqual(before,after)
        state=guard_status(c,1_200_000_000,'STOP_UPSTREAM','STOP_UPSTREAM',1_100_000_000,0.,0.)
        self.assertEqual(state['clearance_zone'],'inside')
        self.assertEqual(state['nearest_points'][0]['base_y_m'],-.198)
        self.assertEqual(state['nearest_points'][0]['index'],56)


if __name__=='__main__':unittest.main()
