import unittest
from check_source_budget import BudgetTrace,REMOTE_MARGIN_NS


class Tests(unittest.TestCase):
    def test_remote_margin_preserved_and_shared_stamp_owners_retained(self):
        t=BudgetTrace()
        t.add('leader',1_000_000_000,1_100_000_000,3.)
        t.add('path',1_000_000_000,1_100_000_000,3.)
        t.add('guard',1_000_000_000-REMOTE_MARGIN_NS,1_260_000_000,3.2)
        r=t.report(0.)['guard']
        self.assertEqual(r['over_0_3s'],1)
        self.assertEqual(r['near_limit_owners'],{'leader+path':1})
        self.assertAlmostEqual(r['near_limit_examples'][0]['effective_age_s'],.31)
        self.assertAlmostEqual(r['near_limit_examples'][0]['raw_age_when_first_observed_s']['leader'],.1)

    def test_follower_and_scan_not_adjusted(self):
        t=BudgetTrace();t.add('follower',1_000_000_000,1_050_000_000,3.)
        t.add('scan',900_000_000,1_050_000_000,3.)
        self.assertEqual(t.owners(1_000_000_000),['follower'])
        self.assertEqual(t.owners(900_000_000),['scan'])

    def test_unmatched_not_guessed_from_latest_input(self):
        t=BudgetTrace();t.add('scan',900_000_000,1_050_000_000,3.)
        t.add('guard',800_000_000,1_110_000_000,3.1)
        self.assertEqual(t.report(0.)['guard']['near_limit_owners'],{'unmatched':1})

    def test_zero_stamp_and_startup_separate(self):
        t=BudgetTrace();t.add('guard',1,1_000_000_000,1.)
        t.add('guard',0,3_000_000_000,3.)
        r=t.report(0.)['guard']
        self.assertEqual(r['received'],1);self.assertEqual(r['zero_stamp'],1)
        self.assertEqual(r['over_0_3s'],0)

    def test_republication_does_not_refresh_parent_observation(self):
        t=BudgetTrace();t.add('follower',1_000_000_000,1_050_000_000,3.)
        t.add('follower',1_000_000_000,1_200_000_000,3.1)
        self.assertAlmostEqual(t.parents[1_000_000_000]['follower']['age_s'],.05)

    def test_origin_can_arrive_after_guard_in_observer(self):
        t=BudgetTrace();t.add('guard',1_000_000_000,1_300_000_000,3.)
        t.add('scan',1_000_000_000,1_310_000_000,3.01)
        self.assertEqual(t.report(0.)['guard']['near_limit_owners'],{'scan':1})


if __name__=='__main__':unittest.main()
