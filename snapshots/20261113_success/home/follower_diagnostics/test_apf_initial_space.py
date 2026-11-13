import json,unittest
from pathlib import Path
from apf_pivot_candidate import APFPlanFailure,build_apf_pivot_plan

class Tests(unittest.TestCase):
    def test_recorded_rear_returns_explain_current_and_rotated_clearance(self):
        p=Path(__file__).parent/'logs/local_avoidance_trial_1793779076449623920.jsonl'
        rows=[json.loads(s) for s in p.read_text().splitlines()]
        row=next(r for r in rows if r['event']=='planning_input')
        with self.assertRaises(APFPlanFailure) as caught:build_apf_pivot_plan(row['points'],row['start'])
        d=caught.exception.apf_diagnostic
        self.assertAlmostEqual(d['current_body_clearance_m'],.33688275323833666)
        self.assertLess(d['nearest_point_base_m'][0],-.5)
        self.assertLess(d['nearest_point_base_m'][1],-.3)
        turn=next(a for a in d['attempts'] if a['heading_deg']==45)
        self.assertLess(turn['min_path_clearance_m'],.3)
        self.assertFalse(d['control_connected'])

if __name__=='__main__':unittest.main()
