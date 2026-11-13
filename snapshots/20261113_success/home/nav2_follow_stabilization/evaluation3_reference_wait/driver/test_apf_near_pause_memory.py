"""Actual APF/driver contract in memory with two synthetic scan-return bursts."""
import json,math,unittest
from pathlib import Path
from local_near_return_pause import NearReturnPause
import test_apf_pivot_candidate as _apf_tests

class Tests(_apf_tests.APFMemoryClosedLoop):
    def data_for_test(self):
        path=Path(__file__).parent/'logs/local_avoidance_trial_1793774222043810461.jsonl'
        rows=[json.loads(s) for s in path.read_text().splitlines()]
        p=next(r for r in rows if r['event']=='planning_input')
        self.pause=NearReturnPause();self.events=[]
        return dict(points=p['points'],start=p['start'])
    def scan_points_for_test(self,points,elapsed):
        if 7.0<=elapsed<7.1 or 7.3<=elapsed<7.4:
            a=math.radians(-79.5);points=points+[(.26+.334*math.cos(a),.334*math.sin(a))]
        return points
    def control_for_test(self,core,result,mono,motor):
        r=self.pause.apply(result,mono,motor)
        event=r.get('near_return_pause',{}).get('event')
        if event in ('started','resumed'):self.events.append(event)
        if self.pause.active:self.assertEqual((r['v'],r['w']),(0.,0.))
        return r
    def test_full_measured_feedback_phase_contract_and_rejoin(self):
        super().test_full_measured_feedback_phase_contract_and_rejoin()
        self.assertIn('started',self.events);self.assertIn('resumed',self.events)
        self.assertEqual(self.events,['started','resumed'])

if __name__=='__main__':unittest.main()
