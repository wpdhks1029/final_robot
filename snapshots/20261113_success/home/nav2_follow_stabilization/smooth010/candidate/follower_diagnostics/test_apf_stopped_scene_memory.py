"""Offline memory replay from recorded stop; no physical resume operation."""
import json,unittest
from pathlib import Path
import test_apf_pivot_candidate as _memory

class Tests(_memory.APFMemoryClosedLoop):
    @unittest.expectedFailure
    def test_full_measured_feedback_phase_contract_and_rejoin(self):
        # Retain the failing captured-scene regression explicitly. This scene
        # has not demonstrated a complete route; do not call it a passed trial.
        super().test_full_measured_feedback_phase_contract_and_rejoin()
    def data_for_test(self):
        p=Path(__file__).parent/'logs/local_avoidance_trial_1793774746886900549.jsonl'
        r=[json.loads(s) for s in p.read_text().splitlines()]
        self.original_plan=next(x['plan'] for x in r if x['event']=='checked_local_plan')
        stopped=next(x for x in r if x['event']=='stopped_control')
        return dict(points=stopped['points']+self.original_plan['box_boundary'],start=stopped['control']['pose'])
    def plan_for_test(self,d):return self.original_plan

if __name__=='__main__':unittest.main()
