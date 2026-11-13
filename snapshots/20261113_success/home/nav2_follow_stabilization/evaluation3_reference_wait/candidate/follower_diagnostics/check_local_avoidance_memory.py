#!/usr/bin/env python3
"""Full local controller + phase-bounded contract with memory transport only."""
import json,time,unittest
from pathlib import Path
from test_local_stop_turn_go import MemoryClosedLoop

def main():
    test=MemoryClosedLoop('test_full_measured_feedback_phase_contract_and_rejoin')
    result=unittest.TestResult();test.run(result)
    if not result.wasSuccessful():
        for _,message in result.failures+result.errors:print(message)
        return 1
    summary=test.summary;folder=Path(__file__).parent/'logs'
    target=folder/f'local_avoidance_memory_{time.time_ns()}.json'
    target.write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(summary,indent=2));print('JSON='+str(target));return 0
if __name__=='__main__':raise SystemExit(main())
