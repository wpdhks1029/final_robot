#!/usr/bin/env python3
"""Recorded current scene through real pure control/contract + memory motor.
No ROS initialization, sockets, serial ports, services or live publishers.
"""
import argparse,json,time
from pathlib import Path
from test_apf_pivot_candidate import APFMemoryClosedLoop

def main():
    root=Path(__file__).resolve().parent
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--shadow',type=Path,required=True);a=p.parse_args()
    rows=json.loads(a.shadow.read_text())['rows']
    row=next(r for r in rows if r.get('reason')=='APF_PIVOT_STEP_CHECKED')
    class CurrentScene(APFMemoryClosedLoop):
        def data_for_test(self):return dict(points=row['points'],start=[0.,0.,0.])
    case=CurrentScene('test_full_measured_feedback_phase_contract_and_rejoin')
    case.test_full_measured_feedback_phase_contract_and_rejoin()
    summary=dict(case.summary,source=str(a.shadow),controller_kind='APF_PIVOT_STEP_V1')
    path=root/'logs'/f'apf_pivot_memory_{time.time_ns()}.json'
    path.write_text(json.dumps(summary,allow_nan=False)+'\n');print(json.dumps(summary));print('JSON='+str(path))

if __name__=='__main__':main()
