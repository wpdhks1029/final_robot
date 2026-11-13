"""Fake ROS wiring: follower-only movement cap never starts the leader."""
import json,sys,tempfile,unittest
from pathlib import Path
from types import SimpleNamespace as S
from unittest.mock import patch
from test_local_avoidance_operator import Flow
import run_local_avoidance_trial as runner

class Tests(unittest.TestCase):
    def test_frame_check_keeps_leader_zero_and_stops_at_twenty_cm(self):
        f=Flow();modules,Base=f.modules()
        class Core(Base):
            def command(self,*args,**kw):
                r=super().command(*args,**kw)
                if f.arm_ns is not None:r['pose']=[max(0.,(f.ros()-f.arm_ns)/1e9*.02),0.,.5]
                return r
        plan=dict(path=[[0,0,0,0],[1,0,0,0]],config={},geometry=dict(reason='LIVE_PATH_GEOMETRY_CLEAR'),attempts=[])
        pool=S(submit=lambda *args:S(done=lambda:True,result=lambda:plan),shutdown=lambda **kw:None)
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);ref=root/'reference.json';ref.write_text('{}')
            argv=['local','--run','--ack-clear-both-power-cut-ready','--reference-plan',str(ref),'--apf-pivot','--gentle-pivot','--pose-source','filtered','--frame-check-distance','.2']
            with patch.dict(sys.modules,modules),patch.dict('os.environ',{'ROS_DOMAIN_ID':'10'}),patch('sys.argv',argv),patch.object(runner,'ROOT',root),patch.object(runner,'LocalCore',Core),patch.object(runner,'ThreadPoolExecutor',return_value=pool),patch.object(runner,'reference_from_plan',return_value=((0,0),0.,0.,0.)),patch.object(runner.time,'monotonic',side_effect=lambda:f.t),patch.object(runner,'wait_for_start',return_value=True):
                runner.main()
            rows=list(map(json.loads,next((root/'logs').glob('*.jsonl')).read_text().splitlines()))
        self.assertTrue(all(e[1]==0 for e in f.events if e[0]=='leader'))
        end=rows[-1];self.assertFalse(end['leader_started']);self.assertTrue(end['frame_check_complete'])
        self.assertTrue(end['follower_zero_confirmed']);self.assertTrue(end['leader_zero_confirmed'])
        stopped=next(r for r in rows if r['event']=='stopped_control')
        self.assertLessEqual(stopped['control']['frame_check_travel_m'],.201)
        self.assertFalse(end['evaluation_pass'])

if __name__=='__main__':unittest.main()
