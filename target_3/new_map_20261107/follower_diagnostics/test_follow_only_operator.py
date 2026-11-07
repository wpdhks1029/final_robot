import json,math,sys,tempfile,unittest
from pathlib import Path
from types import SimpleNamespace as S
from unittest.mock import patch
from test_local_avoidance_operator import Flow
import run_local_avoidance_trial as runner
from directed_path_tracker import DirectedPathTracker

class Tests(unittest.TestCase):
    def flow(self,plan_only=False):
        f=Flow(plan_only=plan_only);modules,core=f.modules();Base=modules['follower_pursuit_preview'].PreviewCore
        class Nav(Base):
            def __init__(self):super().__init__();self.path_tracker=DirectedPathTracker();self.lookahead=.8;self.heading_limit=math.pi/4;self.last_selection=None
            def command(self,*args):
                self.last_selection=self.path_tracker.select(self.inputs['path'][2],self.inputs['follower'][2],.8)
                return super().command(*args)
        modules['tracked_pursuit_candidate']=S(TrackedPreviewCore=Nav)
        def submit(fn,*args):return S(done=lambda:True,result=lambda:fn(*args))
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);ref=root/'reference.json';ref.write_text('{}')
            with patch.dict(sys.modules,modules),patch.dict('os.environ',{'ROS_DOMAIN_ID':'10'}),patch('sys.argv',['local','--plan-only' if plan_only else '--run','--ack-clear-both-power-cut-ready','--reference-plan',str(ref),'--pose-source','filtered','--apf-pivot','--gentle-pivot','--map-rejoin','--follow-only']),patch.object(runner,'ROOT',root),patch.object(runner,'LocalCore',core),patch.object(runner,'ThreadPoolExecutor',return_value=S(submit=submit,shutdown=lambda **kw:None)),patch.object(runner,'reference_from_plan',return_value=((0,0),0.,0.,0.)),patch.object(runner.time,'monotonic',side_effect=lambda:f.t),patch.object(runner,'wait_for_start',return_value=True):
                code=runner.main()
            rows=list(map(json.loads,next((root/'logs').glob('*.jsonl')).read_text().splitlines()))
        return f,code,rows
    def test_follow_check_stops_after_ten_seconds_with_both_zeros(self):
        f,code,rows=self.flow();self.assertEqual(code,0)
        end=rows[-1];self.assertEqual(end['reason'],'FOLLOW_CHECK_TIME_LIMIT');self.assertTrue(end['follower_zero_confirmed']);self.assertTrue(end['leader_zero_confirmed'])
        arm=next(e[1] for e in f.events if e[0]=='arm');disarm=next(e[1] for e in f.events if e[0]=='disarm')
        self.assertLessEqual(disarm-arm,10.1)
        self.assertTrue(all(e[1]['w']==0. for e in f.events if e[0]=='envelope'))
    def test_plan_only_has_no_publishers_or_arm(self):
        f,code,rows=self.flow(True);self.assertEqual(code,0)
        self.assertFalse(any(e[0] in ('arm','leader','envelope') for e in f.events))
        self.assertTrue(any(r['event']=='follow_check_preview' for r in rows))

if __name__=='__main__':unittest.main()
