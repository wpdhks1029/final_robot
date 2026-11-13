import copy,json,tempfile,unittest
from pathlib import Path
from types import SimpleNamespace as S
from check_temporary_detour import stationary_inputs,save_current_plan
from follower_temporary_detour_preview import load_plan,PLAN

class Tests(unittest.TestCase):
    def inputs(self):
        nav=S(inputs={k:(S(sec=100,nanosec=0),1.,(0.,0.,0.,0.,0.)) for k in ('leader','follower')})
        state=dict(mode='HARDWARE_CANDIDATE',armed=False,measured_rpm=[0,0],fault=None,feedback_fresh=True,io_failed=False)
        return nav,(state,1.)
    def test_fresh_disarmed_both_stationary_required(self):
        n,m=self.inputs();stationary_inputs(n,m,100_100_000_000,1.1)
        for key,value in [('armed',True),('measured_rpm',[1,0]),('fault','ERROR'),('feedback_fresh',False)]:
            bad=copy.deepcopy(m);bad[0][key]=value
            with self.assertRaises(ValueError):stationary_inputs(n,bad,100_100_000_000,1.1)
        with self.assertRaises(ValueError):stationary_inputs(n,m,100_400_000_000,1.4)
        n.inputs['leader']=(S(sec=100,nanosec=0),1.,(0.,0.,0.,.04,0.))
        with self.assertRaises(ValueError):stationary_inputs(n,m,100_100_000_000,1.1)
    def test_checked_actual_plan_saved_with_backup_and_loadable(self):
        data,_=load_plan(PLAN)
        with tempfile.TemporaryDirectory() as tmp:
            target=Path(tmp)/'plan.json';target.write_text('old plan')
            save_current_plan(data,target)
            new,digest=load_plan(target)
            self.assertEqual(new['box_forward_m'],0.)
            self.assertEqual(new['live_review']['reason'],'LIVE_PATH_GEOMETRY_CLEAR')
            self.assertEqual(next(Path(tmp).glob('previous_current_detour_*.json')).read_text(),'old plan')
    def test_blocked_new_scene_cannot_replace_old_plan(self):
        data,_=load_plan(PLAN);data['snapshot']['points'].append((0.,0.))
        with tempfile.TemporaryDirectory() as tmp:
            target=Path(tmp)/'plan.json';target.write_text('old plan')
            with self.assertRaises(ValueError):save_current_plan(data,target)
            self.assertEqual(target.read_text(),'old plan')

class ReferenceTests(unittest.TestCase):
    def test_short_live_path_can_validate_saved_line_without_lateral_shift(self):
        import math
        from check_temporary_detour import reference_from_plan
        from temporary_detour_candidate import Pose
        data,_=load_plan(PLAN);s=data['snapshot'];anchor=s['reference_origin'];h=s['reference_heading']
        co,si=math.cos(h),math.sin(h)
        def point(x,y):return (anchor[0]+co*x-si*y,anchor[1]+si*x+co*y)
        pose=Pose(*point(.1,.03),h)
        result=reference_from_plan(data,[point(.5,0),point(.51,0)],pose,(*point(1.7,0),h,0.,0.))
        self.assertAlmostEqual(result[2],.03)
        self.assertAlmostEqual(result[0][0],point(.1,0)[0]);self.assertAlmostEqual(result[0][1],point(.1,0)[1])
    def test_changed_map_line_or_heading_is_rejected(self):
        import math
        from check_temporary_detour import reference_from_plan
        from temporary_detour_candidate import Pose
        data,_=load_plan(PLAN);s=data['snapshot'];a=s['reference_origin'];h=s['reference_heading']
        co,si=math.cos(h),math.sin(h)
        def point(x,y):return (a[0]+co*x-si*y,a[1]+si*x+co*y)
        leader=(*point(1.7,0),h,0.,0.)
        with self.assertRaises(ValueError):reference_from_plan(data,[point(.5,.2),point(.6,.2)],Pose(*point(0,0),h),leader)
        with self.assertRaises(ValueError):reference_from_plan(data,[point(.5,0),point(.6,0)],Pose(*point(0,0),h+.5),leader)


class OutputTests(unittest.TestCase):
    def test_real_success_result_with_existing_flags_builds_and_saves(self):
        from check_temporary_detour import build_output
        from temporary_detour_candidate import Settings
        data,_=load_plan(PLAN)
        output=build_output(data,data['snapshot'],Settings(),None,'left',PLAN)
        self.assertFalse(output['evaluation_pass']);self.assertFalse(output['control_connected'])
        with tempfile.TemporaryDirectory() as tmp:
            target=Path(tmp)/'plan.json';save_current_plan(output,target)
            loaded,_=load_plan(target)
            self.assertEqual(loaded['reason'],'DETOUR_PLANNED')
            self.assertEqual(loaded['reference_plan'],str(PLAN))
    def test_failure_result_keeps_readonly_and_no_evaluation_pass(self):
        from check_temporary_detour import build_output
        from temporary_detour_candidate import Settings
        result=dict(reason='STOP_NO_COLLISION_CHECKED_DETOUR',path=[],evaluation_pass=True,control_connected=True)
        output=build_output(result,{},Settings(),None,'left',None)
        self.assertFalse(output['evaluation_pass']);self.assertFalse(output['control_connected'])
        self.assertEqual(output['reason'],result['reason'])
