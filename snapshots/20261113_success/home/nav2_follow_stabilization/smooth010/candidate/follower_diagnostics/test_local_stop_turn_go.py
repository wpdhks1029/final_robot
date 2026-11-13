import json,math,unittest
from pathlib import Path
import numpy as np
from local_stop_turn_go import LocalCore,pose_at,build_local_plan
from temporary_detour_candidate import Pose
from compare_avoidance_strategies import body_points,advance
from local_avoidance_drive import LocalAvoidanceContract,PivotDrivePort,PROFILE
from follower_motor_mock import MemoryTransport
from test_motor_candidate import Clock

DATA=Path(__file__).parent/'logs/avoidance_candidate_verification_wider.json'

class Tests(unittest.TestCase):
    def test_interpolation_and_limited_hold_preserve_old_source(self):
        rows=[(100_000_000_000,1.,(0.,0.,0.,0.,0.)),(100_100_000_000,1.1,(.006,0.,.02,0.,0.))]
        p,source,error=pose_at(rows,100_050_000_000)
        self.assertAlmostEqual(p.x,.003);self.assertEqual(source,100_000_000_000);self.assertEqual(error,0.)
        p,source,error=pose_at(rows,100_130_000_000)
        self.assertEqual(source,100_100_000_000);self.assertGreater(error,0.)
        with self.assertRaises(ValueError):pose_at(rows,100_160_000_000)
    def test_frozen_local_reference_maps_relative_motion(self):
        c=LocalCore();c.freeze((1.,2.,.2),(0.,.04,-.02))
        p=c.reference_pose(Pose(1.,2.,.2));self.assertAlmostEqual(p.y,.04);self.assertAlmostEqual(p.yaw,-.02)
        p=c.reference_pose(Pose(1.+math.cos(.2),2.+math.sin(.2),.2))
        self.assertAlmostEqual(p.x,math.cos(.02));self.assertAlmostEqual(p.y,.04-math.sin(.02))
    def test_current_recorded_cloud_has_checked_stop_turn_plan(self):
        d=json.loads(DATA.read_text());result=build_local_plan(d['points'],d['start'])
        self.assertEqual(result['geometry']['reason'],'LIVE_PATH_GEOMETRY_CLEAR')
        self.assertGreater(result['return_x'],result['box_far']+.8)
    def test_stale_inputs_and_missing_pose_bracket_stop(self):
        c=LocalCore();c.freeze((0,0,0),(0,0,0))
        c.raw(100_000_000_000,1.,(0.,0.,0.,0.,0.));c.inputs['leader']=(100_000_000_000,1.,(2.,0.,0.,.04,0.))
        c.offer_scan(100_000_000_000,1.,[(2.,0.)])
        with self.assertRaises(ValueError):c.scene(100_900_000_000,1.9)
    def test_hard_clearance_is_not_removed_for_pivot(self):
        c=LocalCore();c.freeze((0,0,0),(0,0,0))
        c.raw(100_000_000_000,1.,(0.,0.,0.,0.,0.));c.inputs['leader']=(100_000_000_000,1.,(2.,0.,0.,.04,0.))
        c.offer_scan(100_000_000_000,1.,[(.5,0.)])
        r=c.command(100_100_000_000,1.1,{})
        self.assertEqual(r['reason'],'STOP_FOOTPRINT_CLEARANCE');self.assertEqual(r['v'],0.)

class MemoryClosedLoop(unittest.TestCase):
    def assert_modes_for_test(self,modes):self.assertEqual(modes,self.expected_modes_for_test())
    def minimum_brakes_for_test(self):return 4
    def expected_modes_for_test(self):return {'DRIVE','TURN','STOP'}
    def core_for_test(self):return LocalCore()
    def offer_pose_for_test(self,core,stamp,mono,data):core.raw(stamp,mono,data)
    def scan_points_for_test(self,points,elapsed):return points
    def control_for_test(self,core,result,mono,motor):return result
    def data_for_test(self):
        return json.loads(DATA.read_text())
    def plan_for_test(self,d):
        return dict(path=d['route'],goals=d['goals'],box_far=d['box_bounds']['far'],return_x=d['config']['return_x'])
    def test_full_measured_feedback_phase_contract_and_rejoin(self):
        d=self.data_for_test();points=np.asarray(d['points']);start=d['start'];c=Clock()
        transport=MemoryTransport();port=PivotDrivePort(transport,c,max_rpm=10)
        contract=LocalAvoidanceContract(port,enabled=True,clock=c,max_rpm=10,trial_seconds=360)
        for _ in range(80):contract.tick(c.ros);c.now+=.01
        core=self.core_for_test();core.freeze((0.,0.,0.),start)
        core.install(self.plan_for_test(d))
        pose=tuple(start);pose_history=[];vel=np.zeros(2);modes=set();brakes=0
        def raw(p):
            dx,dy=p[0]-start[0],p[1]-start[1];co,si=math.cos(start[2]),math.sin(start[2])
            return (co*dx+si*dy,-si*dx+co*dy,p[2]-start[2],float(vel[0]),float(vel[1]))
        # Populate a stationary measured history before the explicit arm.
        for _ in range(12):
            self.offer_pose_for_test(core,c.ros,c.now,raw(pose));pose_history.append((c.ros,pose));contract.tick(c.ros);c.now+=.05
        epoch=contract.arm(c.ros);self.assertIsNotNone(epoch);arm_ns=contract.core.arm_ros_ns;began=c.now;min_guarded=999.
        complete=False;last_feedback_count=None
        for i in range(7000):
            contract.tick(c.ros)
            self.assertIsNone(contract.core.fault,contract.status())
            if contract.core.valid_count!=last_feedback_count:
                last_feedback_count=contract.core.valid_count
                self.offer_pose_for_test(core,c.ros,c.now,raw(pose))
            core.inputs['leader']=(c.ros,c.now,(5.,0.,0.,.04,0.))
            pose_history.append((c.ros,pose))
            if i%2==0:
                old=pose_history[max(0,len(pose_history)-3)]
                core.offer_scan(old[0],c.now,self.scan_points_for_test(body_points(points,old[1]).tolist(),c.now-began))
            result=core.command(c.ros,c.now,contract.status())
            result=self.control_for_test(core,result,c.now,contract.status())
            min_guarded=min(min_guarded,result['clearance_m'])
            self.assertFalse(result['reason'].startswith('STOP_'),result)
            if result['geometry_complete']:complete=True;break
            if result['reason']=='BRAKE_BEFORE_PHASE_CHANGE':brakes+=1
            if result['source_stamp_ns']>=arm_ns and result['scan_stamp_ns']>=arm_ns:
                obj=dict(version=3,controller=PROFILE,epoch=epoch,frame='follower/base_link',stamp_ns=c.ros,
                    source_stamp_ns=result['source_stamp_ns'],scan_stamp_ns=result['scan_stamp_ns'],motion=result['motion'],
                    v=result['v'],w=result['w'],clearance_m=result['clearance_m'],swept_clearance_m=result['swept_clearance_m'])
                self.assertTrue(contract.command(json.dumps(obj),c.ros),contract.last_command_rejection)
                modes.add(result['motion'])
            left=-transport.rpm[0]*.065*2*math.pi/60;right=transport.rpm[1]*.065*2*math.pi/60
            desired=np.array([(left+right)/2,(right-left)/.560]);vel+=(desired-vel)*(.05/.12)
            pose=advance(pose,float(vel[0]),float(vel[1]),.05);c.now+=.05
        self.assertTrue(complete,(i,pose,result));self.assert_modes_for_test(modes);self.assertGreaterEqual(brakes,self.minimum_brakes_for_test())
        duration=c.now-began
        contract.disarm()
        for _ in range(70):contract.tick(c.ros);c.now+=.01
        self.assertEqual(transport.rpm,(0,0));self.assertFalse(contract.core.armed)
        self.summary=dict(memory_only=True,control_connected=False,geometry_complete=complete,
                          pose_source=core.pose_source,swept_prediction_scale=result.get('swept_prediction_scale',1.),
                          virtual_duration_s=duration,min_guarded_clearance_m=min_guarded,raw_updates_from_actual_feedback_count=True,
                          accepted_commands=contract.accepted,phase_brake_ticks=brakes,
                          final_measured_rpm=list(transport.rpm),evaluation_pass=False)

if __name__=='__main__':unittest.main()


class InputLifetimeTests(unittest.TestCase):
    def test_half_second_measurements_are_valid_but_nine_tenths_are_not(self):
        c=LocalCore();c.freeze((0,0,0),(0,0,0))
        c.raw(100_000_000_000,1.,(0.,0.,0.,0.,0.));c.inputs['leader']=(100_000_000_000,1.,(2.,0.,0.,.04,0.))
        c.offer_scan(100_000_000_000,1.,[(2.,0.)])
        self.assertEqual(c.scene(100_500_000_000,1.5)['source'],99_950_000_000)
        with self.assertRaises(ValueError):c.scene(100_900_000_000,1.9)
    def test_vectorized_sweep_matches_all_scalar_footprint_checks(self):
        from local_stop_turn_go import minimum_clearance
        d=json.loads(DATA.read_text());world=np.asarray(d['points'])
        for pose,index in ((tuple(d['start']),0),((.16,d['start'][1],.1),2)):
            c=LocalCore();c.freeze((0,0,0),pose)
            c.raw(100_000_000_000,1.,(0.,0.,0.,0.,0.));c.inputs['leader']=(100_000_000_000,1.,(5.,0.,0.,.04,0.))
            c.offer_scan(100_000_000_000,1.,body_points(world,pose).tolist())
            c.install(dict(path=d['route'],goals=d['goals'],box_far=d['box_bounds']['far'],return_x=d['config']['return_x']));c.fsm.i=index
            r=c.command(100_100_000_000,1.1,dict(measured_rpm=[0,0],zero_feedback_samples=5))
            self.assertEqual(r['reason'],'LOCAL_AVOIDANCE_READY',r)
            wheels=np.round(np.array([r['v']-r['w']*.28,r['v']+r['w']*.28])/.065*60/(2*math.pi))
            v=float(wheels.mean()*.065*2*math.pi/60);w=float((wheels[1]-wheels[0])*.065*2*math.pi/60/.56)
            expected=r['clearance_m']
            for sv,sw in ((v,w),(0.,0.)):
                margin=r['uncertainty_m']+(abs(sv)+math.hypot(.27,.3)*abs(sw))*.1
                for k in range(1,11):expected=min(expected,minimum_clearance(world,advance(pose,sv,sw,k*.1))-margin)
            self.assertAlmostEqual(r['swept_clearance_m'],expected,places=10)


class DelayedPoseMarginTests(unittest.TestCase):
    def test_delayed_pose_uses_spatial_margin_before_any_motion(self):
        c=LocalCore();c.freeze((0,0,0),(0,0,0))
        c.raw(100_000_000_000,1.,(0.,0.,0.,0.,0.));c.inputs['leader']=(100_000_000_000,1.,(2.,0.,0.,.04,0.))
        c.offer_scan(100_000_000_000,1.,[(.6,0.)])
        fresh=c.scene(100_100_000_000,1.1);delayed=c.scene(100_500_000_000,1.5)
        self.assertGreater(delayed['uncertainty'],fresh['uncertainty'])
        r=c.command(100_500_000_000,1.5,{})
        self.assertEqual(r['reason'],'STOP_FOOTPRINT_CLEARANCE');self.assertEqual(r['v'],0.)

class AdaptivePlanTests(unittest.TestCase):
    def cloud(self):
        out=[(-5.,-5.),(5.,5.)]
        for q in np.linspace(0,1,31):out.extend([(.95,-.1+.2*q),(1.15,-.1+.2*q),(.95+.2*q,-.1),(.95+.2*q,.1)])
        return out
    def test_safe_offset_outside_original_six_candidates_is_found(self):
        points=self.cloud()+[(1.5,1.39)]
        result=build_local_plan(points,(0.,0.,0.))
        self.assertGreater(len(result['attempts']),6)
        self.assertTrue(all(x['reason']=='STOP_LIVE_PATH_BLOCKED' for x in result['attempts'][:6]))
        self.assertLess(result['config']['offset'],.78)
        self.assertEqual(result['geometry']['reason'],'LIVE_PATH_GEOMETRY_CLEAR')
        self.assertIn((1.5,1.39),result['points'])
    def test_failure_retains_measured_blocker_and_attempt_diagnostics(self):
        from local_stop_turn_go import LocalPlanFailure
        with self.assertRaises(LocalPlanFailure) as caught:build_local_plan(self.cloud()+[(0.,0.)],(0.,0.,0.),search_seconds=.1)
        d=caught.exception.diagnostic
        self.assertTrue(d['attempts']);self.assertIsNotNone(d['attempts'][0]['blocking'])
        self.assertEqual(d['attempts'][0]['blocking']['point'],[0.,0.])

class QuantizedPivotTests(unittest.TestCase):
    def pivot(self,point):
        c=LocalCore();c.freeze((0,0,0),(0,0,0))
        c.raw(100_000_000_000,1.,(0.,0.,0.,0.,0.))
        c.inputs['leader']=(100_000_000_000,1.,(2.,0.,0.,.04,0.))
        c.offer_scan(100_000_000_000,1.,[point])
        c.install(dict(goals=[('TURN',(0.,0.,1.57))],box_far=1.,return_x=2.))
        return c.command(100_100_000_000,1.1,dict(measured_rpm=[0,0],zero_feedback_samples=5))
    def test_safe_sub_half_rpm_turn_becomes_executable_one_rpm(self):
        r=self.pivot((.595,0.))
        self.assertEqual(r['reason'],'LOCAL_AVOIDANCE_READY',r)
        self.assertTrue(r['minimum_pivot_rpm_applied'])
        self.assertEqual(r['candidate_wheel_rpm'],[-1,1])
        self.assertGreater(r['swept_clearance_m'],.3)
    def test_minimum_pivot_cannot_bypass_full_body_sweep(self):
        r=self.pivot((.38,.592))
        self.assertGreater(r['clearance_m'],.3)
        self.assertEqual(r['reason'],'STOP_SWEPT_CLEARANCE',r)
        self.assertEqual((r['v'],r['w']),(0.,0.))
