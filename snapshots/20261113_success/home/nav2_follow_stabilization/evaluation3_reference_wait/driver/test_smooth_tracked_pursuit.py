import math
import contextlib
import io
from types import SimpleNamespace as S
import unittest
from unittest.mock import patch
import run_local_avoidance_trial as runner

from smooth_tracked_pursuit import SmoothTrackedPreviewCore
from nav2_convoy_gate_core import Nav2ConvoyGateCore
from nav2_follow_safety import bounded_pursuit_result
import test_nav2_follow_safety as safety_tests


def set_inputs(core, t, *, gap=1.57, v=.04, w=0., follower_x=0.):
    ns=100_000_000_000+round(t*1e9)
    stamp=S(sec=ns//10**9,nanosec=ns%10**9)
    core.inputs={'leader':(stamp,t,(follower_x+gap,0.,0.,v,w)),
        'follower':(stamp,t,(follower_x,0.,0.,0.,0.)),
        'path':(stamp,t,((0.,0.),(5.,0.)))}
    core.leader_moving=v>.03
    return ns


class SmoothTests(unittest.TestCase):
    def test_reported_zero_speed_does_not_force_stop_when_gap_allows_catchup(self):
        c=SmoothTrackedPreviewCore()
        for i in range(50):
            ns=set_inputs(c,i*.02,v=.04 if i%2 else 0.)
            v,w,reason=c.command(ns,i*.02)
            if i>0:
                self.assertGreater(v,0.)
                self.assertEqual(reason,'TRACKED_PURSUIT')
        self.assertLessEqual(v,.1)

    def test_near_gap_reverse_rotation_and_stale_path_still_stop_immediately(self):
        for params,expected in [(dict(gap=.9),'STOP_NEAR_LEADER'),
                                (dict(v=-.02),'STOP_LEADER_REVERSE'),
                                (dict(v=0.,w=.1),'STOP_LEADER_ROTATING')]:
            c=SmoothTrackedPreviewCore();ns=set_inputs(c,1.)
            c.feedback_bounded(.1,.01,1.)
            ns=set_inputs(c,1.02,**params)
            v,w,reason=c.command(ns,1.02)
            self.assertEqual((v,w),(0.,0.));self.assertEqual(reason,expected)
        c=SmoothTrackedPreviewCore();ns=set_inputs(c,1.)
        self.assertEqual(c.command(ns+1_000_000_000,2.)[2],'STOP_STALE_PATH')

    def test_acceleration_curvature_and_final_wheel_bound_feedback(self):
        c=SmoothTrackedPreviewCore();previous=0.
        scene=safety_tests.FollowSafetyTests().scene()
        for i in range(80):
            t=i*.02;v,w,reason=c.limit_drive(.1,.2,'TRACKED_PURSUIT',t)
            self.assertLessEqual(v-previous,.12*.02+1e-9)
            if v>0:self.assertAlmostEqual(w/v,2.)
            r=bounded_pursuit_result(scene,v,w,reason,max_linear=.1,max_rpm=15.)
            c.feedback_bounded(r['v'],r['w'],t);previous=r['v']
            wheel=max(abs(r['v']-.28*r['w']),abs(r['v']+.28*r['w']))/.065*60/(2*math.pi)
            self.assertLessEqual(wheel,15.+1e-9)

    def test_gap_hold_ramps_down_but_emergency_resets_immediately(self):
        c=SmoothTrackedPreviewCore();c.feedback_bounded(.1,.02,1.)
        v,w,_=c.limit_drive(0.,0.,'SMOOTH_GAP_HOLD',1.02)
        self.assertAlmostEqual(v,.096);self.assertAlmostEqual(w/v,.2)
        self.assertEqual(c.limit_drive(.1,0.,'STOP_NEAR_LEADER',1.04)[:2],(0.,0.))

    def test_repeated_stamp_does_not_advance_velocity_filter(self):
        c=SmoothTrackedPreviewCore();ns=set_inputs(c,1.,v=.1)
        c.speed_gate(ns,1.)
        stamp,rx,pose=c.inputs['leader']
        c.inputs['leader']=(stamp,rx,(*pose[:3],0.,0.))
        for _ in range(10):c.speed_gate(ns,1.)
        self.assertEqual(c.filtered_leader_v,.1)
        ns=set_inputs(c,1.05,v=0.)
        c.speed_gate(ns,1.05)
        self.assertLess(c.filtered_leader_v,.1)

    def test_gap_below_target_and_stationary_leader_eventually_holds_zero(self):
        c=SmoothTrackedPreviewCore()
        for i in range(50):
            ns=set_inputs(c,i*.02,gap=1.45,v=0.)
            v,w,reason=c.command(ns,i*.02)
            self.assertEqual((v,w),(0.,0.));self.assertEqual(reason,'SMOOTH_GAP_HOLD')

    def test_gate_keeps_bounded_startup_during_small_ramp(self):
        c=Nav2ConvoyGateCore(max_linear=.1);c.graph_ok=True
        c.update_nav(.15,0.,1.)
        c.update_motor(dict(armed=True,fault=None,io_failed=False,feedback_fresh=True),1.)
        c.update_leader_pose(1.57,0.,1.);c.update_follower_pose(0.,0.,1.)
        c.update_follower(dict(trial_phase='FOLLOW',smooth_follow=True,reason='TRACKED_PURSUIT',
                               follower_requested_v=.002,follower_follow_reason='TRACKED_PURSUIT'),1.)
        self.assertAlmostEqual(c.evaluate(1.).linear_x,.04)
        c.update_follower(dict(trial_phase='FOLLOW',smooth_follow=True,reason='TRACKED_PURSUIT',
                               follower_requested_v=.08,follower_follow_reason='TRACKED_PURSUIT'),1.)
        self.assertAlmostEqual(c.evaluate(1.).linear_x,.08)
        c.update_follower(dict(trial_phase='FOLLOW',smooth_follow=True,
                               reason='STOP_FOLLOW_REFERENCE_WAIT'),1.)
        self.assertEqual(c.evaluate(1.).linear_x,0.)

    def test_loop_gap_or_recovery_reset_cannot_jump_back_to_previous_speed(self):
        c=SmoothTrackedPreviewCore();c.feedback_bounded(.1,.01,1.)
        self.assertEqual(c.limit_drive(.1,.02,'TRACKED_PURSUIT',2.)[:2],(0.,0.))
        c.reset_motion()
        self.assertEqual(c.limit_drive(.1,.02,'TRACKED_PURSUIT',3.)[:2],(0.,0.))

    def test_speed_floor_cannot_override_smooth_ramp(self):
        args=['follow','--smooth-follow','--follow-only','--nav2-leader',
              '--auto-start-on-goal','--ack-extended-follow-speed',
              '--follow-max-linear','0.1','--follow-min-linear','0.04']
        with patch('sys.argv',args),contextlib.redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):
            runner.main()

    def test_centimetre_gap_changes_do_not_cancel_low_leader_speed(self):
        c=SmoothTrackedPreviewCore()
        for i,gap in enumerate((1.47,1.49,1.51,1.53,1.48)):
            ns=set_inputs(c,1.+i*.02,gap=gap,v=.006)
            v,w,reason=c.speed_gate(ns,1.+i*.02)
            self.assertAlmostEqual(v,.006)
            self.assertEqual(c.speed_diagnostic['gap_correction_mps'],0.)
            self.assertEqual(reason,'SMOOTH_FORWARD_ALLOWED')

    def test_deadband_has_no_speed_jump_at_its_boundaries(self):
        for boundary in (1.45,1.55):
            c=SmoothTrackedPreviewCore();values=[]
            for i,gap in enumerate((boundary-1e-6,boundary,boundary+1e-6)):
                ns=set_inputs(c,1.+i*.02,gap=gap,v=.04)
                values.append(c.speed_gate(ns,1.+i*.02)[0])
            self.assertLessEqual(max(values)-min(values),1.1e-6)

    def test_real_stationary_leader_does_not_force_creep_inside_band(self):
        c=SmoothTrackedPreviewCore();ns=set_inputs(c,1.,gap=1.48,v=0.)
        self.assertEqual(c.speed_gate(ns,1.),(0.,0.,'SMOOTH_GAP_HOLD'))

    def test_gap_correction_outside_band_and_near_stop_remain(self):
        c=SmoothTrackedPreviewCore();ns=set_inputs(c,1.,gap=1.60,v=.04)
        self.assertAlmostEqual(c.speed_gate(ns,1.)[0],.065)
        c=SmoothTrackedPreviewCore()
        ns=set_inputs(c,1.02,gap=1.40,v=.01)
        self.assertEqual(c.speed_gate(ns,1.02)[0],0.)
        ns=set_inputs(c,1.04,gap=.95,v=.1)
        self.assertEqual(c.speed_gate(ns,1.04)[2],'STOP_NEAR_LEADER')


if __name__=='__main__':unittest.main()
