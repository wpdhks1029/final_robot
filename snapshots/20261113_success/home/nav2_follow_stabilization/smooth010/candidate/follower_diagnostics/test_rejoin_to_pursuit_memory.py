"""Whole measured-feedback contract: bypass -> map rejoin -> brake -> PP."""
import math
from types import SimpleNamespace as S
from test_map_rejoin_guidance import MapMemory,inputs
from local_stop_turn_go import LocalCore
from map_rejoin_guidance import guidance
from tracked_pursuit_candidate import TrackedPreviewCore
from live_rejoin_verification import LiveRejoinVerification

class HandoffCore(LocalCore):
    def __init__(self):
        super().__init__(pose_source='filtered');self.nav=TrackedPreviewCore();self.verify=LiveRejoinVerification()
        self.phase='AVOID';self.follow_start=None;self.follow_progress=None;self.handed=False
    def command(self,now,mono,motor,follow_command=None):
        scene=self.scene(now,mono);p=scene['pose'];stamp=self.inputs['filtered'][0]
        nav=inputs(stamp,mono,(p.x,p.y+.55,p.yaw,0.,0.))
        # A straight leader continues ahead at the nominal safe gap.
        nav['leader']=(nav['follower'][0],mono,(p.x+1.8,0.,0.,.04,0.))
        self.nav.inputs=nav;self.nav.timeout=.8;self.nav.leader_moving=True
        if self.phase=='BRAKE':
            if motor.get('measured_rpm')==[0,0] and motor.get('zero_feedback_samples',0)>=3:
                self.handoff_to_follow(motor);self.phase='FOLLOW';self.follow_start=mono
                self.follow_progress=self.nav.path_tracker.progress;self.handed=True
            else:
                # Keep the existing planner until actual wheel-zero is confirmed.
                r=super().command(now,mono,motor)
                r.update(v=0.,w=0.,motion='STOP',reason='BRAKE_HANDOFF',geometry_complete=False)
                return r
        if self.phase=='FOLLOW':
            v,w,reason=self.nav.command(now,mono)
            if reason.startswith('STOP_') and reason not in ('STOP_FOLLOW_DISTANCE','STOP_LEADER_STOPPED'):raise AssertionError(reason)
            r=super().command(now,mono,motor,(v,w))
            r['geometry_complete']=mono-self.follow_start>=10 and self.nav.path_tracker.progress-self.follow_progress>=.1
            return r
        g=None
        if self.fsm.controller.stage>=1:
            g=guidance(nav,self,now,mono,tracker=self.nav.path_tracker);self.fsm.controller.map_guidance=g
        r=super().command(now,mono,motor)
        rear=p.x-.27*abs(math.cos(p.yaw))-.3*abs(math.sin(p.yaw))
        eligible=self.fsm.controller.stage==3 and rear>self.plan['box_far']+.3
        checked=self.verify.check(nav,now,mono,r['pose'],(0,0),0.,eligible,require_frozen_alignment=False,path_selection=g['path_selection'] if g else None)
        r['geometry_complete']=False
        if checked['verified']:
            self.phase='BRAKE';r.update(v=0.,w=0.,motion='STOP',reason='BRAKE_HANDOFF')
        return r

class Tests(MapMemory):
    def core_for_test(self):
        self.handoff_core=HandoffCore();return self.handoff_core
    def test_full_measured_feedback_phase_contract_and_rejoin(self):
        super().test_full_measured_feedback_phase_contract_and_rejoin()
        self.assertTrue(self.handoff_core.handed)
        self.assertEqual(self.handoff_core.phase,'FOLLOW')
