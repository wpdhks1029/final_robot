import math,unittest
from unittest.mock import patch
import numpy as np
import test_local_stop_turn_go as harness
from test_filtered_local_pose import FilteredMemory
from continuous_local_core import ContinuousCore,build_plan
from map_rejoin_guidance import guidance
from live_rejoin_verification import LiveRejoinVerification
from test_map_rejoin_guidance import inputs

class Core(ContinuousCore):
    def __init__(self):super().__init__(pose_source='filtered');self.verify=LiveRejoinVerification()
    def command(self,now,mono,motor,follow_command=None):
        sc=self.scene(now,mono);p=sc['pose'];nav=inputs(self.inputs['filtered'][0],mono,(p.x,p.y,p.yaw,0,0))
        if self.fsm.controller.stage>=1:self.fsm.controller.map_guidance=guidance(nav,self,now,mono)
        r=super().command(now,mono,motor,follow_command)
        rear=p.x-.27*abs(math.cos(p.yaw))-.3*abs(math.sin(p.yaw))
        eligible=self.fsm.controller.stage==3 and rear>self.plan['box_far']+.3
        v=self.verify.check(nav,now,mono,r['pose'],(0,0),0.,eligible,require_frozen_alignment=False)
        r['geometry_complete']=v['verified'];return r

class Tests(FilteredMemory):
    def assert_modes_for_test(self,modes):
        self.assertIn('DRIVE',modes);self.assertTrue(modes<= {'DRIVE','STOP'})
    def minimum_brakes_for_test(self):return 0
    def expected_modes_for_test(self):return {'DRIVE','STOP'}
    def core_for_test(self):return Core()
    def data_for_test(self):
        pts=[(-5.,-5.),(10.,5.)]
        for f in np.linspace(0,1,41):pts.extend([(1.,-.1+.2*f),(1.2,-.1+.2*f),(1.+.2*f,-.1),(1.+.2*f,.1)])
        return dict(points=pts,start=[0.,0.,0.])
    def plan_for_test(self,d):return build_plan(d['points'],d['start'])
    def test_full_measured_feedback_phase_contract_and_rejoin(self):
        original=harness.LocalAvoidanceContract
        def factory(*args,**kw):
            c=original(*args,**kw);c.continuous20=True;return c
        with patch.object(harness,'LocalAvoidanceContract',side_effect=factory):
            super().test_full_measured_feedback_phase_contract_and_rejoin()

if __name__=='__main__':unittest.main()
