import math,unittest
from types import SimpleNamespace as S
from local_stop_turn_go import LocalCore
from map_rejoin_guidance import guidance
from live_rejoin_verification import LiveRejoinVerification
from test_filtered_local_pose import FilteredMemory
from gentle_pivot_candidate import build_gentle_plan


def inputs(stamp,mono,pose):
    t=S(sec=stamp//10**9,nanosec=stamp%10**9)
    return {'follower':(t,mono,pose),'path':(t,mono,[(0.,0.),(10.,0.)])}

class Tests(unittest.TestCase):
    def test_future_map_timestamp_recovers_only_with_new_measured_pose(self):
        from map_rejoin_guidance import MapPoseBracketMissing
        c=LocalCore(pose_source='filtered');c.freeze((0,0,0),(0,0,0))
        c.filtered(100_000_000_000,1.,(0,0,0,0,0))
        nav=inputs(100_080_000_000,1.08,(0,0,0))
        with self.assertRaises(MapPoseBracketMissing) as caught:guidance(nav,c,100_100_000_000,1.1)
        self.assertTrue(caught.exception.diagnostic['waiting_for_newer_pose'])
        c.filtered(100_100_000_000,1.1,(0,0,0,0,0))
        g=guidance(nav,c,100_100_000_000,1.1)
        self.assertEqual(g['map_pose_support_ns'],100_000_000_000)
    def test_recorded_202ms_gap_rejected_then_new_supported_map_recovers(self):
        from map_rejoin_guidance import MapPoseBracketMissing
        c=LocalCore(pose_source='filtered');c.freeze((0,0,0),(0,0,0));ns=100_000_000_000
        c.filtered(ns,1.,(0,0,0,0,0));c.filtered(ns+202_764_981,1.203,(0,0,0,0,0))
        nav=inputs(ns+100_790_336,1.203,(0,0,0))
        with self.assertRaises(MapPoseBracketMissing) as caught:guidance(nav,c,ns+210_000_000,1.21)
        self.assertFalse(caught.exception.diagnostic['waiting_for_newer_pose'])
        c.filtered(ns+252_764_981,1.253,(0,0,0,0,0))
        nav=inputs(ns+220_000_000,1.253,(0,0,0))
        g=guidance(nav,c,ns+260_000_000,1.26)
        self.assertEqual(g['map_pose_support_ns'],ns+202_764_981)

    def test_map_jump_is_rejected_before_target_use(self):
        from map_rejoin_guidance import MapPoseContinuity
        c=MapPoseContinuity();c.check(inputs(100_000_000_000,1.,(0.,0.,0.))['follower'])
        c.check(inputs(100_100_000_000,1.1,(.005,0.,0.))['follower'])
        with self.assertRaisesRegex(ValueError,'LOCALIZATION_JUMP'):
            c.check(inputs(100_200_000_000,1.2,(.55,0.,0.))['follower'])
    def test_reported_half_metre_offset_and_yaw_error_target_actual_map_path(self):
        c=LocalCore(pose_source='filtered');c.freeze((0,0,0),(0,0,0))
        ns=100_000_000_000;c.filtered(ns,1.,(3.,0.,0.,0.,0.))
        g=guidance(inputs(ns,1.,(3.,.555,math.radians(11.1))),c,ns,1.)
        dx,dy=g['target_local'][0]-3.,g['target_local'][1]
        co,si=math.cos(math.radians(11.1)),math.sin(math.radians(11.1))
        self.assertAlmostEqual(3.+co*dx-si*dy,g['target_map'][0])
        self.assertAlmostEqual(.555+si*dx+co*dy,0.)
        self.assertLess(dy,-.5)
    def test_missing_stale_or_short_path_cannot_generate_target(self):
        c=LocalCore();c.freeze((0,0,0),(0,0,0));ns=100_000_000_000;c.raw(ns,1.,(0,0,0,0,0))
        with self.assertRaisesRegex(ValueError,'MISSING'):guidance({},c,ns,1.)
        with self.assertRaisesRegex(ValueError,'STALE'):guidance(inputs(ns,1.,(0,0,0)),c,ns+900_000_000,1.9)
    def test_map_odom_timestamp_must_have_measured_support(self):
        c=LocalCore();c.freeze((0,0,0),(0,0,0));c.raw(100_000_000_000,1.,(0,0,0,0,0))
        with self.assertRaises(ValueError):guidance(inputs(100_200_000_000,1.2,(0,0,0)),c,100_200_000_000,1.2)

class MapCore(LocalCore):
    def __init__(self):super().__init__(pose_source='filtered');self.verify=LiveRejoinVerification()
    def command(self,now,mono,motor,follow_command=None):
        sc=self.scene(now,mono);p=sc['pose'];stamp=self.inputs['filtered'][0]
        # Map localization differs from the frozen odom projection by55cm.
        nav=inputs(stamp,mono,(p.x,p.y+.55,p.yaw,0.,0.))
        if self.fsm.controller.stage>=1:
            self.fsm.controller.map_guidance=guidance(nav,self,now,mono)
        r=super().command(now,mono,motor,follow_command)
        rear=p.x-.27*abs(math.cos(p.yaw))-.3*abs(math.sin(p.yaw))
        eligible=self.fsm.controller.stage==3 and rear>self.plan['box_far']+.3
        v=self.verify.check(nav,now,mono,r['pose'],(0,0),0.,eligible,require_frozen_alignment=False)
        r['geometry_complete']=v['verified'];self.last_map_check=v
        return r

class MapMemory(FilteredMemory):
    def core_for_test(self):return MapCore()
    def data_for_test(self):
        import numpy as np
        pts=[(-5.,-5.),(10.,5.)]
        for f in np.linspace(0,1,41):pts.extend([(1.,-.1+.2*f),(1.2,-.1+.2*f),(1.+.2*f,-.1),(1.+.2*f,.1)])
        return dict(points=pts,start=[0.,0.,0.])
    def plan_for_test(self,d):return dict(build_gentle_plan(d['points'],d['start']),live_map_rejoin=True)

if __name__=='__main__':unittest.main()
