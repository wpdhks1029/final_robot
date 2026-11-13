"""Quantized-wheel PP recovery, with delayed pose and a bounded actuator lag."""
import math,unittest
from tracked_pursuit_candidate import TrackedPreviewCore
from compare_avoidance_strategies import advance

def simulate(lookahead):
    n=TrackedPreviewCore(lookahead);n.speed_gate=lambda *args:(.025,0.,'FORWARD_ALLOWED')
    path=[(i*.1,0.) for i in range(101)]
    p=(1.,-.066,math.radians(-6.5));history=[p]*5;v_actual=w_actual=0.;rows=[]
    for k in range(1000):
        measured=history[-5]
        n.inputs={'path':(0,0,path),'follower':(0,0,(*measured,0.,0.)),'leader':(0,0,(9.,0.,0.,.04,0.))}
        v,w,reason=n.command(0,0)
        if reason!='TRACKED_PURSUIT':raise AssertionError(reason)
        w=max(-8*v,min(8*v,w))
        left=round((v-w*.28)/.065*60/(2*math.pi));right=round((v+w*.28)/.065*60/(2*math.pi))
        qv=(left+right)*.5*.065*2*math.pi/60;qw=(right-left)*.065*2*math.pi/60/.56
        v_actual+=(qv-v_actual)*.25;w_actual+=(qw-w_actual)*.25
        p=advance(p,v_actual,w_actual,.05);history.append(p);rows.append(p)
    return rows

class Tests(unittest.TestCase):
    def test_shorter_lookahead_limits_heading_excursion_with_quantized_wheels(self):
        old=simulate(.8);new=simulate(.35)
        # From the measured handoff offset, stay within the actual 10cm/10deg criteria.
        self.assertLess(max(abs(p[1]) for p in new),.1)
        self.assertLess(max(abs(p[2]) for p in new),math.radians(10))
        self.assertLess(abs(new[-1][1]),abs(old[-1][1]))
        self.assertLess(abs(new[-1][1]),.02)
    def test_invalid_lookahead_rejected(self):
        for value in (float('nan'),0.,2.):
            with self.assertRaises(ValueError):TrackedPreviewCore(value)

if __name__=='__main__':unittest.main()
