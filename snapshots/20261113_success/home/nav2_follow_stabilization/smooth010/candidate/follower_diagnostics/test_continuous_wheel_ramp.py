import json,math,unittest
from test_continuous20 import CoreTests
from test_local_avoidance_drive import Tests as Setup

class Tests(unittest.TestCase):
    def test_equal_opposite_quantized_rpm_holds_zero_then_resumes_forward(self):
        c,ns=CoreTests().core();c.offer_scan(ns,1.,[(5.,5.)]);c.last_curve=(.99,.0025,.020)
        r=c.command(ns,1.,{},(.025,.15))
        self.assertEqual(r['reason'],'CONTINUOUS_RAMP_WARMUP')
        self.assertEqual(r['wheel_diagnostic']['rounded_rpm'],[-1,1]);self.assertEqual((r['v'],r['w'],r['motion']),(0.,0.,'STOP'))
        r=c.command(ns,1.02,{},(.025,.15))
        self.assertEqual(r['reason'],'CONTINUOUS_READY');self.assertGreater(r['wheel_diagnostic']['quantized_v'],0.)
        clock,t,port,contract=Setup().contract();contract.continuous20=True
        envelope=Setup().envelope(clock,contract,'DRIVE',r['v'],r['w'])
        self.assertTrue(contract.command(json.dumps(envelope),clock.ros))
    def test_slew_tail_is_scaled_with_both_wheels_before_envelope(self):
        c,ns=CoreTests().core();c.offer_scan(ns,1.,[(5.,5.)]);c.last_curve=(.99,.025,.2)
        r=c.command(ns,1.,{},(.025,0.))
        self.assertEqual(r['reason'],'CONTINUOUS_READY');self.assertLess(r['wheel_diagnostic']['final_scale'],1.)
        self.assertLessEqual(max(abs(x) for x in r['wheel_diagnostic']['rounded_rpm']),10)
        self.assertLessEqual(max(abs(r['v']-.28*r['w']),abs(r['v']+.28*r['w']))/.065*60/(2*math.pi),10.+1e-9)
    def test_near_obstacle_guard_is_not_bypassed_by_warmup(self):
        c,ns=CoreTests().core();c.offer_scan(ns,1.,[(.3,0.)]);c.last_curve=(.99,.0025,.020)
        self.assertEqual(c.command(ns,1.,{},(.025,.15))['reason'],'STOP_FOOTPRINT_CLEARANCE')

if __name__=='__main__':unittest.main()
