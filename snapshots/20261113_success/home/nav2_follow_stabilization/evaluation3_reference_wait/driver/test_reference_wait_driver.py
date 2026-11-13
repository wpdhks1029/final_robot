import json,unittest
from test_local_avoidance_drive import Tests as Setup

class WaitDriverTests(unittest.TestCase):
    def make(self):
        clock,device,port,c=Setup().contract(seconds=0);c.continuous20=True
        return clock,device,port,c
    def send(self,clock,c,motion='STOP',source=None,**extra):
        obj=Setup().envelope(clock,c,motion,.02 if motion=='DRIVE' else 0.,0.)
        obj.update(clearance_m=.1,swept_clearance_m=.08)
        if source is not None:obj['source_stamp_ns']=source
        obj.update(extra)
        return c.command(json.dumps(obj),clock.ros)
    def test_zero_wait_past_six_minutes_then_fresh_motion_and_disarm(self):
        clock,device,port,c=self.make()
        for _ in range(12200):
            self.assertTrue(self.send(clock,c));c.tick(clock.ros);clock.now+=.05
            self.assertIsNone(c.core.fault)
            self.assertEqual(device.rpm,(0,0))
        self.assertTrue(c.core.armed);self.assertIsNone(c.deadline)
        self.assertTrue(c.status()['session_unlimited']);self.assertTrue(c.status()['reference_wait_supported'])
        for _ in range(20):self.assertTrue(self.send(clock,c,'DRIVE'));c.tick(clock.ros);clock.now+=.02
        self.assertNotEqual(device.rpm,(0,0));c.disarm();c.tick(clock.ros);self.assertEqual(device.rpm,(0,0))
    def test_source_expired_during_transit_produces_only_zero_without_fault(self):
        clock,device,port,c=self.make();old=clock.ros
        for _ in range(50):self.send(clock,c,'DRIVE');c.tick(clock.ros);clock.now+=.02
        self.assertTrue(self.send(clock,c,'DRIVE',source=old));c.tick(clock.ros)
        self.assertIsNone(c.core.fault);self.assertTrue(c.core.armed)
        self.assertEqual(c.port.motion_kind,'STOP');self.assertEqual(c.core.command[:2],(0.,0.))
        self.assertTrue(c.status()['reference_wait_active']);self.assertEqual(c.reference_expired_holds,1)
        self.assertEqual(c.status()['last_reference_hold']['source_stamp_ns'],old)
        # A fresh DRIVE cannot silently clear the wait before the operator sees
        # it. A real fresh STOP acknowledges the hold before later resumption.
        clock.now+=.02
        self.assertTrue(self.send(clock,c,'DRIVE'))
        self.assertEqual(c.core.command[:2],(0.,0.));self.assertTrue(c.reference_wait_active)
        clock.now+=.02
        self.assertTrue(self.send(clock,c,'STOP'));self.assertFalse(c.reference_wait_active)
    def test_recorded_point800619845_source_race_holds_zero(self):
        clock,device,port,c=self.make()
        for _ in range(50):self.send(clock,c);c.tick(clock.ros);clock.now+=.02
        source=clock.ros-800619845
        self.assertTrue(self.send(clock,c,'DRIVE',source=source));c.tick(clock.ros)
        self.assertIsNone(c.core.fault)
        self.assertEqual(c.core.command[:2],(0.,0.));self.assertEqual(c.port.motion_kind,'STOP')
    def test_future_source_and_wrong_epoch_still_latch(self):
        for kind in ('future','epoch'):
            clock,device,port,c=self.make()
            kw={'source':clock.ros+1} if kind=='future' else {'epoch':'0000000000000000'}
            self.assertFalse(self.send(clock,c,'DRIVE',**kw));self.assertIsNotNone(c.core.fault)
    def test_command_and_feedback_watchdogs_still_stop(self):
        for failure in ('command','feedback'):
            clock,device,port,c=self.make()
            if failure=='feedback':device.feedback_enabled=False
            for _ in range(30):
                if failure=='feedback' and c.core.armed:self.send(clock,c)
                c.tick(clock.ros);clock.now+=.02
            self.assertIsNotNone(c.core.fault);self.assertFalse(c.core.armed);self.assertEqual(device.rpm,(0,0))
    def test_stale_source_does_not_mask_out_of_bounds_motion(self):
        clock,device,port,c=self.make();old=clock.ros
        for _ in range(50):self.send(clock,c);c.tick(clock.ros);clock.now+=.02
        self.assertFalse(self.send(clock,c,'DRIVE',source=old,v=.5))
        self.assertEqual(c.core.fault,'INVALID_LOCAL_AVOIDANCE_ENVELOPE')

if __name__=='__main__':unittest.main()
