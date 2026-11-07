import json
import unittest
from types import SimpleNamespace as S
from follower_guard_gateway import Gateway,parse_guard_envelope
from follower_drive_candidate import DriveContract,BoundedDrivePort
from follower_motor_mock import MemoryTransport
from test_motor_candidate import Clock


def guard(stamp,v=.02):
    return S(header=S(frame_id='follower/base_link',stamp=S(sec=stamp//10**9,nanosec=stamp%10**9)),
             twist=S(linear=S(x=v,y=0.,z=0.),angular=S(x=0.,y=0.,z=0.)))


class Tests(unittest.TestCase):
    def state(self):
        return dict(mode='MEMORY_ONLY',input_kind='guard',armed=True,enabled=True,
                    feedback_fresh=True,fault=None,epoch='a'*16,arm_ros_ns=1_000_000_000)

    def make_gateway(self):
        g=Gateway();g.update_status(json.dumps(self.state()),1.)
        g.update_guard(guard(1_100_000_000),1.)
        return g

    def make_drive(self):
        c=Clock();d=MemoryTransport();p=BoundedDrivePort(d,c)
        k=DriveContract(p,enabled=True,clock=c)
        for _ in range(70):k.tick(c.ros);c.now+=.01
        self.assertIsNotNone(k.arm(c.ros));c.now+=.01
        return c,d,k

    def test_both_stamps_preserved(self):
        obj=self.make_gateway().envelope(1_200_000_000,1.1)
        self.assertEqual(obj['source_stamp_ns'],1_100_000_000)
        self.assertEqual(obj['stamp_ns'],1_200_000_000)
        self.assertEqual(parse_guard_envelope(json.dumps(obj)),obj)

    def test_unarmed_wrong_input_fault_no_command(self):
        for key,value in [('armed',False),('enabled',False),('input_kind','manual'),('fault','ERROR'),('feedback_fresh',False)]:
            g=self.make_gateway();s=self.state();s[key]=value;g.update_status(json.dumps(s),1.)
            self.assertIsNone(g.envelope(1_200_000_000,1.1))

    def test_stale_original_not_refreshed_by_receipt(self):
        g=self.make_gateway();g.update_guard(guard(1_100_000_000),1.4)
        g.update_status(json.dumps(self.state()),1.4)
        self.assertIsNone(g.envelope(1_500_000_000,1.4))

    def test_prearm_future_and_receive_timeout(self):
        for stamp in [900_000_000,1_300_000_000]:
            g=self.make_gateway();g.update_guard(guard(stamp),1.)
            self.assertIsNone(g.envelope(1_200_000_000,1.1))
        self.assertIsNone(self.make_gateway().envelope(1_200_000_000,1.4))

    def test_generation_stamp_does_not_repeat_or_go_back(self):
        g=self.make_gateway();self.assertIsNotNone(g.envelope(1_200_000_000,1.1))
        self.assertIsNone(g.envelope(1_200_000_000,1.1));self.assertIsNone(g.envelope(1_190_000_000,1.1))

    def test_invalid_frames_reverse_inplace_rejected(self):
        for m in [guard(1_100_000_000,-.01),guard(0),guard(1_100_000_000)]:
            g=self.make_gateway()
            if m.twist.linear.x>0:m.header.frame_id='map'
            g.update_guard(m,1.);self.assertIsNone(g.envelope(1_200_000_000,1.1))

    def test_wire_schema_strict(self):
        obj=self.make_gateway().envelope(1_200_000_000,1.1)
        for extra in [dict(source_stamp_ns=0),dict(version=1),dict(source_stamp_ns=True),dict(extra=1)]:
            bad=obj.copy();bad.update(extra)
            with self.assertRaises(ValueError):parse_guard_envelope(json.dumps(bad))

    def test_guard_envelope_drives_memory_with_original_stamp(self):
        c,d,k=self.make_drive();g=Gateway()
        for _ in range(30):
            s=k.status();s.update(mode='MEMORY_ONLY',input_kind='guard')
            g.update_status(json.dumps(s),c.now);g.update_guard(guard(c.ros),c.now)
            obj=g.envelope(c.ros,c.now)
            self.assertTrue(k.command(json.dumps(obj),c.ros,require_source=True))
            k.tick(c.ros);c.now+=.01
        self.assertEqual(d.rpm,(-3,3))

    def test_fresh_generation_cannot_hide_old_source_from_drive(self):
        c,d,k=self.make_drive()
        obj=dict(version=2,epoch=k.core.epoch,frame='follower/base_link',stamp_ns=c.ros,
                 source_stamp_ns=c.ros-1_000_000_000,v=.02,w=0.)
        self.assertFalse(k.command(json.dumps(obj),c.ros,require_source=True))
        self.assertFalse(k.core.armed);self.assertEqual(d.rpm,(0,0))


if __name__=='__main__':unittest.main()
