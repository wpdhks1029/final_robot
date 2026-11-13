import unittest
from unittest.mock import patch
from follower_lifted_turn import TurnPort,TurnTrial,main
from follower_motor_candidate import MotorCore,velocity_packet
from follower_motor_mock import MemoryTransport
from test_motor_candidate import Clock


class Tests(unittest.TestCase):
    def make(self):
        c=Clock();d=MemoryTransport();p=TurnPort(d)
        core=MotorCore(p,dry_run=False,allow_motion=True,clock=c)
        return c,d,p,core,TurnTrial(core,p)

    def advance(self,c,t,n):
        for _ in range(n):
            with patch('follower_lifted_turn.time.monotonic',return_value=c.now):t.step(c.ros)
            c.now+=.01
            if t.phase in ('DONE','FAILED'):break

    def test_raw_signs_then_measured_zero(self):
        c,d,p,core,t=self.make();self.advance(c,t,115)
        self.assertEqual(d.rpm,(3,3));self.assertLess(core.odom_v,1e-9)
        self.assertGreater(core.odom_w,0)
        self.advance(c,t,150);self.assertEqual(t.phase,'DONE')
        self.assertEqual(d.rpm,(0,0));self.assertFalse(core.armed)

    def test_no_forward_packet_and_expiry(self):
        c,d,p,core,t=self.make();p.enable_once(c.now)
        with patch('follower_lifted_turn.time.monotonic',return_value=c.now):
            with self.assertRaises(ValueError):p.write(velocity_packet(-3,3))
        with patch('follower_lifted_turn.time.monotonic',return_value=c.now+.5):
            with self.assertRaises(ValueError):p.write(velocity_packet(3,3))
        with self.assertRaises(RuntimeError):p.enable_once(c.now)

    def test_loss_stops(self):
        c,d,p,core,t=self.make();self.advance(c,t,115);d.feedback_enabled=False
        self.advance(c,t,150);self.assertEqual(t.phase,'FAILED');self.assertFalse(core.armed)
        self.assertEqual(d.rpm,(0,0))

    def test_default_no_open(self):
        with patch('sys.argv',['turn']),patch.object(TurnPort,'open') as opened:
            self.assertEqual(main(),0);opened.assert_not_called()

    def test_ack_required(self):
        with patch('sys.argv',['turn','--run-lifted-turn']),patch.object(TurnPort,'open') as opened:
            with self.assertRaises(SystemExit):main()
            opened.assert_not_called()


if __name__=='__main__':unittest.main()
