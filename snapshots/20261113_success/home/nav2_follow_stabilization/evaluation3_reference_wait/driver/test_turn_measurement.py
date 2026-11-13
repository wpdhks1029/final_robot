import math
import unittest
from unittest.mock import patch

from follower_motor_session import Session,TurnMeasurementPort,main
from follower_lifted_turn import TurnTrial
from follower_motor_candidate import MotorCore,velocity_packet
from follower_motor_mock import MemoryTransport
from follower_comparison_observer import summary
from test_motor_candidate import Clock


class Tests(unittest.TestCase):
    def make(self):
        c=Clock();d=MemoryTransport();p=TurnMeasurementPort(d)
        core=MotorCore(p,dry_run=False,allow_motion=True,clock=c)
        return c,d,core,Session(core,p,TurnTrial)

    def advance(self,c,s,n):
        for _ in range(n):
            with patch('follower_lifted_turn.time.monotonic',return_value=c.now):s.step(c.ros)
            c.now+=.01

    def test_wait_turn_once_and_continue_stopped_feedback(self):
        c,d,core,s=self.make();self.advance(c,s,150);self.assertEqual(d.rpm,(0,0))
        self.assertTrue(s.request_start());self.advance(c,s,115)
        self.assertEqual(d.rpm,(3,3));self.assertGreater(core.odom_w,0)
        self.advance(c,s,350);self.assertEqual(s.phase,'PULSE')
        self.advance(c,s,250);self.assertEqual(s.phase,'DONE');self.assertEqual(d.rpm,(0,0))
        count=core.valid_count;self.advance(c,s,150);self.assertGreater(core.valid_count,count)
        self.assertFalse(s.request_start())

    def test_stop_latched(self):
        c,d,core,s=self.make();self.advance(c,s,150);s.request_start();self.advance(c,s,115)
        s.stop();self.advance(c,s,100)
        self.assertEqual(d.rpm,(0,0));self.assertFalse(s.request_start())

    def test_allowlist_and_five_second_boundary(self):
        c,d,core,s=self.make();s.port.enable_once(c.now)
        with patch('follower_lifted_turn.time.monotonic',return_value=c.now):
            with self.assertRaises(ValueError):s.port.write(velocity_packet(-3,3))
        with patch('follower_lifted_turn.time.monotonic',return_value=c.now+5.):
            with self.assertRaises(ValueError):s.port.write(velocity_packet(3,3))

    def test_default_no_open(self):
        with patch('sys.argv',['turn']),patch.object(TurnMeasurementPort,'open') as opened:
            self.assertEqual(main(measurement=True,turn=True),0);opened.assert_not_called()

    def test_requires_turn_specific_ack(self):
        with patch('sys.argv',['turn','--run-turn-measurement-session','--ack-clear-1m-power-cut-ready']),patch.object(TurnMeasurementPort,'open') as opened:
            with self.assertRaises(SystemExit):main(measurement=True,turn=True)
            opened.assert_not_called()

    def test_yaw_wrap_and_gyro_integration(self):
        rows=[dict(rx_s=0.,stamp_ns=1_000_000_000,x=0.,y=0.,yaw=math.radians(179),wz=.1),
              dict(rx_s=.1,stamp_ns=1_100_000_000,x=0.,y=0.,yaw=math.radians(-179),wz=.1)]
        result=summary(rows)
        self.assertIn('delta_yaw=2.000deg',result);self.assertIn('integrated_wz=0.573deg',result)

    def test_gyro_gaps_not_filled(self):
        rows=[dict(rx_s=0.,stamp_ns=1_000_000_000,x=None,wz=.1),
              dict(rx_s=1.,stamp_ns=2_000_000_000,x=None,wz=.1)]
        result=summary(rows)
        self.assertIn('integration_skips=1',result);self.assertIn('integrated_wz=0.000deg',result)


if __name__=='__main__':unittest.main()
