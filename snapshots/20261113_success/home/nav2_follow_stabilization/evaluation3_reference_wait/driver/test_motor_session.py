import unittest
from unittest.mock import patch
from follower_motor_session import Session,MeasurementPort,main
from follower_lifted_pulse import PulsePort
from follower_motor_candidate import MotorCore
from follower_motor_mock import MemoryTransport
from test_motor_candidate import Clock


class Tests(unittest.TestCase):
    def measurement(self):
        clock=Clock();device=MemoryTransport();port=MeasurementPort(device)
        core=MotorCore(port,dry_run=False,allow_motion=True,clock=clock)
        return clock,device,core,Session(core,port)

    def test_measurement_five_second_window_once(self):
        c,d,core,s=self.measurement();self.advance(c,s,150);self.assertTrue(s.request_start())
        self.advance(c,s,115);begin=s.port.deadline-5.
        self.assertEqual(d.rpm,(-3,3));self.advance(c,s,350)
        self.assertEqual(s.phase,'PULSE');self.assertLess(c.now,begin+5.)
        self.advance(c,s,250);self.assertEqual(s.phase,'DONE');self.assertEqual(d.rpm,(0,0))
        count=core.valid_count;self.advance(c,s,150)
        self.assertGreater(core.valid_count,count);self.assertFalse(s.request_start())

    def test_manual_stop_during_measurement(self):
        c,d,core,s=self.measurement();self.advance(c,s,150);s.request_start()
        self.advance(c,s,115);self.assertEqual(d.rpm,(-3,3))
        s.stop();self.advance(c,s,100)
        self.assertEqual(s.phase,'ABORTED');self.assertEqual(d.rpm,(0,0))
        self.assertFalse(s.request_start());self.assertFalse(core.armed)

    def test_stop_while_waiting_prevents_later_start(self):
        c,d,core,s=self.measurement();self.advance(c,s,150);s.stop()
        self.advance(c,s,300);self.assertFalse(s.request_start());self.assertEqual(d.rpm,(0,0))

    def test_measurement_window_packet_expiry(self):
        from follower_motor_candidate import velocity_packet
        c,d,core,s=self.measurement();s.port.enable_once(c.now)
        with patch('follower_lifted_pulse.time.monotonic',return_value=c.now+5.):
            with self.assertRaises(ValueError):s.port.write(velocity_packet(-3,3))

    def test_measurement_default_opens_nothing(self):
        with patch('sys.argv',['measure']),patch.object(MeasurementPort,'open') as opened:
            self.assertEqual(main(measurement=True),0);opened.assert_not_called()

    def make(self):
        clock=Clock();device=MemoryTransport();port=PulsePort(device)
        core=MotorCore(port,dry_run=False,allow_motion=True,clock=clock)
        return clock,device,core,Session(core,port)

    def advance(self,clock,s,n):
        for _ in range(n):
            with patch('follower_lifted_pulse.time.monotonic',return_value=clock.now):s.step(clock.ros)
            clock.now+=.01

    def test_wait_never_auto_moves(self):
        c,d,core,s=self.make();self.assertFalse(s.request_start())
        self.advance(c,s,1000)
        self.assertTrue(s.ready());self.assertFalse(core.armed)
        self.assertEqual(d.rpm,(0,0));self.assertFalse(s.used)

    def test_once_then_continuous_zero_feedback(self):
        c,d,core,s=self.make();self.advance(c,s,150)
        self.assertTrue(s.request_start());self.assertFalse(s.request_start())
        self.advance(c,s,115);self.assertEqual(d.rpm,(-3,3))
        self.advance(c,s,200);self.assertEqual(s.phase,'DONE')
        count=core.valid_count;self.advance(c,s,500)
        self.assertGreater(core.valid_count,count+30)
        self.assertFalse(core.armed);self.assertEqual(d.rpm,(0,0))
        self.assertFalse(s.request_start())

    def test_feedback_loss_latches_no_restart(self):
        c,d,core,s=self.make();self.advance(c,s,150);self.assertTrue(s.request_start())
        self.advance(c,s,115);d.feedback_enabled=False;self.advance(c,s,100)
        self.assertEqual(s.phase,'FAULT');self.assertFalse(core.armed)
        d.feedback_enabled=True;self.advance(c,s,100)
        self.assertFalse(s.request_start());self.assertEqual(d.rpm,(0,0))

    def test_default_no_port(self):
        with patch('sys.argv',['session']),patch.object(PulsePort,'open') as opened:
            self.assertEqual(main(),0);opened.assert_not_called()

    def test_missing_ack_no_port(self):
        with patch('sys.argv',['session','--run-session']),patch.object(PulsePort,'open') as opened:
            with self.assertRaises(SystemExit):main()
            opened.assert_not_called()

    def test_pipe_refused(self):
        with patch('sys.argv',['session','--run-session','--ack-clear-1m-power-cut-ready']),patch('sys.stdin.isatty',return_value=False),patch.object(PulsePort,'open') as opened:
            with self.assertRaises(SystemExit):main()
            opened.assert_not_called()


if __name__=='__main__':unittest.main()
