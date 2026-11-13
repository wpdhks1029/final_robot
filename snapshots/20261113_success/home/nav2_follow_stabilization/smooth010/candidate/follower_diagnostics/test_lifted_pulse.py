"""Memory/fake-clock only. Does not open serial devices."""
import unittest
from unittest.mock import patch,Mock
from follower_lifted_pulse import PulsePort,Trial,main,stationary_observation_tick
from follower_motor_candidate import MotorCore,velocity_packet
from follower_motor_mock import MemoryTransport
from test_motor_candidate import Clock


class Tests(unittest.TestCase):
    def test_post_stop_observation_stays_zero(self):
        clock,device,port,core,trial=self.make();self.advance(clock,trial,250)
        self.assertEqual(trial.phase,'DONE');observer=Mock()
        for _ in range(100):
            clock.now+=.01
            stationary_observation_tick(core,observer,clock.ros)
            self.assertEqual(device.rpm,(0,0));self.assertFalse(core.armed)
        self.assertTrue(observer.publish.called)

    def test_post_stop_rejects_armed_core(self):
        clock,device,port,core,trial=self.make();self.advance(clock,trial,115)
        with self.assertRaises(RuntimeError):stationary_observation_tick(core,Mock(),clock.ros)

    def test_ground_default_no_open(self):
        with patch('sys.argv',['ground']),patch.object(PulsePort,'open') as opened:
            self.assertEqual(main(ground=True),0);opened.assert_not_called()

    def test_ground_missing_ack_no_open(self):
        with patch('sys.argv',['ground','--run-ground-forward-pulse']),patch.object(PulsePort,'open') as opened:
            with self.assertRaises(SystemExit):main(ground=True)
            opened.assert_not_called()

    def test_ground_rejects_lifted_ack(self):
        with patch('sys.argv',['ground','--run-ground-forward-pulse','--ack-wheels-raised-power-cut-ready']),patch.object(PulsePort,'open') as opened:
            with self.assertRaises(SystemExit):main(ground=True)
            opened.assert_not_called()

    def make(self):
        clock=Clock();device=MemoryTransport();port=PulsePort(device)
        core=MotorCore(port,dry_run=False,allow_motion=True,clock=clock)
        return clock,device,port,core,Trial(core,port)

    def advance(self,clock,trial,n):
        for _ in range(n):
            with patch('follower_lifted_pulse.time.monotonic',return_value=clock.now):
                trial.step(clock.ros)
            clock.now+=.01
            if trial.phase in ('DONE','FAILED'):break

    def test_finite_pulse_and_measured_stop(self):
        clock,device,port,core,trial=self.make()
        self.advance(clock,trial,115)
        self.assertEqual(trial.phase,'PULSE');self.assertEqual(device.rpm,(-3,3))
        self.advance(clock,trial,150)
        self.assertEqual(trial.phase,'DONE');self.assertEqual(device.rpm,(0,0))
        self.assertFalse(core.armed);self.assertTrue(port.used)

    def test_window_refuses_late_or_other_packet(self):
        clock,device,port,core,trial=self.make();port.enable_once(clock.now)
        with patch('follower_lifted_pulse.time.monotonic',return_value=clock.now+.51):
            with self.assertRaises(ValueError):port.write(velocity_packet(-3,3))
        with self.assertRaises(ValueError):port.write(velocity_packet(3,3))
        with self.assertRaises(RuntimeError):port.enable_once(clock.now)

    def test_feedback_loss_fails_and_disarms(self):
        clock,device,port,core,trial=self.make();self.advance(clock,trial,115)
        device.feedback_enabled=False;self.advance(clock,trial,80)
        self.assertEqual(trial.phase,'FAILED');self.assertFalse(core.armed)
        self.assertEqual(device.rpm,(0,0))

    def test_no_feedback_never_pulses(self):
        clock,device,port,core,trial=self.make();device.feedback_enabled=False
        self.advance(clock,trial,350)
        self.assertEqual(trial.phase,'FAILED');self.assertFalse(port.used)

    def test_interrupted_loop_still_shutdown_zero(self):
        clock,device,port,core,trial=self.make();self.advance(clock,trial,115)
        core.shutdown();self.assertEqual(device.rpm,(0,0));self.assertTrue(device.closed)

    def test_default_cli_no_open(self):
        with patch('sys.argv',['trial']),patch.object(PulsePort,'open') as opened:
            self.assertEqual(main(),0);opened.assert_not_called()

    def test_missing_ack_no_open(self):
        with patch('sys.argv',['trial','--run-lifted-forward-pulse']),patch.object(PulsePort,'open') as opened:
            with self.assertRaises(SystemExit):main()
            opened.assert_not_called()


if __name__=='__main__':unittest.main()
