import contextlib
import io
import struct
import unittest
from unittest.mock import patch

from follower_drive_candidate import DriveCore
from follower_lifted_diagnostic import execute, Evidence, main, PulsePort
from follower_motor_mock import MemoryTransport
from test_motor_candidate import Clock


class AlarmMemory(MemoryTransport):
    def __init__(self, when):
        super().__init__()
        self.when = when
        self.moving_sent = 0

    def write(self, packet):
        result = super().write(packet)
        moving = packet[3] == 207 and self.rpm != (0,0)
        self.moving_sent += int(moving)
        if self.when == 'baseline' or (self.when == 'moving' and moving):
            data = struct.pack('<hBihBi',0,0,1,0,1,2)
            raw = bytes([0xB8,0xB7,1,216,14])+data
            self.rx[:] = raw+bytes([(-sum(raw)) & 255])
        return result


class LiftedDiagnosticTests(unittest.TestCase):
    def run_trial(self, device=None, interrupt=False):
        clock = Clock()
        device = device or MemoryTransport()
        port = PulsePort(device)
        core = DriveCore(port, dry_run=False, allow_motion=True, clock=clock)
        evidence = Evidence()
        def sleep(dt):
            clock.now += dt
            if interrupt and clock.now > 2.2:
                raise KeyboardInterrupt()
        with patch('follower_lifted_pulse.time.monotonic', side_effect=clock), \
             patch.object(port, 'close', side_effect=device.close), \
             contextlib.redirect_stdout(io.StringIO()):
            if interrupt:
                with self.assertRaises(KeyboardInterrupt):
                    execute(core,port,evidence,stamp=lambda:clock.ros,sleep=sleep)
                result = None
            else:
                result = execute(core,port,evidence,stamp=lambda:clock.ros,sleep=sleep)
        return result, device, core, evidence

    def test_one_half_second_pulse_and_evidence(self):
        result, device, core, evidence = self.run_trial()
        self.assertEqual(result['phase'], 'DONE')
        self.assertFalse(result['physical_stop_verified'])
        self.assertTrue(core.closed)
        self.assertEqual(device.rpm, (0,0))
        moving = [r for r in evidence.events if r['event']=='tx' and
                  r['packet_hex'].split()[3]=='cf' and
                  r['packet_hex'] != 'b7 b8 01 cf 07 01 00 00 01 00 00 01 b7']
        self.assertTrue(moving)
        for row in moving:
            p = bytes.fromhex(row['packet_hex'])
            self.assertEqual((struct.unpack_from('<h',p,6)[0],
                              struct.unpack_from('<h',p,9)[0]),(-3,3))
        self.assertLess(moving[-1]['mono']-moving[0]['mono'],.5)

    def test_baseline_device_alarm_prevents_pulse(self):
        result, device, core, evidence = self.run_trial(AlarmMemory('baseline'))
        self.assertEqual(result['fault'], 'DEVICE_STATUS_NONZERO')
        self.assertFalse(result['pulse_used'])
        self.assertEqual(device.moving_sent, 0)

    def test_motion_device_alarm_aborts_without_retry(self):
        result, device, core, evidence = self.run_trial(AlarmMemory('moving'))
        self.assertEqual(result['phase'], 'FAILED')
        self.assertEqual(result['fault'], 'DEVICE_STATUS_NONZERO')
        self.assertEqual(device.moving_sent, 1)
        self.assertEqual(device.rpm,(0,0))
        self.assertFalse(core.armed)

    def test_interrupt_attempts_zero_and_closes(self):
        result, device, core, evidence = self.run_trial(interrupt=True)
        self.assertTrue(core.closed)
        self.assertEqual(device.rpm, (0,0))
        self.assertFalse(core.armed)

    def test_default_and_missing_ack_never_open(self):
        with patch.object(PulsePort,'open') as opened, contextlib.redirect_stdout(io.StringIO()):
            with patch('sys.argv',['diagnostic']):
                self.assertEqual(main(),0)
            with patch('sys.argv',['diagnostic','--run-lifted-forward-pulse']):
                with self.assertRaises(SystemExit):
                    main()
            opened.assert_not_called()


if __name__ == '__main__':
    unittest.main()
