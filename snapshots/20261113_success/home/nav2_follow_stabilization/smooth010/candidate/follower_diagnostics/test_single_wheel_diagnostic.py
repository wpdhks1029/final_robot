import contextlib
import io
import struct
import unittest
from unittest.mock import patch

from follower_single_wheel_diagnostic import SingleWheelPort, SingleWheelTrial, main, classify_response
from follower_lifted_diagnostic import Evidence, execute
from follower_drive_candidate import DriveCore
from follower_motor_mock import MemoryTransport
from follower_motor_candidate import velocity_packet
from test_motor_candidate import Clock


class WrongChannel(MemoryTransport):
    def write(self, packet):
        size = super().write(packet)
        if self.rpm != (0,0):
            data=struct.pack('<hBihBi',-3,0,1,3,0,2)
            raw=bytes([0xB8,0xB7,1,216,14])+data
            self.rx[:]=raw+bytes([(-sum(raw))&255])
        return size


class SingleWheelTests(unittest.TestCase):
    def simulate(self, wheel, device=None, seconds=.2):
        clock=Clock();device=device or MemoryTransport()
        port=SingleWheelPort(device,wheel,seconds)
        core=DriveCore(port,dry_run=False,allow_motion=True,clock=clock)
        evidence=Evidence()
        def advance(dt): clock.now+=dt
        with patch('follower_single_wheel_diagnostic.time.monotonic',side_effect=clock), \
             patch.object(port,'close',side_effect=device.close), \
             contextlib.redirect_stdout(io.StringIO()):
            result=execute(core,port,evidence,stamp=lambda:clock.ros,
                           sleep=advance,trial_factory=SingleWheelTrial)
        return result,device,core,evidence

    def test_each_channel_only_and_zero_on_completion(self):
        for wheel,target in [('left',(-3,0)),('right',(0,3))]:
            result,device,core,evidence=self.simulate(wheel)
            self.assertEqual(result['phase'],'DONE')
            moving=[]
            for row in evidence.events:
                if row['event']=='tx':
                    p=bytes.fromhex(row['packet_hex'])
                    if p[3]==207:
                        rpm=(struct.unpack_from('<h',p,6)[0],struct.unpack_from('<h',p,9)[0])
                        self.assertIn(rpm,[(0,0),target])
                        if rpm!=(0,0): moving.append(row)
            self.assertTrue(moving)
            self.assertLess(moving[-1]['mono']-moving[0]['mono'],.2)
            self.assertEqual(device.rpm,(0,0))
            self.assertTrue(core.closed)

    def test_inactive_channel_response_stops(self):
        for wheel in ('left','right'):
            result,device,core,evidence=self.simulate(wheel,WrongChannel())
            self.assertEqual(result['fault'],'INACTIVE_CHANNEL_MOVING')
            self.assertEqual(result['phase'],'FAILED')
            self.assertEqual(device.rpm,(0,0))

    def test_exact_packet_deadline_and_one_shot(self):
        device=MemoryTransport();port=SingleWheelPort(device,'left')
        port.enable_once(1.)
        with patch('follower_single_wheel_diagnostic.time.monotonic',return_value=1.1):
            port.write(velocity_packet(-3,0))
            for pair in [(0,3),(-3,3),(-4,0)]:
                with self.assertRaises(ValueError):port.write(velocity_packet(*pair))
        with patch('follower_single_wheel_diagnostic.time.monotonic',return_value=1.2):
            with self.assertRaises(ValueError):port.write(velocity_packet(-3,0))
        with self.assertRaises(RuntimeError):port.enable_once(2.)
        with self.assertRaises(ValueError):port.select('right')

    def test_default_or_missing_ack_does_not_open(self):
        with patch.object(SingleWheelPort,'open') as opened,contextlib.redirect_stdout(io.StringIO()):
            with patch('sys.argv',['single']):self.assertEqual(main(),0)
            with patch('sys.argv',['single','--run','--wheel','left']):
                with self.assertRaises(SystemExit):main()
            opened.assert_not_called()

    def test_half_second_is_still_one_selected_channel_pulse(self):
        for wheel in ('left','right'):
            result,device,core,evidence=self.simulate(wheel,seconds=.5)
            self.assertEqual(result['phase'],'DONE')
            target=(-3,0) if wheel=='left' else (0,3)
            tx=[e for e in evidence.events if e['event']=='tx' and
                e['packet_hex']==velocity_packet(*target).hex(' ')]
            self.assertTrue(tx)
            self.assertLess(tx[-1]['mono']-tx[0]['mono'],.5)
            self.assertEqual(device.rpm,(0,0))

    def test_no_motion_response_is_not_reported_done(self):
        samples=[dict(phase=phase,feedback=dict(m1_rpm=0,m2_rpm=0,
                    m1_position=0,m2_position=0)) for phase in ('BASELINE','DONE')]
        for wheel in ('left','right'):
            r=classify_response(dict(phase='DONE'),samples,wheel)
            self.assertEqual(r['phase'],'INCONCLUSIVE')
            self.assertEqual(r['error'],'NO_SELECTED_CHANNEL_RESPONSE')
            self.assertEqual(r['controller_phase'],'DONE')
        r=classify_response(dict(phase='FAILED',fault='DEVICE_STATUS_NONZERO'),samples,'left')
        self.assertEqual(r['phase'],'FAILED')
        self.assertEqual(r['fault'],'DEVICE_STATUS_NONZERO')

    def test_duration_cannot_increase_after_use(self):
        p=SingleWheelPort(MemoryTransport())
        with self.assertRaises(ValueError):p.set_duration(1.)
        p.enable_once(1.)
        with self.assertRaises(ValueError):p.set_duration(.5)


if __name__=='__main__':unittest.main()
