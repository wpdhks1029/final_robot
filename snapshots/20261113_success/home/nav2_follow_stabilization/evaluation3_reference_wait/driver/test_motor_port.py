"""Port-ownership failure paths using mocks only; never touches /dev or serial."""
import os
import stat
import sys
from types import SimpleNamespace as S
import unittest
from unittest.mock import Mock,patch

from follower_motor_port import ExclusivePort,owners
from follower_motor_candidate import read_request,velocity_packet


class PortTests(unittest.TestCase):
    def test_fuser_device_argument_compatibility(self):
        with patch('follower_motor_port.subprocess.run',return_value=S(returncode=1,stdout='',stderr='')) as run:
            self.assertEqual(owners('/dev/ttyUSB0'),set())
            self.assertEqual(run.call_args.args[0],['fuser','/dev/ttyUSB0'])
        with self.assertRaises(ValueError):owners('--kill')

    def setup_mocks(self,results):
        device=Mock();device.fileno.return_value=99
        factory=Mock(return_value=device)
        stack=[patch('follower_motor_port.os.path.realpath',return_value='/dev/FAKE'),
               patch('follower_motor_port.os.stat',return_value=S(st_mode=stat.S_IFCHR)),
               patch('follower_motor_port.subprocess.run',side_effect=results),
               patch('follower_motor_port.fcntl.ioctl'),
               patch('follower_motor_port.fcntl.fcntl',return_value=os.O_NONBLOCK),
               patch.dict(sys.modules,{'serial':S(Serial=factory)})]
        for item in stack:item.start();self.addCleanup(item.stop)
        return device,factory

    def empty(self):return S(returncode=1,stdout='',stderr='')
    def owner(self,pid):return S(returncode=0,stdout=str(pid),stderr='/dev/FAKE:')

    def test_success_pre_post_and_close(self):
        device,factory=self.setup_mocks([self.empty(),self.owner(os.getpid())])
        port=ExclusivePort.open('/dev/FAKE')
        args=factory.call_args.kwargs
        self.assertTrue(args['exclusive']);self.assertEqual(args['timeout'],0)
        self.assertEqual(args['write_timeout'],0)
        port.close();port.close();device.close.assert_called_once()

    def test_busy_refuses_before_serial_open(self):
        device,factory=self.setup_mocks([self.owner(12345)])
        with self.assertRaises(RuntimeError):ExclusivePort.open('/dev/FAKE')
        factory.assert_not_called()

    def test_fuser_error_refuses(self):
        device,factory=self.setup_mocks([S(returncode=1,stdout='',stderr='Permission denied')])
        with self.assertRaises(RuntimeError):ExclusivePort.open('/dev/FAKE')
        factory.assert_not_called()

    def test_unknown_fuser_output_refuses(self):
        device,factory=self.setup_mocks([S(returncode=0,stdout='unexpected',stderr='')])
        with self.assertRaises(RuntimeError):ExclusivePort.open('/dev/FAKE')
        factory.assert_not_called()

    def test_owner_race_closes_without_sending(self):
        device,factory=self.setup_mocks([self.empty(),S(returncode=0,stdout=f'{os.getpid()} 12345',stderr='')])
        with self.assertRaises(RuntimeError):ExclusivePort.open('/dev/FAKE')
        device.close.assert_called_once();device.write.assert_not_called()

    def test_missing_post_owner_closes(self):
        device,factory=self.setup_mocks([self.empty(),self.empty()])
        with self.assertRaises(RuntimeError):ExclusivePort.open('/dev/FAKE')
        device.close.assert_called_once();device.write.assert_not_called()

    def test_kernel_exclusive_failure_closes(self):
        device,factory=self.setup_mocks([self.empty()])
        with patch('follower_motor_port.fcntl.ioctl',side_effect=OSError('fake ioctl error')):
            with self.assertRaises(OSError):ExclusivePort.open('/dev/FAKE')
        device.close.assert_called_once();device.write.assert_not_called()

    def test_regular_file_rejected(self):
        device,factory=self.setup_mocks([])
        with patch('follower_motor_port.os.stat',return_value=S(st_mode=stat.S_IFREG)):
            with self.assertRaises(RuntimeError):ExclusivePort.open('/dev/FAKE')
        factory.assert_not_called()

    def test_wrong_baud_rejected(self):
        device,factory=self.setup_mocks([])
        with self.assertRaises(ValueError):ExclusivePort.open('/dev/FAKE',57600)
        factory.assert_not_called()

    def test_serial_open_error_propagates(self):
        device,factory=self.setup_mocks([self.empty()]);factory.side_effect=OSError('fake open failure')
        with self.assertRaises(OSError):ExclusivePort.open('/dev/FAKE')

    def test_transparent_nonblocking_methods(self):
        device=Mock();device.in_waiting=7;device.read.return_value=b'abc'
        port=ExclusivePort(device)
        self.assertEqual(port.in_waiting,7);self.assertEqual(port.read(3),b'abc')
        port.write(read_request());port.reset_input_buffer()
        device.write.assert_called_once_with(read_request());device.reset_input_buffer.assert_called_once()

    def test_byte_boundary_blocks_motion_and_other_packets(self):
        device=Mock();port=ExclusivePort(device)
        for raw in (velocity_packet(-3,3),read_request(2),b'bad',velocity_packet(0,1)):
            with self.assertRaises(ValueError):port.write(raw)
        device.write.assert_not_called()
        port.write(velocity_packet(0,0));device.write.assert_called_once_with(velocity_packet(0,0))


if __name__=='__main__':unittest.main(verbosity=2)
