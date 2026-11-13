import errno
import os
import pty
import tty
import unittest
from types import SimpleNamespace as S
from unittest.mock import Mock,patch
from follower_motor_port import SingleWriteSerial,ExclusivePort
from follower_motor_candidate import MotorCore,read_request
from test_motor_candidate import Clock


class Tests(unittest.TestCase):
    def adapter(self):
        device=Mock();device.fileno.return_value=99;device.in_waiting=0
        with patch('follower_motor_port.fcntl.fcntl',return_value=os.O_NONBLOCK):
            adapter=SingleWriteSerial(device)
        return device,adapter

    def test_one_os_write_without_pyserial_write(self):
        d,a=self.adapter()
        with patch('follower_motor_port.os.write',return_value=7) as write:
            self.assertEqual(a.write(read_request()),7)
            write.assert_called_once_with(99,read_request())
        d.write.assert_not_called()

    def test_full_write_no_elapsed_timeout(self):
        d,a=self.adapter();clock=Clock()
        def delayed(fd,packet):
            clock.now+=.05
            return len(packet)
        k=MotorCore(ExclusivePort(a),dry_run=False,clock=clock)
        with patch('follower_motor_port.os.write',side_effect=delayed):k.tick(clock.ros)
        self.assertIsNone(k.fault)
        self.assertFalse(k.io_failed)

    def test_eagain_and_eintr_not_retried_inside_adapter(self):
        for exc in [BlockingIOError(errno.EAGAIN,'busy'),InterruptedError(errno.EINTR,'interrupted')]:
            d,a=self.adapter()
            with patch('follower_motor_port.os.write',side_effect=exc) as write:
                with self.assertRaises(OSError):a.write(read_request())
                self.assertEqual(write.call_count,1)

    def test_partial_write_still_latches_core_fault(self):
        d,a=self.adapter();clock=Clock();k=MotorCore(ExclusivePort(a),dry_run=False,clock=clock)
        with patch('follower_motor_port.os.write',return_value=2):k.tick(clock.ros)
        self.assertEqual(k.fault,'SERIAL_WRITE_ERROR')
        self.assertEqual(k.last_io_error['written_bytes'],2)
        self.assertFalse(k.armed)

    def test_blocking_descriptor_rejected(self):
        d=Mock();d.fileno.return_value=99
        with patch('follower_motor_port.fcntl.fcntl',return_value=0):
            with self.assertRaises(RuntimeError):SingleWriteSerial(d)

    def test_pseudo_terminal_actual_nonblocking_bytes(self):
        master,slave=pty.openpty()
        try:
            tty.setraw(slave);os.set_blocking(slave,False)
            adapter=SingleWriteSerial(S(fileno=lambda:slave))
            packet=read_request()
            self.assertEqual(adapter.write(packet),len(packet))
            self.assertEqual(os.read(master,len(packet)),packet)
        finally:
            os.close(master);os.close(slave)


if __name__=='__main__':unittest.main()
