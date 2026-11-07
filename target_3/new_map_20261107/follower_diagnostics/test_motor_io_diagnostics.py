import unittest
from unittest.mock import patch
from follower_motor_candidate import MotorCore,read_request
from test_motor_candidate import FakeSerial,Clock


class Tests(unittest.TestCase):
    def make(self):
        c=Clock();d=FakeSerial();return c,d,MotorCore(d,dry_run=False,clock=c)

    def test_write_exception_retained(self):
        c,d,k=self.make();d.write_error=True;k.tick(c.ros)
        self.assertEqual(k.last_io_error['operation'],'write')
        self.assertEqual(k.last_io_error['type'],'OSError')
        self.assertEqual(k.last_io_error['message'],'fake write failure')
        self.assertTrue(k.io_failed);self.assertFalse(k.feedback_fresh())

    def test_partial_write_count_retained(self):
        c,d,k=self.make();d.partial=True;k.tick(c.ros)
        e=k.last_io_error;self.assertEqual(e['written_bytes'],e['expected_bytes']-1)
        self.assertEqual(k.fault,'SERIAL_WRITE_ERROR')

    def test_reset_failure_distinguished_from_write(self):
        c,d,k=self.make()
        with patch.object(d,'reset_input_buffer',side_effect=OSError('USB disconnected')):
            k._transmit(read_request(),c.now,preempt=True)
        self.assertEqual(k.last_io_error['operation'],'reset_input_before_write')
        self.assertEqual(k.last_io_error['message'],'USB disconnected')

    def test_read_failure_retained(self):
        c,d,k=self.make();d.read_error=True;k.tick(c.ros)
        self.assertEqual(k.fault,'SERIAL_READ_ERROR')
        self.assertEqual(k.last_io_error['type'],'OSError')

    def test_response_timeout_preserves_stop_context(self):
        c,d,k=self.make();d.autoreply=False;k.tick(c.ros)
        c.now+=.31;k.tick(c.ros)
        e=k.last_response_error
        self.assertEqual(k.fault,'RESPONSE_TIMEOUT')
        self.assertFalse(e['armed']);self.assertEqual(e['last_sent'],[0,0])
        self.assertGreaterEqual(e['waiting_s'],.3)
        self.assertEqual(e['valid_feedback'],0)
        self.assertEqual(e['history'][-1]['event'],'tx')
        self.assertEqual(e['history'][-1]['packet_hex'].split()[3],'cf')

    def test_trace_is_bounded(self):
        c,d,k=self.make()
        for _ in range(400):
            k.tick(c.ros);c.now+=.01
        self.assertIsNone(k.fault);self.assertEqual(len(k.io_history),24)


if __name__=='__main__':unittest.main()
