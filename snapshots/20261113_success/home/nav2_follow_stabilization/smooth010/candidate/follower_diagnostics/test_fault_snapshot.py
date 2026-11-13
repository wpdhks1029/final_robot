import json
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import patch

from fault_snapshot import capture_fault, FaultSnapshotWriter
from follower_drive_candidate import DriveCore
from test_motor_candidate import FakeSerial, Clock
from follower_drive_node import main


class FaultSnapshotTests(unittest.TestCase):
    def snapshot(self):
        core = DriveCore(config=None)
        core.io_history.append(dict(event='tx', packet_hex='aa'))
        core._record_feedback(dict(m1_rpm=0,m2_rpm=0,m1_status=0,m2_status=1,
                                   m1_position=0,m2_position=1), 1.)
        return core, capture_fault(core, 123, [], dict(core_s=.001))

    def test_snapshot_retains_device_fault_and_detaches_history(self):
        core, snapshot = self.snapshot()
        core.io_history[0]['packet_hex'] = 'changed'
        core.feedback['m2_status'] = 0
        self.assertEqual(snapshot['io_history'][0]['packet_hex'], 'aa')
        self.assertEqual(snapshot['feedback']['m2_status'], 1)
        self.assertEqual(snapshot['fault'], 'DEVICE_STATUS_NONZERO')
        self.assertFalse(snapshot['armed'])

    def test_healthy_core_is_not_recorded(self):
        with self.assertRaises(ValueError):
            capture_fault(DriveCore(), 1, [], {})

    def test_atomic_file_has_fault_and_no_temporary_left(self):
        _, snapshot = self.snapshot()
        with tempfile.TemporaryDirectory() as directory:
            writer = FaultSnapshotWriter(directory)
            self.assertTrue(writer.submit(snapshot))
            self.assertTrue(writer.close(timeout=1.))
            files = list(Path(directory).glob('*.json'))
            self.assertEqual(len(files), 1)
            self.assertEqual(json.loads(files[0].read_text()), snapshot)
            self.assertFalse(list(Path(directory).glob('*.tmp')))
            self.assertFalse(writer.errors)
            self.assertFalse(writer.submit(snapshot))

    def test_slow_disk_does_not_block_submit_or_stop_path(self):
        entered, release = threading.Event(), threading.Event()
        _, snapshot = self.snapshot()
        original = FaultSnapshotWriter._write
        def slow_write(writer, data):
            entered.set()
            release.wait(2.)
            original(writer, data)
        with tempfile.TemporaryDirectory() as directory:
            with patch.object(FaultSnapshotWriter, '_write', slow_write):
                writer = FaultSnapshotWriter(directory)
                try:
                    self.assertTrue(writer.submit(snapshot))
                    self.assertTrue(entered.wait(1.))
                    self.assertTrue(writer.submit(snapshot))
                    self.assertFalse(writer.submit(snapshot))
                    # Exercise the real motor tick on a memory transport while
                    # the file writer is intentionally blocked.
                    clock, transport = Clock(), FakeSerial()
                    core = DriveCore(transport, dry_run=False, clock=clock)
                    core.last_sent = (-4,4)
                    core.latch_fault('DEVICE_STATUS_NONZERO')
                    core.tick(clock.ros)
                    self.assertEqual(core.last_sent, (0,0))
                    self.assertTrue(transport.sent)
                finally:
                    release.set()
                    self.assertTrue(writer.close(timeout=1.))

    def test_write_failure_is_retained_and_does_not_escape(self):
        _, snapshot = self.snapshot()
        with tempfile.TemporaryDirectory() as directory:
            writer = FaultSnapshotWriter(directory)
            with patch.object(writer, '_write', side_effect=OSError('disk full')):
                self.assertTrue(writer.submit(snapshot))
                self.assertTrue(writer.close(timeout=1.))
            self.assertIn('disk full', writer.errors[0])

    def test_option_without_run_creates_no_directory_or_writer(self):
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory)/'unused'
            with patch('sys.argv', ['driver','--fault-snapshot-dir',str(target)]):
                self.assertEqual(main(), 0)
            self.assertFalse(target.exists())


if __name__ == '__main__':
    unittest.main()
