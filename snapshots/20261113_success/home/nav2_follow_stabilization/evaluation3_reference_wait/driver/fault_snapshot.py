"""Bounded, optional fault evidence writer; no ROS or motor interfaces.

Only first-fault snapshots are queued. Filesystem work runs on a separate
thread. This preserves an existing history window, not a continuous wire trace.
"""
from collections import deque
from copy import deepcopy
import json
import os
from pathlib import Path
import queue
import threading
import uuid


def capture_fault(core, ros_ns, tick_history, current_tick):
    if core.fault is None:
        raise ValueError('FAULT_REQUIRED')
    return deepcopy(dict(version=1, kind='motor_first_fault_snapshot',
        ros_ns=ros_ns, fault=core.fault, armed=core.armed,
        last_sent_rpm=list(core.last_sent), feedback=core.feedback,
        valid_feedback=core.valid_count,
        device_error=getattr(core, 'last_device_status_error', None),
        io_error=core.last_io_error, response_error=core.last_response_error,
        feedback_error=core.last_feedback_error,
        io_history=list(core.io_history), driver_tick_history=list(tick_history),
        current_tick=current_tick,
        limitations=['Bounded application-level history; not a continuous wire capture.',
                     'Successful write records do not prove device execution or physical stopping.']))


class FaultSnapshotWriter:
    def __init__(self, directory):
        self.directory = Path(directory).expanduser()
        self.directory.mkdir(parents=True, exist_ok=True)
        # Fail before opening a motor port if the selected directory is unwritable.
        probe = self.directory / ('.write_probe_' + uuid.uuid4().hex)
        with probe.open('x'):
            pass
        probe.unlink()
        self.pending = queue.Queue(maxsize=1)
        self.errors = deque(maxlen=4)
        self.written = deque(maxlen=4)
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True,
                                       name='motor-fault-evidence')
        self.thread.start()

    def submit(self, snapshot):
        if self.stop.is_set():
            return False
        try:
            self.pending.put_nowait(snapshot)
            return True
        except queue.Full:
            return False

    def _write(self, snapshot):
        target = self.directory / ('motor_fault_' + str(snapshot['ros_ns']) +
                                   '_' + uuid.uuid4().hex + '.json')
        temporary = target.with_suffix('.tmp')
        try:
            with temporary.open('x') as stream:
                json.dump(snapshot, stream, allow_nan=False, indent=2)
                stream.write('\n')
                stream.flush()
                os.fsync(stream.fileno())
            temporary.replace(target)
            self.written.append(str(target))
        finally:
            temporary.unlink(missing_ok=True)

    def _run(self):
        while not self.stop.is_set() or not self.pending.empty():
            try:
                snapshot = self.pending.get(timeout=.05)
            except queue.Empty:
                continue
            try:
                self._write(snapshot)
            except Exception as exc:
                self.errors.append(type(exc).__name__ + ': ' + str(exc)[:300])
            finally:
                self.pending.task_done()

    def close(self, timeout=.2):
        self.stop.set()
        self.thread.join(timeout)
        return not self.thread.is_alive()
