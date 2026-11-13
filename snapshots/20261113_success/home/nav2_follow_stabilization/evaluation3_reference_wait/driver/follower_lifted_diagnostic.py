#!/usr/bin/env python3
"""One lifted-wheel 3RPM/0.5s forward pulse with device-fault checking.

Default opens nothing. No ROS, repeated pulse, auto rearm or settings writes.
Software stopping is not guaranteed during USB/process/power faults.
"""
import argparse
import json
from pathlib import Path
import time

from follower_drive_candidate import DriveCore
from follower_lifted_pulse import PulsePort, Trial


class Evidence:
    def __init__(self):
        self.events = []
        self.seen = set()
        self.feedback = []
        self.last_count = 0

    def observe(self, core, phase, ros_ns):
        for row in core.io_history:
            if id(row) not in self.seen:
                # Retain references so object IDs cannot be recycled. History
                # entries are append-only. No serialization/disk work here.
                if len(self.events) >= 2048:
                    raise RuntimeError('EVIDENCE_LIMIT')
                self.seen.add(id(row))
                self.events.append(row)
        if core.valid_count != self.last_count:
            self.last_count = core.valid_count
            self.feedback.append(dict(ros_ns=ros_ns, mono=core.clock(),
                phase=phase, feedback=dict(core.feedback),
                sent_rpm=list(core.last_sent), fault=core.fault))


def execute(core, port, evidence, *, stamp=time.time_ns, sleep=time.sleep,
            trial_factory=Trial):
    trial = trial_factory(core, port)
    started = core.clock()
    previous = None
    try:
        while trial.phase not in ('DONE', 'FAILED'):
            ns = stamp()
            if core.clock() - started > 6.:
                core.latch_fault('LIFTED_DIAGNOSTIC_DEADLINE')
            trial.step(ns)
            evidence.observe(core, trial.phase, ns)
            if trial.phase != previous:
                print(json.dumps(dict(phase=trial.phase, fault=core.fault,
                    feedback=core.feedback, sent_rpm=core.last_sent)), flush=True)
                previous = trial.phase
            sleep(.01)
    finally:
        core.shutdown()
        evidence.observe(core, trial.phase, stamp())
    return dict(phase=trial.phase, error=trial.error, fault=core.fault,
                feedback=core.feedback, last_sent_rpm=list(core.last_sent),
                pulse_used=port.used,
                physical_stop_verified=False,
                meaning='DONE means feedback checks passed, not physical verification.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-lifted-forward-pulse', action='store_true')
    parser.add_argument('--ack-wheels-raised-power-cut-ready', action='store_true')
    args = parser.parse_args()
    if not args.run_lifted_forward_pulse:
        parser.print_help()
        return 0
    if not args.ack_wheels_raised_power_cut_ready:
        parser.error('Wheels raised, secured chassis and immediate power cut required')
    import yaml
    cfg = yaml.safe_load(Path(__file__).with_name('follower_hardware.yaml').read_text())['follower_md200t_pid4']['ros__parameters']
    if (cfg['driver_id'] != 1 or cfg['baudrate'] != 19200
            or cfg['wheel_radius'] != .065 or cfg['wheel_track'] != .560
            or cfg['invert_left'] is not True or cfg['invert_right'] is not False
            or not cfg['port'].endswith('usb-0:1.4:1.0-port0')):
        raise RuntimeError('Confirmed hardware configuration changed')
    directory = Path(__file__).with_name('logs')
    directory.mkdir(exist_ok=True)
    output = directory / ('lifted_diagnostic_' + str(time.time_ns()) + '.json')
    # Reserve/check the evidence destination before opening a serial port.
    with output.open('x'):
        pass
    evidence = Evidence()
    port = core = None
    result = dict(phase='NOT_STARTED')
    try:
        port = PulsePort.open(cfg['port'], 19200)
        core = DriveCore(port, dry_run=False, allow_motion=True)
        result = execute(core, port, evidence)
    except BaseException as exc:
        result = dict(phase='INTERRUPTED_OR_FAILED', error=type(exc).__name__+': '+str(exc),
                      fault=None if core is None else core.fault,
                      physical_stop_verified=False)
        if not isinstance(exc, (KeyboardInterrupt, Exception)):
            raise
    finally:
        try:
            if core is not None:
                core.shutdown()
            elif port is not None:
                port.close()
        finally:
            report = dict(version=1, result=result, io_events=evidence.events,
                          feedback_samples=evidence.feedback,
                          limitations=['Application-level transmit/receive records; not a wire analyzer.',
                                       'Reported encoder zero cannot prove physical stopping.'])
            output.write_text(json.dumps(report, allow_nan=False, indent=2)+'\n')
            print('REPORT='+str(output), flush=True)
    print(json.dumps(result), flush=True)
    return 0 if result.get('phase') == 'DONE' else 1


if __name__ == '__main__':
    raise SystemExit(main())
