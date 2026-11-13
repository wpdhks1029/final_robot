#!/usr/bin/env python3
"""One raised-wheel channel pulse: left=M1 -3RPM, right=M2 +3RPM.

Physical left/right wiring is not verified. Default opens nothing. No ROS,
setting writes, repeats or auto rearm. Software cannot guarantee stopping.
"""
import argparse
import json
import math
from pathlib import Path
import time

from follower_drive_candidate import DriveCore
from follower_lifted_diagnostic import Evidence, execute
from follower_lifted_pulse import Trial
from follower_motor_candidate import read_request, velocity_packet
from follower_motor_port import ExclusivePort


class SingleWheelPort(ExclusivePort):
    def __init__(self, device, wheel='left', pulse_seconds=.2):
        super().__init__(device)
        self.used = False
        self.deadline = None
        self.set_duration(pulse_seconds)
        self.select(wheel)

    def set_duration(self, seconds):
        if self.used or seconds not in (.2,.5):
            raise ValueError('Only unused 0.2s or 0.5s pulse permitted')
        self.pulse_seconds = seconds

    def select(self, wheel):
        if self.used or wheel not in ('left','right'):
            raise ValueError('Invalid or already used channel')
        self.wheel = wheel
        self.target = (-3,0) if wheel == 'left' else (0,3)

    def enable_once(self, now):
        if self.used:
            raise RuntimeError('One pulse only')
        self.used = True
        self.deadline = now + self.pulse_seconds

    def write(self, packet):
        if packet in (read_request(1), velocity_packet(0,0)):
            return self.device.write(packet)
        if (packet == velocity_packet(*self.target) and self.deadline is not None
                and time.monotonic() < self.deadline):
            return self.device.write(packet)
        raise ValueError('Only selected single-wheel packet within chosen window permitted')


class SingleWheelTrial(Trial):
    def requested_motion(self):
        speed = 2*math.pi*.065*3/60
        return speed/2, (-1 if self.port.wheel == 'left' else 1)*speed/.560

    def step(self, ros_ns):
        super().step(ros_ns)
        feedback = self.core.feedback
        if self.phase in ('PULSE','VERIFY_STOP') and feedback is not None:
            inactive = 'm2_rpm' if self.port.wheel == 'left' else 'm1_rpm'
            reason = None
            if feedback[inactive] != 0:
                reason = 'INACTIVE_CHANNEL_MOVING'
            elif max(abs(feedback[k]) for k in ('m1_rpm','m2_rpm')) > 6:
                reason = 'SINGLE_WHEEL_FEEDBACK_OVERSPEED'
            if reason:
                self.core.latch_fault(reason)
                self.core.disarm()
                self.core.tick(ros_ns)
                self.error = reason
                self.phase = 'FAILED'


def classify_response(result, samples, wheel):
    selected = 'm1' if wheel == 'left' else 'm2'
    baseline = [r for r in samples if r['phase']=='BASELINE']
    active = [r for r in samples if r['phase']!='BASELINE']
    rpm_peak = max((abs(r['feedback'][selected+'_rpm']) for r in active), default=0)
    delta = None
    if baseline and active:
        before = baseline[-1]['feedback'][selected+'_position']
        after = active[-1]['feedback'][selected+'_position']
        delta = ((after-before+2**31) % 2**32)-2**31
    updated = dict(result, selected_rpm_peak=rpm_peak,
                   selected_position_delta_raw=delta,
                   controller_phase=result['phase'])
    if result['phase']=='DONE' and rpm_peak==0 and delta in (None,0):
        updated.update(phase='INCONCLUSIVE',error='NO_SELECTED_CHANNEL_RESPONSE')
    return updated


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', action='store_true')
    parser.add_argument('--wheel', choices=('left','right'))
    parser.add_argument('--pulse-seconds', type=float, choices=(.2,.5), default=.2)
    parser.add_argument('--ack-wheels-raised-power-cut-ready', action='store_true')
    args = parser.parse_args()
    if not args.run:
        parser.print_help()
        return 0
    if not args.wheel or not args.ack_wheels_raised_power_cut_ready:
        parser.error('Select one wheel and acknowledge raised wheels/secured chassis/power cut')
    import yaml
    cfg = yaml.safe_load(Path(__file__).with_name('follower_hardware.yaml').read_text())['follower_md200t_pid4']['ros__parameters']
    if (cfg['driver_id'] != 1 or cfg['baudrate'] != 19200
            or cfg['wheel_radius'] != .065 or cfg['wheel_track'] != .560
            or cfg['invert_left'] is not True or cfg['invert_right'] is not False
            or not cfg['port'].endswith('usb-0:1.4:1.0-port0')):
        raise RuntimeError('Confirmed hardware configuration changed')
    directory = Path(__file__).with_name('logs')
    directory.mkdir(exist_ok=True)
    output = directory / ('single_wheel_'+args.wheel+'_'+str(time.time_ns())+'.json')
    with output.open('x'):
        pass
    core = port = None
    evidence = Evidence()
    result = dict(phase='NOT_STARTED', physical_stop_verified=False)
    try:
        port = SingleWheelPort.open(cfg['port'], 19200)
        port.select(args.wheel)
        port.set_duration(args.pulse_seconds)
        core = DriveCore(port, dry_run=False, allow_motion=True)
        result = execute(core, port, evidence, trial_factory=SingleWheelTrial)
        result = classify_response(result,evidence.feedback,args.wheel)
    except (KeyboardInterrupt, Exception) as exc:
        result = dict(phase='INTERRUPTED_OR_FAILED', error=type(exc).__name__+': '+str(exc),
                      fault=None if core is None else core.fault, physical_stop_verified=False)
    finally:
        try:
            if core is not None:
                core.shutdown()
            elif port is not None:
                port.close()
        finally:
            output.write_text(json.dumps(dict(version=1, channel=args.wheel,
                raw_rpm_target=[-3,0] if args.wheel=='left' else [0,3],
                software_window_s=args.pulse_seconds, result=result, io_events=evidence.events,
                feedback_samples=evidence.feedback,
                limitations=['Physical channel mapping and physical stop are not verified.']),
                allow_nan=False, indent=2)+'\n')
            print('REPORT='+str(output), flush=True)
    print(json.dumps(result), flush=True)
    return 0 if result.get('phase') == 'DONE' else 1


if __name__ == '__main__':
    raise SystemExit(main())
