#!/usr/bin/env python3
"""Continuous measured odom, with ONE explicit terminal-triggered ground pulse.

Default opens nothing. Hardware mode starts DISARMED; type START only after
preparation/recording. Fixed 3RPM/0.5s once. No ROS motion subscriptions/services.
Software cannot stop a motor on SIGKILL, USB loss or process suspension.
"""
import argparse
import json
from pathlib import Path
import select
import sys
import time

from follower_lifted_pulse import PulsePort,Trial
from follower_motor_candidate import MotorCore
from follower_feedback_trace import FeedbackTrace
from follower_lifted_turn import TurnPort,TurnTrial


class MeasurementPort(PulsePort):
    """Fixed 5-second command window; same exact +/-3 RPM packet allowlist."""
    def enable_once(self,now):
        if self.used:raise RuntimeError('Only one pulse per process')
        self.used=True;self.deadline=now+5.


class TurnMeasurementPort(TurnPort):
    """Single fixed +3/+3 RPM turn window, maximum 5 seconds."""
    def enable_once(self,now):
        if self.used:raise RuntimeError('Only one turn per process')
        self.used=True;self.deadline=now+5.


class Session:
    def __init__(self,core,port,trial_factory=Trial):
        self.core=core;self.port=port;self.trial=None;self.used=False
        self.trial_factory=trial_factory
        self.phase='WAIT';self.zero_samples=0;self.previous=core.valid_count

    def ready(self):
        c=self.core
        return (not self.used and self.phase=='WAIT' and not c.fault
                and c.feedback_fresh() and self.zero_samples>=5
                and c.feedback['m1_rpm']==c.feedback['m2_rpm']==0)

    def request_start(self):
        if not self.ready():return False
        self.used=True;self.trial=self.trial_factory(self.core,self.port);self.phase='BASELINE'
        return True

    def stop(self):
        self.core.disarm();self.used=True
        if self.phase!='FAULT':self.phase='ABORTED'

    def step(self,stamp):
        c=self.core
        if self.trial is not None and self.phase not in ('DONE','FAULT','ABORTED'):
            self.trial.step(stamp);self.phase=self.trial.phase
            if self.phase=='FAILED':self.phase='FAULT'
        else:
            c.tick(stamp)
            if c.valid_count!=self.previous:
                zero=c.feedback['m1_rpm']==c.feedback['m2_rpm']==0
                self.zero_samples=self.zero_samples+1 if zero else 0
                if not zero:c.latch_fault('UNEXPECTED_MOTION_OUTSIDE_PULSE')
        self.previous=c.valid_count
        if c.fault:self.phase='FAULT'
        if self.phase=='FAULT':c.disarm()


def main(*,measurement=False,turn=False):
    if turn and not measurement:raise ValueError('Turn mode requires measurement mode')
    description=('Continuous odom measurement trial: raw M1=-3/M2=+3 RPM, '
                 '5-second software maximum, once. Starts WAIT. MEASURE_5S triggers motion; '
                 'STOP requests stop. No obstacle guard/hardware watchdog.' if measurement else __doc__)
    if turn:
        description=('Ground CCW turn measurement: raw M1=+3/M2=+3 RPM, max 5s once. '
                     'Starts WAIT; TURN_5S requests turn. STOP aborts. No obstacle guard/hardware watchdog.')
    p=argparse.ArgumentParser(description=description)
    run_flag='--run-turn-measurement-session' if turn else ('--run-measurement-session' if measurement else '--run-session')
    p.add_argument(run_flag,dest='run_session',action='store_true')
    ack_flag='--ack-clear-turn-area-power-cut-ready' if turn else '--ack-clear-1m-power-cut-ready'
    p.add_argument(ack_flag,dest='ack_preparation',action='store_true')
    if measurement:p.add_argument('--capture-comparison',action='store_true',
                                  help='Record raw/EKF/test position topics in this process; print summary 15s after DONE')
    if measurement:p.add_argument('--capture-position-ekf',action='store_true',
                                  help='Also run a fresh TEST position EKF without dynamic TF; implies capture-comparison')
    args=p.parse_args()
    if measurement and args.capture_position_ekf:args.capture_comparison=True
    if not args.run_session:p.print_help();return 0
    if not args.ack_preparation:p.error('Physical preparation acknowledgment required')
    if not sys.stdin.isatty():p.error('Use an interactive terminal; redirected/piped START is refused')
    import yaml
    from follower_pulse_odom import Observer
    cfg=yaml.safe_load(Path(__file__).with_name('follower_hardware.yaml').read_text())['follower_md200t_pid4']['ros__parameters']
    if (cfg['baudrate']!=19200 or cfg['driver_id']!=1 or cfg['wheel_radius']!=.065
            or cfg['wheel_track']!=.560 or cfg['invert_left'] is not True
            or cfg['invert_right'] is not False or not cfg['port'].endswith('usb-0:1.4:1.0-port0')):
        raise RuntimeError('Confirmed hardware configuration changed')
    core=None;port=None;observer=None;trace=None;trace_stream=None
    try:
        if measurement and args.capture_comparison:
            if args.capture_position_ekf:
                from follower_ekf_comparison_observer import EkfComparisonObserver
                observer=EkfComparisonObserver()
            else:
                from follower_comparison_observer import ComparisonObserver
                observer=ComparisonObserver()
        else:observer=Observer()  # Conflicting odom publisher rejected before port opening.
        if measurement:
            directory=Path(__file__).parent/'logs';directory.mkdir(exist_ok=True)
            prefix='turn_measurement_feedback' if turn else 'measurement_feedback'
            trace_path=directory/f'{prefix}_{time.time_ns()}.jsonl'
            trace_stream=trace_path.open('x');trace=FeedbackTrace(trace_stream)
            print('FEEDBACK_TRACE='+str(trace_path),flush=True)
        port_class=TurnMeasurementPort if turn else (MeasurementPort if measurement else PulsePort)
        port=port_class.open(cfg['port'],19200)
        core=MotorCore(port,dry_run=False,allow_motion=True)
        session=Session(core,port,trial_factory=TurnTrial if turn else Trial)
        token='TURN_5S' if turn else ('MEASURE_5S' if measurement else 'START')
        print(f'WAIT: query/zero only. {token} + Enter requests ONE pulse; STOP + Enter aborts; Ctrl+C exits. No automatic following.',flush=True)
        last=0.;previous=None;capture_done_at=None
        while True:
            if select.select([sys.stdin],[],[],0)[0]:
                line=sys.stdin.readline()
                if not line:break
                if line.strip()=='STOP':
                    session.stop();print('STOP_REQUESTED: disarmed, no rearm in this session.',flush=True)
                elif line.strip()==token:
                    if measurement and args.capture_position_ekf and not observer.comparison_ready():
                        print('START_REJECTED: TEST EKF still warming up; no motion requested.',flush=True)
                    else:print('START_ACCEPTED' if session.request_start() else 'START_REJECTED (not ready/already used/fault)',flush=True)
                else:print('Unknown input; no motion requested.',flush=True)
            stamp=time.time_ns()
            session.step(stamp);observer.publish(core)
            if trace is not None:trace.record(core,session.phase,stamp)
            now=time.monotonic()
            if measurement and args.capture_comparison and session.phase in ('DONE','ABORTED','FAULT'):
                if capture_done_at is None:capture_done_at=now
                if now-capture_done_at>=15. and not observer.finished:observer.finish_capture()
            if now-last>=1. or session.phase!=previous:
                last=now;previous=session.phase
                print(json.dumps(dict(phase=session.phase,ready=session.ready(),armed=core.armed,
                    comparison_ekf_ready=observer.comparison_ready() if measurement and args.capture_position_ekf else None,
                    used=session.used,fault=core.fault,feedback_fresh=core.feedback_fresh(),
                    measured_rpm=None if core.feedback is None else [core.feedback['m1_rpm'],core.feedback['m2_rpm']],
                    measured_position=None if core.feedback is None else [core.feedback['m1_position'],core.feedback['m2_position']],
                    position_delta_raw=None if trace is None else trace.delta,
                    odom_x=core.odom_x,odom_y=core.odom_y,valid_feedback=core.valid_count)),flush=True)
            time.sleep(.01)
    except KeyboardInterrupt:print('Exiting: attempting zero RPM.',flush=True)
    finally:
        if core is not None:core.shutdown()
        elif port is not None:port.close()
        if trace_stream is not None:trace_stream.close()
        if observer is not None:observer.close()
    return 0


if __name__=='__main__':raise SystemExit(main())
