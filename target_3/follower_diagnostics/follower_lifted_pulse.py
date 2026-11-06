"""Fixed lifted-wheel trial candidate. Default invocation does not open hardware.

One forward pulse: raw M1=-3, M2=+3 RPM for at most a 0.5s software window.
No ROS subscriptions, automatic following, repeated pulses or automatic rearming.
SIGKILL, process suspension and USB loss can prevent software stopping.
"""
import argparse
import math
from pathlib import Path
import time

from follower_motor_candidate import MotorCore,velocity_packet,read_request
from follower_motor_port import ExclusivePort


class PulsePort(ExclusivePort):
    def __init__(self,device):
        super().__init__(device)
        self.deadline=None
        self.used=False

    def enable_once(self,now):
        if self.used:raise RuntimeError('Only one pulse per process')
        self.used=True;self.deadline=now+.5

    def write(self,packet):
        if packet in (read_request(1),velocity_packet(0,0,1)):
            return self.device.write(packet)
        if (packet==velocity_packet(-3,3,1) and self.deadline is not None
                and time.monotonic()<self.deadline):
            return self.device.write(packet)
        raise ValueError('Pulse packet or time window rejected')


class Trial:
    def __init__(self,core,port):
        self.core=core;self.port=port;self.start=core.clock()
        self.phase='BASELINE';self.samples=0;self.previous=0
        self.stop_at=None;self.zero_samples=0;self.error=None

    def requested_motion(self):
        return 2*math.pi*.065*3/60,0.

    def step(self,ros_ns):
        c=self.core;now=c.clock()
        if self.phase=='PULSE' and now>=self.port.deadline:
            c.disarm();self.phase='VERIFY_STOP';self.stop_at=now
        if self.phase=='PULSE':
            v,w=self.requested_motion()
            if not c.accept_command(v,w,frame='follower/base_link',
                                    stamp_ns=ros_ns,ros_now_ns=ros_ns,epoch=c.epoch):
                self.error='Command rejected'
        c.tick(ros_ns)
        if c.fault or self.error:
            self.error=self.error or c.fault;c.disarm();self.phase='FAILED';return
        fresh=c.valid_count!=self.previous;self.previous=c.valid_count
        zero=fresh and c.feedback['m1_rpm']==0 and c.feedback['m2_rpm']==0
        if self.phase=='BASELINE':
            if fresh and not zero:
                self.error='Nonzero baseline';c.disarm();self.phase='FAILED';return
            if zero:self.samples+=1
            if now-self.start>=1. and self.samples>=5:
                if c.arm(c.epoch,ros_ns) is None:
                    self.error='Arm rejected';self.phase='FAILED';return
                self.port.enable_once(now);self.phase='PULSE'
            elif now-self.start>3.:
                self.error='Baseline timeout';self.phase='FAILED'
        elif self.phase=='VERIFY_STOP':
            if fresh:self.zero_samples=self.zero_samples+1 if zero else 0
            if self.zero_samples>=3:self.phase='DONE'
            elif now-self.stop_at>3.:
                self.error='Zero feedback not confirmed';self.phase='FAILED'


def stationary_observation_tick(core,observer,stamp_ns):
    """After DONE only: retain fresh measured odom without any motion authority."""
    if core.armed:raise RuntimeError('Post-trial observation requires disarmed core')
    core.tick(stamp_ns)
    observer.publish(core)
    if core.fault:raise RuntimeError('Post-trial feedback fault: '+core.fault)
    if core.feedback is not None and any(core.feedback[k]!=0 for k in ('m1_rpm','m2_rpm')):
        core.latch_fault('NONZERO_POST_TRIAL_FEEDBACK')
        raise RuntimeError('Nonzero post-trial RPM; inspect physical stopping')


def main(*,ground=False):
    description=(
        'Ground trial: fixed M1=-3/M2=+3 RPM, 0.5s once, then zero feedback check. '
        'Default opens nothing. No obstacle guard or hardware watchdog; '
        'clear space and immediate power cut required.' if ground else __doc__)
    parser=argparse.ArgumentParser(description=description)
    run_flag='--run-ground-forward-pulse' if ground else '--run-lifted-forward-pulse'
    ack_flag='--ack-clear-1m-power-cut-ready' if ground else '--ack-wheels-raised-power-cut-ready'
    parser.add_argument(run_flag,dest='run_trial',action='store_true')
    parser.add_argument(ack_flag,dest='ack_preparation',action='store_true')
    if ground:
        parser.add_argument('--publish-odom',action='store_true',
                            help='Publish newly measured feedback to /follower/odom_raw; no TF or command subscriptions')
    args=parser.parse_args()
    if not args.run_trial:parser.print_help();return 0
    if not args.ack_preparation:parser.error('Physical preparation acknowledgment required')
    import yaml
    p=yaml.safe_load(Path(__file__).with_name('follower_hardware.yaml').read_text())['follower_md200t_pid4']['ros__parameters']
    if (p['baudrate']!=19200 or p['driver_id']!=1 or p['wheel_radius']!=.065
            or p['wheel_track']!=.560 or p['invert_left'] is not True
            or p['invert_right'] is not False or not p['port'].endswith('usb-0:1.4:1.0-port0')):
        raise RuntimeError('Confirmed hardware configuration changed')
    core=None;port=None;observer=None
    try:
        if ground and args.publish_odom:
            from follower_pulse_odom import Observer
            observer=Observer()
        port=PulsePort.open(p['port'],19200)
        core=MotorCore(port,dry_run=False,allow_motion=True)
        trial=Trial(core,port);previous=None
        while trial.phase not in ('DONE','FAILED'):
            trial.step(time.time_ns())
            if observer is not None:observer.publish(core)
            if trial.phase!=previous:
                print(f'phase={trial.phase}, measured={core.feedback}, error={trial.error}',flush=True)
                previous=trial.phase
            time.sleep(.01)
        print(f'measured_odom_delta_m=({core.odom_x:.6f}, {core.odom_y:.6f}), '
              f'yaw_rad={core.odom_yaw:.6f}, integration_skips={core.integration_skips}, '
              f'valid_feedback={core.valid_count}',flush=True)
        if observer is not None and trial.phase=='DONE':
            print('POST_STOP_OBSERVATION: 15s disarmed, PID4/fresh odom only; do not move robot.',flush=True)
            until=time.monotonic()+15.
            while time.monotonic()<until:
                stationary_observation_tick(core,observer,time.time_ns())
                time.sleep(.01)
            print('POST_STOP_OBSERVATION_DONE: exiting, restore locked driver.',flush=True)
        return 0 if trial.phase=='DONE' else 1
    except KeyboardInterrupt:
        print('Interrupted: attempting zero RPM',flush=True);return 1
    finally:
        if core is not None:core.shutdown()
        elif port is not None:port.close()
        if observer is not None:observer.close()


if __name__=='__main__':raise SystemExit(main())
