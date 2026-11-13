#!/usr/bin/env python3
"""ONE lifted-wheel turn-direction trial: raw M1=+3/M2=+3 RPM for 0.5s.

Expected physical left backward/right forward, equivalent to positive yaw.
No ground rotation, ROS inputs/outputs, torque-off, brake release or settings writes.
Default opens nothing. USB loss/SIGKILL/process suspension can prevent stopping.
"""
import argparse
import json
import math
from pathlib import Path
import time

from follower_lifted_pulse import PulsePort,Trial
from follower_motor_candidate import MotorCore,read_request,velocity_packet
from follower_feedback_trace import FeedbackTrace


class TurnPort(PulsePort):
    def write(self,packet):
        if packet in (read_request(1),velocity_packet(0,0,1)):
            return self.device.write(packet)
        if (packet==velocity_packet(3,3,1) and self.deadline is not None
                and time.monotonic()<self.deadline):
            return self.device.write(packet)
        raise ValueError('Only zero/PID4 or fixed +3/+3 within one 0.5s window allowed')


class TurnTrial(Trial):
    def requested_motion(self):
        return 0.,2*(2*math.pi*.065*3/60)/.560


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run-lifted-turn',action='store_true')
    p.add_argument('--ack-wheels-raised-power-cut-ready',action='store_true')
    a=p.parse_args()
    if not a.run_lifted_turn:p.print_help();return 0
    if not a.ack_wheels_raised_power_cut_ready:p.error('Lifted wheels and power-cut preparation acknowledgment required')
    import yaml
    cfg=yaml.safe_load(Path(__file__).with_name('follower_hardware.yaml').read_text())['follower_md200t_pid4']['ros__parameters']
    if (cfg['driver_id']!=1 or cfg['baudrate']!=19200 or cfg['wheel_radius']!=.065
            or cfg['wheel_track']!=.560 or cfg['invert_left'] is not True or cfg['invert_right'] is not False
            or not cfg['port'].endswith('usb-0:1.4:1.0-port0')):
        raise RuntimeError('Confirmed hardware configuration changed')
    port=core=stream=None
    try:
        directory=Path(__file__).parent/'logs';directory.mkdir(exist_ok=True)
        path=directory/f'lifted_turn_{time.time_ns()}.jsonl';stream=path.open('x')
        trace=FeedbackTrace(stream);print('TURN_TRACE='+str(path),flush=True)
        port=TurnPort.open(cfg['port'],19200)
        core=MotorCore(port,dry_run=False,allow_motion=True)
        trial=TurnTrial(core,port);previous=None
        while trial.phase not in ('DONE','FAILED'):
            stamp=time.time_ns();trial.step(stamp);trace.record(core,trial.phase,stamp)
            if previous!=trial.phase:
                previous=trial.phase
                print(json.dumps(dict(phase=trial.phase,measured=core.feedback,error=trial.error)),flush=True)
            time.sleep(.01)
        return 0 if trial.phase=='DONE' else 1
    except KeyboardInterrupt:
        print('Interrupted; attempting zero RPM.',flush=True);return 1
    finally:
        if core is not None:core.shutdown()
        elif port is not None:port.close()
        if stream is not None:stream.close()


if __name__=='__main__':raise SystemExit(main())
