#!/usr/bin/env python3
"""PID4-only MD200T settings query. No speed/enable/reset/settings-write packets.

Run only on a stopped robot after stopping the existing serial owner.
Missing/unsupported replies remain unknown. No ROS or odom output.
"""
import argparse
import json
from pathlib import Path
import time

from follower_motor_port import ExclusivePort
from follower_motor_candidate import read_request
from md200t_pid216_v2 import decode

# Verified against manufacturer serial specification V6.4h, pp11/19/45/50/59/64/65/84/94/96.
# PID170 is firmware/model dependent; timeout does not mean zero or wrong setting.
FIELDS={1:('version_raw',1),21:('hall_type_m1',1),65:('hall_type_m2',1),
        79:('motor_type',1),156:('encoder_ppr_m1',2),170:('encoder_ppr_m2_model_dependent',2),
        221:('max_rpm_common_or_m1',2),122:('max_rpm_m2',2),185:('communication_watch_delay_raw',2)}


def request(pid):
    if pid not in FIELDS and pid!=216:raise ValueError('PID not on read-only allowlist')
    raw=bytes([0xB7,0xB8,1,4,1,pid])
    return raw+bytes([(-sum(raw))&255])


class ReadOnlyPort(ExclusivePort):
    def write(self,packet):
        if packet not in tuple(request(pid) for pid in [*FIELDS,216]):
            raise ValueError('Only allowlisted ID1 PID4 read requests may be sent')
        return self.device.write(packet)


def extract(buffer):
    frames=[]
    while len(buffer)>=6:
        if buffer[:3]!=b'\xb8\xb7\x01':del buffer[0];continue
        length=buffer[4]
        if not 1<=length<=64:del buffer[0];continue
        size=length+6
        if len(buffer)<size:break
        frame=bytes(buffer[:size])
        if sum(frame)&255:del buffer[0];continue
        del buffer[:size];frames.append(frame)
    return frames


def query(port,pid):
    packet=request(pid)
    if port.write(packet)!=len(packet):raise OSError('Partial serial write')
    until=time.monotonic()+1.;buffer=bytearray()
    while time.monotonic()<until:
        count=port.in_waiting
        if count:
            buffer.extend(port.read(min(count,512)))
            if len(buffer)>4096:raise RuntimeError('RX overflow')
            for frame in extract(buffer):
                if frame[3]==pid:return frame
        time.sleep(.005)
    return None


def value(pid,frame):
    name,length=FIELDS[pid]
    if frame is None:return dict(pid=pid,name=name,result='NO_REPLY_UNKNOWN')
    if frame[4]!=length:return dict(pid=pid,name=name,result='UNEXPECTED_LENGTH_UNKNOWN',rx=frame.hex(' '))
    data=frame[5:-1]
    return dict(pid=pid,name=name,result='OK',raw_unsigned=int.from_bytes(data,'little'),
                raw_signed=int.from_bytes(data,'little',signed=True),rx=frame.hex(' '))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-read-only',action='store_true')
    args=parser.parse_args()
    if not args.run_read_only:parser.print_help();return 0
    import yaml
    p=yaml.safe_load(Path(__file__).with_name('follower_hardware.yaml').read_text())['follower_md200t_pid4']['ros__parameters']
    if p['driver_id']!=1 or p['baudrate']!=19200 or not p['port'].endswith('usb-0:1.4:1.0-port0'):
        raise RuntimeError('Confirmed serial configuration changed')
    port=None;results=[]
    try:
        port=ReadOnlyPort.open(p['port'],19200)
        frame=query(port,216);feedback=decode(frame) if frame else None
        if feedback is None or any(feedback[k]!=0 for k in ('m1_rpm','m2_rpm')):
            raise RuntimeError('Fresh zero RPM not confirmed; query stopped, no stop command sent')
        print('READ ONLY: zero feedback confirmed; PID4 requests only. No settings changed.',flush=True)
        for pid in FIELDS:
            r=value(pid,query(port,pid));r['tx']=request(pid).hex(' ')
            results.append(r);print(json.dumps(r),flush=True)
            frame=query(port,216);feedback=decode(frame) if frame else None
            if feedback is None or any(feedback[k]!=0 for k in ('m1_rpm','m2_rpm')):
                raise RuntimeError('Zero feedback lost; query stopped, no stop command sent')
    finally:
        if port is not None:port.close()
        if results:
            directory=Path(__file__).parent/'logs';directory.mkdir(exist_ok=True)
            path=directory/f'md200t_settings_{time.time_ns()}.json'
            path.write_text(json.dumps(results,indent=2)+'\n')
            print('JSON='+str(path))
    return 0


if __name__=='__main__':raise SystemExit(main())
