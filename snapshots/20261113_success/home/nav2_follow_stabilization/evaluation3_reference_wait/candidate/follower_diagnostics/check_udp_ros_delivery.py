#!/usr/bin/env python3
"""명시한 상대와 작은 UDP 패킷을 교환하며 ROS 시각만 기록. 모터·서비스 접근 없음."""
import argparse
from collections import defaultdict
import ipaddress
import json
from pathlib import Path
import secrets
import socket
import struct
import time

WIRE=struct.Struct('!4sBBQQQQ')
MAGIC=b'FDG1'


def encode(kind,session,seq,mono,wall):
    if kind not in (0,1):raise ValueError('잘못된 패킷 종류')
    return WIRE.pack(MAGIC,1,kind,session,seq,mono,wall)


def decode(data):
    if len(data)!=WIRE.size:raise ValueError('패킷 길이 불일치')
    magic,version,kind,session,seq,mono,wall=WIRE.unpack(data)
    if magic!=MAGIC or version!=1 or kind not in (0,1):raise ValueError('진단 패킷 아님')
    return kind,session,seq,mono,wall


def summary(rows,start_ns,warmup_s):
    groups=defaultdict(list)
    for r in rows:
        if (r['rx_mono_ns']-start_ns)/1e9>=warmup_s:groups[r['name']].append(r)
    out={}
    for name,a in groups.items():
        gaps=[(y['rx_mono_ns']-x['rx_mono_ns'])/1e9 for x,y in zip(a,a[1:])]
        rtt=sorted(x['rtt_s'] for x in a if 'rtt_s' in x)
        entry=dict(received=len(a),max_gap_s=max(gaps,default=None),gaps_over_0_2s=sum(g>.2 for g in gaps))
        if rtt:entry.update(rtt_p95_s=rtt[int((len(rtt)-1)*.95)],rtt_max_s=rtt[-1])
        if name=='udp_peer_probe':
            sessions=defaultdict(set)
            for x in a:sessions[x['session']].add(x['seq'])
            entry['unseen_sequences_within_observed_span']=sum(max(v)-min(v)+1-len(v) for v in sessions.values())
        out[name]=entry
    correlations=[]
    for name in ('leader','path'):
        a=groups.get(name,[])
        for x,y in zip(a,a[1:]):
            lo,hi=x['rx_mono_ns'],y['rx_mono_ns']
            if hi-lo<=200_000_000:continue
            def overlapping_gap(other):
                b=groups.get(other,[])
                return max(((v['rx_mono_ns']-u['rx_mono_ns'])/1e9 for u,v in zip(b,b[1:])
                            if u['rx_mono_ns']<hi and v['rx_mono_ns']>lo),default=None)
            echoes=[r['rtt_s'] for r in groups.get('udp_echo',[]) if lo<=r['rx_mono_ns']<=hi+100_000_000]
            correlations.append(dict(topic=name,before_rx_wall_ns=x['rx_wall_ns'],after_rx_wall_ns=y['rx_wall_ns'],
                ros_gap_s=(hi-lo)/1e9,overlapping_udp_gap_s=overlapping_gap('udp_peer_probe'),
                overlapping_observer_gap_s=overlapping_gap('observer_tick'),nearby_max_rtt_s=max(echoes,default=None)))
    return dict(excluded_first_seconds=warmup_s,streams=out,ros_long_gap_correlations=correlations)


def main():
    import os
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--role',choices=('leader','follower'),required=True)
    p.add_argument('--peer',required=True)
    p.add_argument('--seconds',type=int,default=120)
    p.add_argument('--port',type=int,default=42017)
    a=p.parse_args()
    try:peer=str(ipaddress.IPv4Address(a.peer))
    except ValueError:p.error('peer는 상대 IPv4 주소여야 합니다.')
    if not 30<=a.seconds<=180:p.error('seconds는 30~180초입니다.')
    if not 1024<=a.port<=65535:p.error('port는 1024~65535입니다.')
    if os.environ.get('ROS_DOMAIN_ID')!='10':p.error('ROS_DOMAIN_ID=10이 필요합니다.')
    import rclpy
    from nav_msgs.msg import Odometry,Path as RosPath
    from rclpy.qos import QoSProfile,ReliabilityPolicy,HistoryPolicy,DurabilityPolicy
    sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0',a.port));sock.setblocking(False)
    rclpy.init(args=[]);node=rclpy.create_node('udp_ros_delivery_'+a.role)
    qos=QoSProfile(depth=1,history=HistoryPolicy.KEEP_LAST,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE)
    rows=[];subs=[];session=secrets.randbits(64);seq=0;tx={};errors=[];bad=0
    def record(name,source_ns=0,**extra):
        rows.append(dict(name=name,source_ns=source_ns,rx_mono_ns=time.monotonic_ns(),rx_wall_ns=time.time_ns(),**extra))
    def ros_cb(name,m):record(name,m.header.stamp.sec*10**9+m.header.stamp.nanosec)
    topics=[('leader','/leader/odom',Odometry),('path','/leader/path',RosPath)]
    topics += [('filtered','/odometry/filtered',Odometry)] if a.role=='leader' else [('follower','/follower/odom_map',Odometry)]
    for name,topic,kind in topics:subs.append(node.create_subscription(kind,topic,lambda m,k=name:ros_cb(k,m),qos))
    start=time.monotonic_ns();next_send=start
    print(f'통신 진단 {a.seconds}초: {a.role} ↔ {peer}:{a.port}, UDP {WIRE.size}바이트/20Hz. 이동 명령 없음.',flush=True)
    try:
        while rclpy.ok() and time.monotonic_ns()-start<a.seconds*1_000_000_000:
            now=time.monotonic_ns()
            if now>=next_send:
                record('observer_tick');seq+=1;wall=time.time_ns();tx[seq]=now
                try:sock.sendto(encode(0,session,seq,now,wall),(peer,a.port))
                except OSError as exc:
                    if len(errors)<50:errors.append(str(exc))
                next_send=now+50_000_000
            for _ in range(100):
                try:data,addr=sock.recvfrom(256)
                except BlockingIOError:break
                if addr!=(peer,a.port):bad+=1;continue
                try:kind,sid,num,sent,wall=decode(data)
                except ValueError:bad+=1;continue
                if kind==0:
                    record('udp_peer_probe',wall,session=sid,seq=num)
                    try:sock.sendto(encode(1,sid,num,sent,wall),addr)
                    except OSError as exc:
                        if len(errors)<50:errors.append(str(exc))
                elif sid==session and tx.get(num)==sent:
                    record('udp_echo',wall,seq=num,rtt_s=(time.monotonic_ns()-sent)/1e9)
                    del tx[num]
            rclpy.spin_once(node,timeout_sec=.005)
    except KeyboardInterrupt:pass
    finally:
        sock.close()
        report=summary(rows,start,20.)
        result=dict(role=a.role,peer=peer,port=a.port,session=session,elapsed_s=(time.monotonic_ns()-start)/1e9,
                    packet_bytes=WIRE.size,summary=report,send_errors=errors,invalid_or_unexpected_packets=bad,rows=rows,
                    limitation='UDP와ROS는서로다른전송경로일수있다. 관측공백과미관측순번만으로특정네트워크원인이나손실을확정하지않는다.')
        directory=Path.home()/('leader_diagnostics' if a.role=='leader' else 'follower_diagnostics')/'logs'
        directory.mkdir(exist_ok=True,parents=True);path=directory/f'udp_ros_{a.role}_{time.time_ns()}.json'
        path.write_text(json.dumps(result,ensure_ascii=False,allow_nan=False)+'\n')
        print(json.dumps(report,ensure_ascii=False,indent=2));print('JSON='+str(path),flush=True)
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
