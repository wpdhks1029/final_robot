#!/usr/bin/env python3
"""저장 LaserScan의 인접 점군 정합. 실제 TF/ROS/모터 발행 없음. 참값 검증을 대체하지 않음."""
import argparse,sqlite3,math,json
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

def points(m,limit):
    rr=np.asarray(m.ranges,dtype=float);aa=m.angle_min+np.arange(len(rr))*m.angle_increment
    keep=np.isfinite(rr)&(rr>max(.2,m.range_min))&(rr<min(limit,m.range_max))
    p=np.column_stack((rr[keep]*np.cos(aa[keep]),rr[keep]*np.sin(aa[keep])))
    return p[::max(1,math.ceil(len(p)/600))]

def fit(src,dst):
    R=np.eye(2);t=np.zeros(2)
    edge=dst[2:]-dst[:-2];norm=np.linalg.norm(edge,axis=1)
    normals=np.zeros_like(dst);normals[1:-1,0]=-edge[:,1]/np.maximum(norm,1e-9);normals[1:-1,1]=edge[:,0]/np.maximum(norm,1e-9)
    normals[1:-1][(norm<.005)|(norm>.6)]=0
    for _ in range(30):
        p=src@R.T+t;d=((p[:,None,:]-dst[None,:,:])**2).sum(axis=2);idx=d.argmin(axis=1);dist=d[np.arange(len(p)),idx]
        n=normals[idx];res=((p-dst[idx])*n).sum(axis=1)
        keep=(dist<.3**2)&(np.linalg.norm(n,axis=1)>.5)
        if keep.sum()<25:raise ValueError('정합점 부족')
        cutoff=max(.005,float(np.quantile(abs(res[keep]),.8)));keep &= abs(res)<=cutoff
        A=np.column_stack((n[:,0],n[:,1],-n[:,0]*p[:,1]+n[:,1]*p[:,0]))
        step=np.linalg.lstsq(A[keep],-res[keep],rcond=None)[0]
        th=step[2];dr=np.array([[math.cos(th),-math.sin(th)],[math.sin(th),math.cos(th)]])
        R,t=dr@R,dr@t+step[:2]
        if abs(th)<1e-6 and np.linalg.norm(step[:2])<1e-5:break
    p=src@R.T+t;dist=np.sqrt(((p[:,None,:]-dst[None,:,:])**2).sum(axis=2).min(axis=1))
    return math.atan2(R[1,0],R[0,0]),float(np.median(dist)),len(src)

def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('database');a=ap.parse_args()
    db=sqlite3.connect('file:'+a.database+'?mode=ro',uri=True);topics={n:i for i,n in db.execute('select id,name from topics')}
    active=[]
    for ts,b in db.execute('select timestamp,data from messages where topic_id=? order by timestamp',(topics['/cmd_vel'],)):
        m=deserialize_message(b,Twist)
        if abs(m.linear.x)+abs(m.angular.z)>1e-6:active.append(ts)
    groups=[[]]
    for ts in active:
        if groups[-1] and ts-groups[-1][-1]>10**9:groups.append([])
        groups[-1].append(ts)
    first,last=groups[-1][0],groups[-1][-1];scans=[];prev=0
    for ts,b in db.execute('select timestamp,data from messages where topic_id=? and timestamp between ? and ? order by timestamp',(topics['/scan'],first-300000000,last+1500000000)):
        if ts-prev<700000000:continue
        scans.append((ts,deserialize_message(b,LaserScan)));prev=ts
    for limit in (4.,8.):
        angles=[];errors=[]
        for (_,old),(_,new) in zip(scans,scans[1:]):
            th,err,n=fit(points(new,limit),points(old,limit));angles.append(th);errors.append(err)
        print(json.dumps(dict(range_limit_m=limit,scan_pairs=len(angles),relative_yaw_sum_deg=math.degrees(sum(angles)),median_pair_residual_m=float(np.median(errors)),max_pair_residual_m=max(errors),max_pair_turn_deg=math.degrees(max(map(abs,angles))))),flush=True)
    print('점군 정합에 의한 추정이며 지도 위치나 물리 정확도의 검증은 아닙니다.')
if __name__=='__main__':main()
