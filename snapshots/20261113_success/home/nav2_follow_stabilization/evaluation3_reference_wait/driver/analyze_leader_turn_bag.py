import sqlite3,math,json,bisect
from collections import defaultdict
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import argparse
parser=argparse.ArgumentParser(description='저장된 rosbag의 마지막 회전 명령 구간을 읽기 전용 분석')
parser.add_argument('database')
p=parser.parse_args().database
c=sqlite3.connect('file:'+p+'?mode=ro',uri=True)
top={i:(n,get_message(t)) for i,n,t in c.execute('select id,name,type from topics')}
r=defaultdict(list)
def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
for tid,t,b in c.execute('select topic_id,timestamp,data from messages order by timestamp'):
 n,k=top[tid]
 if n in ('/scan','/tf_static','/tf'):continue
 m=deserialize_message(b,k);t=t/1e9
 if n=='/cmd_vel':r[n].append((t,m.linear.x,m.angular.z))
 elif n=='/imu/data':r[n].append((t,yaw(m.orientation),m.angular_velocity.z))
 elif n in ('/odom','/odometry/filtered'):r[n].append((t,yaw(m.pose.pose.orientation),m.twist.twist.angular.z,m.pose.pose.position.x,m.pose.pose.position.y))
 elif n=='/amcl_pose':r[n].append((t,yaw(m.pose.pose.orientation),m.pose.pose.position.x,m.pose.pose.position.y))
 elif n=='/tf':
  for tr in m.transforms:
   if (tr.header.frame_id,tr.child_frame_id)==('map','odom'):r['map_odom'].append((t,yaw(tr.transform.rotation),tr.transform.translation.x,tr.transform.translation.y))
active=[x for x in r['/cmd_vel'] if abs(x[1])+abs(x[2])>1e-6];groups=[[]]
for x in active:
 if groups[-1] and x[0]-groups[-1][-1][0]>1.:groups.append([])
 groups[-1].append(x)
print('COMMAND_SEGMENTS',[(g[0][0],g[-1][0],len(g)) for g in groups])
active=groups[-1];start,end=active[0][0],active[-1][0];print('WINDOW',start,end,end-start,'COMMANDS',len(active), 'W',sorted(set(x[2] for x in active)))
def delta(a,b):return math.degrees(math.atan2(math.sin(b-a),math.cos(b-a)))
for n,rows in r.items():
 if n=='/cmd_vel':continue
 before=[x for x in rows if start-2<=x[0]<start];after=[x for x in rows if end+2<x[0]<=end+4]
 if not before:before=[x for x in rows if x[0]<start][-1:]
 if not after:after=[x for x in rows if x[0]>end][:1]
 if before and after:
  a,b=before[-1],after[-1];print(n,'before',a,'after',b,'DELTA_DEG',delta(a[1],b[1]))
 if n in ('/imu/data','/odom','/odometry/filtered'):
  rr=[x for x in rows if start<=x[0]<=end+2];integ=sum((b[0]-a[0])*(a[2]+b[2])/2 for a,b in zip(rr,rr[1:]));print(' INTEGRATED_W_DEG',math.degrees(integ),'max_gap',max(b[0]-a[0] for a,b in zip(rr,rr[1:])))
print('AMCL',[(round(x[0]-start,3),round(math.degrees(x[1]),3),*x[2:]) for x in r['/amcl_pose']])

rr=[x for x in r['/imu/data'] if start<=x[0]<=end]
print('IMU_MOTION',len(rr),'zero_w',sum(x[2]==0 for x in rr),'w_range',min(x[2] for x in rr),max(x[2] for x in rr))
for off in (5,15,30):
 near=[x for x in r['/imu/data'] if end+off-1<=x[0]<=end+off+1]
 if near:print('IMU_POST',off,math.degrees(near[-1][1]))
