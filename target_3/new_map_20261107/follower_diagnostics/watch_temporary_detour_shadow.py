#!/usr/bin/env python3
"""임시 경로의 실제 입력 관찰. ROS 명령 발행/arm/모터 서비스 접근 없음."""
import argparse,json,math,os,time
from pathlib import Path
import numpy as np
from temporary_detour_candidate import Pose,pursue
from apf_case3_candidate import wrap,clearance,Settings
from replay_recorded_detour import coordinated_speed


class PreparedPathReview:
    """Prepare path geometry once; conservatively skip distant point arithmetic."""
    def __init__(self,path):
        self.poses=np.asarray(path,dtype=float)
        p=self.poses
        if p.ndim!=2 or p.shape[1]!=4 or len(p)==0 or not np.isfinite(p).all() or np.any(abs(p[:,3])>.98*2/.560):
            raise ValueError('STOP_PATH_OR_SCAN_INVALID')
        self.co=np.cos(p[:,2,None]);self.si=np.sin(p[:,2,None]);self.movement=np.zeros(len(p))
        if len(p)>1:
            angles=np.diff(p[:,2]);angles=np.arctan2(np.sin(angles),np.cos(angles))
            self.movement[1:]=np.hypot(np.diff(p[:,0]),np.diff(p[:,1]))+math.hypot(.27,.3)*abs(angles)
        self.blocks=[];body_radius=math.hypot(.27,.3)
        for begin in range(0,len(p),64):
            end=min(len(p),begin+64);part=p[begin:end]
            bound=.30+.02+float(self.movement[begin:end].max())+.001
            radius=body_radius+bound
            bounds=(part[:,0].min()-radius,part[:,0].max()+radius,part[:,1].min()-radius,part[:,1].max()+radius)
            self.blocks.append((begin,end,bounds,bound))

    def review(self,points,start_index=0):
        if not points:return dict(reason='STOP_PATH_OR_SCAN_MISSING')
        q=np.asarray(points,dtype=float)
        if q.ndim!=2 or q.shape[1]!=2 or not np.isfinite(q).all() or not 0<=start_index<len(self.poses):
            return dict(reason='STOP_PATH_OR_SCAN_INVALID')
        minimum=float('inf');blocked=None;pruned=False
        for begin,end,bounds,bound in self.blocks:
            if end<=start_index:continue
            begin=max(begin,start_index);part=self.poses[begin:end]
            near=(q[:,0]>=bounds[0])&(q[:,0]<=bounds[1])&(q[:,1]>=bounds[2])&(q[:,1]<=bounds[3])
            # An excluded point is farther than the entire bounding circle plus
            # the protection threshold. It cannot violate any pose in the block.
            if not near.all():minimum=min(minimum,bound);pruned=True
            local=q[near]
            if not len(local):continue
            dx=local[:,0][None,:]-part[:,0,None];dy=local[:,1][None,:]-part[:,1,None]
            co,si=self.co[begin:end],self.si[begin:end]
            distances=np.hypot(np.maximum(abs(co*dx+si*dy)-.27,0),np.maximum(abs(-si*dx+co*dy)-.30,0))
            nearest=distances.min(axis=1);minimum=min(minimum,float(nearest.min()))
            failures=np.flatnonzero(nearest<=.30+.02+self.movement[begin:end])
            if len(failures) and blocked is None:
                i=int(failures[0]);point=local[int(distances[i].argmin())]
                blocked=dict(path_index=begin+i,path_pose=part[i,:3].tolist(),point=point.tolist(),clearance_m=float(nearest[i]))
        return dict(reason='STOP_LIVE_PATH_BLOCKED' if blocked else 'LIVE_PATH_GEOMETRY_CLEAR',
                    min_path_clearance_m=minimum,clearance_is_lower_bound=pruned,blocking=blocked)


def review_path(path,points):
    if not path or not points:return dict(reason='STOP_PATH_OR_SCAN_MISSING')
    try:return PreparedPathReview(path).review(points)
    except (ValueError,TypeError):return dict(reason='STOP_PATH_OR_SCAN_INVALID')


def finalize_row(row,after_ns,compute_s):
    row['compute_s']=compute_s
    age=(after_ns-row['source_ns'])/1e9
    row['source_age_after_compute_s']=age
    stale=not 0<=age<=.3
    row['timing_reason']='STOP_INPUT_AGED_DURING_REVIEW' if stale else 'INPUT_TIME_VALID'
    row['geometry_reason']=row['reason']
    if stale and row['reason']=='LIVE_PATH_GEOMETRY_CLEAR':row['reason']='STOP_INPUT_AGED_DURING_REVIEW'
    if stale or row['geometry_reason']!='LIVE_PATH_GEOMETRY_CLEAR' or row.get('tracking_reason','STOP').startswith('STOP'):
        row.update(guarded_candidate_v=0.,guarded_candidate_w=0.,leader_candidate_v=0.)
    return row


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds',type=int,default=20)
    parser.add_argument('--plan',type=Path,default=Path(__file__).parent/'logs/recorded_left_detour_time_plan.json')
    parser.add_argument('--virtual-forward-probe',action='store_true')
    a=parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID')!='10':parser.error('ROS_DOMAIN_ID=10 required')
    if not 5<=a.seconds<=60:parser.error('seconds must be 5..60')
    data=json.loads(a.plan.read_text());snapshot=data['snapshot'];origin=snapshot['reference_origin'];heading=snapshot['reference_heading']
    geometry_settings=Settings()
    if data.get('reason')!='DETOUR_PLANNED' or not data.get('path'):parser.error('Checked path required')
    reviewer=PreparedPathReview(data['path'])
    import rclpy
    from rclpy.time import Time
    from rclpy.duration import Duration
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import Odometry,Path as RosPath
    from tf2_ros import Buffer,TransformListener,TransformException
    from follower_remote_qos import latest_state_qos
    from follower_pursuit_preview import PreviewCore
    from watch_apf_case3 import project
    from temporary_detour_candidate import to_reference
    rclpy.init(args=[]);node=rclpy.create_node('temporary_detour_shadow_readonly')
    buffer=Buffer();listener=TransformListener(buffer,node,qos=latest_state_qos());nav=PreviewCore();scan=None
    def scan_cb(m):
        nonlocal scan
        scan=(m,time.monotonic())
    subs=[node.create_subscription(LaserScan,'/follower/scan',scan_cb,latest_state_qos()),
          node.create_subscription(Odometry,'/follower/odom_map',lambda m:nav.odom_callback('follower',m,node.get_clock().now().nanoseconds,time.monotonic()),latest_state_qos()),
          node.create_subscription(Odometry,'/leader/odom',lambda m:nav.odom_callback('leader',m,node.get_clock().now().nanoseconds,time.monotonic()),latest_state_qos()),
          node.create_subscription(RosPath,'/leader/path',lambda m:nav.path_callback(m,node.get_clock().now().nanoseconds,time.monotonic()),latest_state_qos())]
    rows=[];scenes=[];scene_key=None;last=0.;index=0;start=time.monotonic()
    print('관찰 전용: 실제 scan/pose/path/leader로 경로·가상 명령 계산. 모터 발행·arm·서비스 호출 없음.',flush=True)
    try:
        while rclpy.ok() and time.monotonic()-start<a.seconds:
            rclpy.spin_once(node,timeout_sec=.05)
            if time.monotonic()-last<1.:continue
            last=time.monotonic();now=node.get_clock().now().nanoseconds
            compute_started=time.monotonic()
            row=dict(ros_ns=now,control_connected=False,virtual_probe=a.virtual_forward_probe,
                     guarded_candidate_v=0.,guarded_candidate_w=0.,leader_candidate_v=0.)
            try:
                stamps=[]
                for key in ('leader','path','follower'):
                    item=nav.inputs.get(key)
                    if item is None:raise ValueError('STOP_'+key.upper()+'_MISSING')
                    stamp=item[0].sec*10**9+item[0].nanosec-(50_000_000 if key in ('leader','path') else 0)
                    if not (stamp>0 and 0<=(now-stamp)/1e9<=.3 and 0<=last-item[1]<=.3):raise ValueError('STOP_'+key.upper()+'_STALE')
                    stamps.append(stamp)
                if scan is None:raise ValueError('STOP_SCAN_MISSING')
                m,rx=scan;stamp=m.header.stamp.sec*10**9+m.header.stamp.nanosec
                if not (stamp>0 and 0<=(now-stamp)/1e9<=.3 and 0<=last-rx<=.3):raise ValueError('STOP_SCAN_STALE')
                stamps.append(stamp)
                transform=buffer.lookup_transform('follower/base_link','follower/laser',Time.from_msg(m.header.stamp),timeout=Duration(seconds=0.))
                pose=Pose(*nav.inputs['follower'][2][:3]);co,si=math.cos(heading),math.sin(heading)
                dx,dy=pose.x-origin[0],pose.y-origin[1]
                local=Pose(co*dx+si*dy,-si*dx+co*dy,wrap(pose.yaw-heading))
                if math.hypot(local.x-data['path'][index][0],local.y-data['path'][index][1])>.15:raise ValueError('STOP_PLAN_POSE_MISMATCH')
                body_points=project(m,transform)
                points=to_reference(body_points,pose,origin,heading)
                geometry=reviewer.review(points,index);row.update(geometry)
                block=geometry.get('blocking');point=block['point'] if block else (0.,0.)
                key=(geometry['reason'],math.floor(point[0]/.2),math.floor(point[1]/.2))
                if key!=scene_key:
                    scenes.append(dict(ros_ns=now,scan_source_ns=stamp,pose=vars(pose),points=points,
                                       body_points=body_points,geometry=geometry))
                    scenes=scenes[-30:];scene_key=key
                speed=.06 if a.virtual_forward_probe or nav.leader_moving else 0.
                v,w,index,tracking=pursue(data['path'],local,speed,index,.15)
                current_clearance=min(clearance(x,y,geometry_settings) for x,y in body_points)
                factor=min(1.,max(0.,(current_clearance-.3)/.3))
                permitted=geometry['reason']=='LIVE_PATH_GEOMETRY_CLEAR' and not tracking.startswith('STOP')
                guarded_v=v*factor if permitted else 0.;guarded_w=w*factor if permitted else 0.
                leader=nav.inputs['leader'][2]
                leader_x=co*(leader[0]-origin[0])+si*(leader[1]-origin[1])
                start_yaw=data['path'][0][2]
                target_gap=.27*abs(math.cos(start_yaw))+.3*abs(math.sin(start_yaw))+1.+.27
                leader_v=coordinated_speed(leader_x,local.x,local.yaw,guarded_v,target_gap,enabled=guarded_v>0)
                row.update(source_ns=min(stamps),source_age_s=(now-min(stamps))/1e9,
                           tracking_reason=tracking,candidate_v=v,candidate_w=w,path_index=index,
                           current_clearance_m=current_clearance,guarded_candidate_v=guarded_v,guarded_candidate_w=guarded_w,
                           leader_candidate_v=leader_v,
                           upstream_reason='VIRTUAL_FORWARD_PROBE' if a.virtual_forward_probe else ('LEADER_MOVING' if nav.leader_moving else 'STOP_LEADER_STOPPED'))
                # 계산 중 오래된 입력은 유효하다고 표시하지 않는다.
                after=node.get_clock().now().nanoseconds
                finalize_row(row,after,time.monotonic()-compute_started)
            except (ValueError,TransformException) as exc:row['reason']=str(exc)
            rows.append(row);print(json.dumps(row,allow_nan=False,ensure_ascii=False),flush=True)
    except KeyboardInterrupt:pass
    finally:
        folder=Path(__file__).parent/'logs';folder.mkdir(exist_ok=True);dest=folder/f'detour_shadow_{time.time_ns()}.json'
        dest.write_text(json.dumps(dict(plan=str(a.plan),rows=rows,scenes=scenes,control_connected=False,evaluation_pass=False),allow_nan=False)+'\n')
        print('JSON='+str(dest),flush=True);node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
