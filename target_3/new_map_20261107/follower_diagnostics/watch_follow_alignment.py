#!/usr/bin/env python3
"""Read-only, measured-time map/filtered alignment audit. No motion interfaces."""
import argparse,json,math,os,time
from pathlib import Path
from local_stop_turn_go import pose_at
from watch_follower_yaw import attach
from follower_yaw_diagnostics import synchronized_summary

def alignment_sample(map_row,history):
    stamp=map_row['stamp_ns'];p,support,uncertainty=pose_at(history,stamp,max_v=.09,max_w=.33)
    if not all(math.isfinite(v) for v in (map_row['x'],map_row['y'],map_row['yaw'],p.x,p.y,p.yaw)):raise ValueError('NONFINITE_POSE')
    yaw=math.atan2(math.sin(map_row['yaw']-p.yaw),math.cos(map_row['yaw']-p.yaw))
    c,s=math.cos(yaw),math.sin(yaw)
    return dict(stamp_ns=stamp,pose_support_ns=support,hold_uncertainty_m=uncertainty,
                transform=[map_row['x']-c*p.x+s*p.y,map_row['y']-s*p.x-c*p.y,yaw])

def summarize(rows):
    valid=[r for r in rows if 'transform' in r]
    if len(valid)<2:return dict(reason='NOT_ENOUGH_MATCHED_SAMPLES',matched_samples=len(valid))
    base=valid[0]['transform'];shifts=[];angles=[]
    for r in valid:
        t=r['transform'];shifts.append(math.dist(t[:2],base[:2]))
        angles.append(math.degrees(math.atan2(math.sin(t[2]-base[2]),math.cos(t[2]-base[2]))))
    translation=max(shifts);span=max(angles)-min(angles)
    return dict(reason='OBSERVED_MAP_ALIGNMENT_VARIATION' if translation>.05 or span>3. else 'OBSERVED_STATIONARY_ALIGNMENT_STABLE',
                matched_samples=len(valid),max_transform_translation_change_m=translation,
                transform_yaw_span_deg=span,duration_s=(valid[-1]['stamp_ns']-valid[0]['stamp_ns'])/1e9,
                note='Diagnostic observation only; no calibration or physical accuracy verdict')

def common_yaw_comparison(streams,begin=None,end=None):
    keys=('raw','filtered','imu','map')
    if any(len(streams.get(k,[]))<2 for k in keys):return dict(reason='NOT_ENOUGH_COMMON_SOURCES')
    first=max(streams[k][0]['stamp_ns'] for k in keys)
    last=min(streams[k][-1]['stamp_ns'] for k in keys)
    return synchronized_summary(streams,max(first,begin) if begin is not None else first,min(last,end) if end is not None else last)

def phase_comparisons(rows,streams):
    groups=[];current=[];key=None
    for row in rows:
        label=(row.get('trial_phase'),row.get('avoidance_stage'))
        valid='transform' in row and row.get('motor_armed') is True and label[0] is not None
        if not valid or label!=key:
            if len(current)>=2:groups.append((key,current))
            current=[];key=label if valid else None
        if valid:current.append(row)
    if len(current)>=2:groups.append((key,current))
    results=[]
    for (phase,stage),part in groups:
        summary=summarize(part)
        results.append(dict(trial_phase=phase,avoidance_stage=stage,
            begin_ns=part[0]['stamp_ns'],end_ns=part[-1]['stamp_ns'],matched_samples=len(part),
            transform_yaw_span_deg=summary.get('transform_yaw_span_deg'),
            max_transform_translation_change_m=summary.get('max_transform_translation_change_m'),
            yaw_comparison=common_yaw_comparison(streams,part[0]['stamp_ns'],part[-1]['stamp_ns'])))
    return results

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--seconds',type=int,default=20);parser.add_argument('--observe-motion',action='store_true');a=parser.parse_args()
    if not 10<=a.seconds<=400:parser.error('seconds must be 10..400')
    if os.environ.get('ROS_DOMAIN_ID')!='10':parser.error('ROS_DOMAIN_ID=10 required')
    import rclpy
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from follower_remote_qos import latest_state_qos
    rclpy.init(args=[]);node=rclpy.create_node('follow_alignment_readonly');watch=attach(node);leader={};rows=[];last_stamp=None;last_print=0.;phase_now={};phase_events=[]
    def on_leader(msg):
        leader.update(stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,rx=time.monotonic(),v=msg.twist.twist.linear.x,w=msg.twist.twist.angular.z)
    sub=node.create_subscription(Odometry,'/leader/odom',on_leader,latest_state_qos())
    def on_control(msg):
        try:
            control=json.loads(msg.data);phase=control.get('trial_phase')
            if phase not in {'FOLLOW','AVOID','PLAN_BRAKE','PLANNING','REJOIN_BRAKE','OBSTACLE_WAIT'}:return
            stage=control.get('apf',{}).get('stage_name') if phase=='AVOID' else None
            phase_now.update(trial_phase=phase,avoidance_stage=stage,rx=time.monotonic())
            if not phase_events or (phase,stage)!=(phase_events[-1]['trial_phase'],phase_events[-1]['avoidance_stage']):
                phase_events.append(dict(trial_phase=phase,avoidance_stage=stage,rx_ros_ns=node.get_clock().now().nanoseconds,scan_stamp_ns=control.get('scan_stamp_ns')))
        except (ValueError,TypeError,AttributeError):return
    control_sub=node.create_subscription(String,'/follower/local_avoidance/status',on_control,latest_state_qos())
    mode='별도 시험의 이동을 관찰합니다. 이 도구는 주행을 시작하지 않습니다.' if a.observe_motion else '양쪽 정지·후발 비무장 유지.'
    print(f'조회 전용 {a.seconds}초: {mode} 초기 pose를 바꾸지 마세요. 명령·arm·서비스 호출 없음.',flush=True)
    begin=time.monotonic()
    try:
        while rclpy.ok() and time.monotonic()-begin<a.seconds:
            rclpy.spin_once(node,timeout_sec=.05);mono=time.monotonic();now=node.get_clock().now().nanoseconds
            row=dict(ros_ns=now)
            try:
                if watch.motor is None or mono-watch.motor[1]>.3:raise ValueError('MOTOR_STATUS_MISSING_OR_STALE')
                motor=watch.motor[0]
                if motor.get('feedback_fresh') is not True or motor.get('fault'):raise ValueError('MOTOR_FAULT_OR_FEEDBACK_STALE')
                if not a.observe_motion and (motor.get('armed') is not False or motor.get('measured_rpm')!=[0,0]):raise ValueError('KEEP_FOLLOWER_DISARMED_AND_STOPPED')
                if not leader or not 0<=(now-leader['stamp']+50_000_000)/1e9<=.8 or mono-leader['rx']>.8:raise ValueError('LEADER_MISSING_OR_STALE')
                if not a.observe_motion and (abs(leader['v'])>.01 or abs(leader['w'])>.02):raise ValueError('KEEP_LEADER_STOPPED')
                for key in ('map','filtered'):
                    if key not in watch.latest:raise ValueError(key.upper()+'_MISSING')
                    item=watch.latest[key]
                    if not 0<=(now-item['stamp_ns'])/1e9<=.8 or mono-item['rx_mono']>.8:raise ValueError(key.upper()+'_STALE')
                filtered=watch.latest['filtered']
                if not a.observe_motion and (abs(filtered['v'])>.01 or abs(filtered['w'])>.02):raise ValueError('FILTERED_MOTION_NOT_ZERO')
                m=watch.latest['map']
                if m['stamp_ns']==last_stamp:continue
                history=[(x['stamp_ns'],x['rx_mono'],(x['x'],x['y'],x['yaw'],x['v'],x['w'])) for x in watch.streams['filtered'][-100:]]
                row.update(alignment_sample(m,history),reason='MATCHED_MOTION_OBSERVATION' if a.observe_motion else 'MATCHED_STATIONARY_POSES',
                           motor_armed=motor.get('armed'),measured_rpm=motor.get('measured_rpm'),filtered_pose=[filtered['x'],filtered['y'],filtered['yaw']],
                           filtered_twist=[filtered['v'],filtered['w']],map_pose=[m['x'],m['y'],m['yaw']]);last_stamp=m['stamp_ns']
            except ValueError as exc:row['reason']=str(exc)
            if phase_now and 0<=mono-phase_now['rx']<=.5:
                row.update(trial_phase=phase_now['trial_phase'],avoidance_stage=phase_now['avoidance_stage'])
            rows.append(row)
            if mono-last_print>=2.:
                print(json.dumps(row),flush=True);last_print=mono
    except KeyboardInterrupt:pass
    finally:
        summary=dict(summarize(rows),control_connected=False,evaluation_pass=False,rejections=watch.rejections,observe_motion=a.observe_motion)
        if a.observe_motion:
            motion_samples=sum(r.get('motor_armed') is True and (any(abs(v)>.001 for v in r.get('filtered_twist',[])) or any(v!=0 for v in r.get('measured_rpm',[]))) for r in rows)
            summary['motion_sample_count']=motion_samples
            summary['reason']='MOTION_ALIGNMENT_OBSERVATION_SAVED' if motion_samples else 'NO_MOTION_OBSERVED'
            summary['yaw_comparison']=common_yaw_comparison(watch.streams)
            moving=[r for r in rows if r.get('motor_armed') is True and any(abs(v)>.001 for v in r.get('filtered_twist',[])) and 'stamp_ns' in r]
            if moving:summary['motion_yaw_comparison']=common_yaw_comparison(watch.streams,moving[0]['stamp_ns'],moving[-1]['stamp_ns'])
            summary['phase_comparisons']=phase_comparisons(rows,watch.streams)
            summary['phase_note']='Phase labels are recent controller status context, not exact physical transition timestamps'
        try:
            folder=Path(__file__).resolve().parent/'logs';folder.mkdir(exist_ok=True);out=folder/f'follow_alignment_{time.time_ns()}.json'
            out.write_text(json.dumps(dict(summary=summary,observations=rows,streams=watch.streams,phase_events=phase_events),allow_nan=False))
            print(json.dumps(summary,ensure_ascii=False,indent=2));print('JSON='+str(out))
        finally:
            node.destroy_node()
            if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
