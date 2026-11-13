"""Pure synchronized heading comparison; no automatic calibration."""
import math


def rpy(q):
    values=(q.x,q.y,q.z,q.w)
    if not all(math.isfinite(v) for v in values) or abs(sum(v*v for v in values)-1)>.02:
        raise ValueError('INVALID_QUATERNION')
    x,y,z,w=values
    return (math.atan2(2*(w*x+y*z),1-2*(x*x+y*y)),
            math.asin(max(-1.,min(1.,2*(w*y-z*x)))),
            math.atan2(2*(w*z+x*y),1-2*(y*y+z*z)))


def angle_delta(a,b):return math.atan2(math.sin(a-b),math.cos(a-b))


def rotation_circle_diagnostic(points):
    if not points or not all(math.isfinite(v) for p in points for v in p):raise ValueError('SCAN_EMPTY_OR_INVALID')
    radius=math.hypot(.27,.30);ordered=sorted(points,key=lambda p:math.hypot(*p))
    required=radius+.02+.30;nearest=[]
    for x,y in ordered[:5]:
        nearest.append(dict(base_x_m=x,base_y_m=y,center_distance_m=math.hypot(x,y),
            laser_range_m=math.hypot(x-.26,y),laser_angle_deg=math.degrees(math.atan2(y,x-.26)),
            footprint_clearance_m=math.hypot(max(abs(x)-.27,0),max(abs(y)-.30,0))))
    return dict(circle_clearance_m=math.hypot(*ordered[0])-radius-.02,
                required_center_distance_m=required,
                points_inside_required_circle=sum(math.hypot(*p)<=required for p in points),nearest_points=nearest)


def synchronized_summary(streams,begin_ns=None,end_ns=None):
    required=('raw','imu')
    if any(len(streams.get(k,[]))<2 for k in required):return dict(reason='NOT_ENOUGH_RAW_IMU_DATA')
    begin=max(streams[k][0]['stamp_ns'] for k in required)
    end=min(streams[k][-1]['stamp_ns'] for k in required)
    if begin_ns is not None:begin=max(begin,begin_ns)
    if end_ns is not None:end=min(end,end_ns)
    if end<=begin:return dict(reason='NO_COMMON_TIME_WINDOW')
    out={}
    for key,rows in streams.items():
        if len(rows)<2:continue
        unwrapped=[rows[0]['yaw']]
        for a,b in zip(rows,rows[1:]):unwrapped.append(unwrapped[-1]+angle_delta(b['yaw'],a['yaw']))
        def at(stamp):
            for i in range(len(rows)-1):
                a,b=rows[i]['stamp_ns'],rows[i+1]['stamp_ns']
                if a<=stamp<=b and 0<b-a<=300_000_000:
                    return unwrapped[i]+(unwrapped[i+1]-unwrapped[i])*(stamp-a)/(b-a)
            return None
        a,b=at(begin),at(end)
        gaps=[(y['stamp_ns']-x['stamp_ns'])/1e9 for x,y in zip(rows,rows[1:])
              if x['stamp_ns']<end and y['stamp_ns']>begin]
        if a is None or b is None or not gaps or max(gaps)>.3:
            out[key]=dict(reason='SOURCE_GAP_OR_WINDOW_MISMATCH');continue
        out[key]=dict(delta_deg=math.degrees(b-a),samples=len(rows),max_gap_s=max(gaps),
                      max_abs_roll_deg=max(abs(math.degrees(r['roll'])) for r in rows),
                      max_abs_pitch_deg=max(abs(math.degrees(r['pitch'])) for r in rows))
    raw=out.get('raw',{}).get('delta_deg');imu=out.get('imu',{}).get('delta_deg')
    valid=raw is not None and imu is not None
    return dict(reason='COMMON_WINDOW_COMPARED' if valid else 'INCOMPLETE_SOURCE_WINDOW',
                begin_ns=begin,end_ns=end,duration_s=(end-begin)/1e9,sources=out,
                imu_minus_raw_deg=imu-raw if valid else None,
                imu_over_raw_ratio=imu/raw if valid and abs(raw)>=10 else None,
                automatic_calibration_applied=False,
                note='IMU and map are comparison sources, not external angle ground truth')


def pivot_request(raw_delta,imu_delta,target,elapsed,clearance,reference='raw'):
    """Pure bounded pivot policy, radians/seconds/metres. No forward command."""
    if not all(math.isfinite(v) for v in (raw_delta,imu_delta,target,elapsed,clearance)):
        return 0.,'STOP_INVALID_INPUT'
    if clearance<=.3:return 0.,'STOP_ROTATION_CIRCLE_CLEARANCE'
    sign=1 if target>0 else -1
    if reference not in ('raw','imu'):return 0.,'STOP_INVALID_ANGLE_SOURCE'
    if abs(imu_delta)>=math.radians(45):return 0.,'STOP_IMU_45_DEG_LIMIT'
    if abs(raw_delta)>=math.radians(40):return 0.,'STOP_RAW_40_DEG_LIMIT'
    if sign*raw_delta<math.radians(-5) or sign*imu_delta<math.radians(-5):return 0.,'STOP_OPPOSITE_ROTATION'
    primary=imu_delta if reference=='imu' else raw_delta
    if sign*primary>=abs(target):return 0.,'IMU_TARGET_REACHED' if reference=='imu' else 'RAW_TARGET_REACHED'
    if elapsed>=12:return 0.,'STOP_12_SECOND_LIMIT'
    speed=.06 if abs(target)-sign*primary<math.radians(3) else .12
    factor=min(1.,max(0.,(clearance-.3)/.15))
    return sign*max(2*.065*2*math.pi/60/.56,speed*factor),'PIVOT_CHECK'
