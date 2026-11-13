"""Fail-closed local scan envelope for Nav2-plan Pure Pursuit follow-only mode.

The scan is kept in follower/base_link coordinates. It is never interpolated
through filtered odometry or used to build a detour. A fresh map pose is only
used to place the footprint for a short swept-clearance check.
"""
import math
import numpy as np

from temporary_detour_candidate import Pose
from local_stop_turn_go import advance, minimum_clearance


def build_scene(*, pose_row, scan_row, now_ns, now_mono,
                timeout_s=0.8, max_pose_scan_delta_s=0.4,
                footprint_x_m=0.27, footprint_y_m=0.30,
                exclude_body_internal=False):
    """Return a map-frame scan scene or fail closed on stale/misaligned input.

    pose_row is (source_ns, receive_monotonic, (x,y,yaw,v,w)); scan_row is
    (source_ns, receive_monotonic, base-frame points).
    """
    if pose_row is None:
        raise ValueError('FOLLOW_MAP_POSE_MISSING')
    if scan_row is None:
        raise ValueError('FOLLOW_SCAN_MISSING')
    pose_ns, pose_rx, values = pose_row
    scan_ns, scan_rx, points = scan_row
    if (len(values) < 5 or not all(math.isfinite(float(v)) for v in values[:5])
            or len(points) == 0 or any(len(p) != 2 or not all(math.isfinite(float(v)) for v in p)
                                 for p in points)):
        raise ValueError('FOLLOW_INPUT_INVALID')
    for stamp, received, name in ((pose_ns, pose_rx, 'POSE'), (scan_ns, scan_rx, 'SCAN')):
        if stamp <= 0 or not 0 <= (now_ns - stamp) / 1e9 <= timeout_s:
            raise ValueError(f'FOLLOW_{name}_STALE')
        if not 0 <= now_mono - received <= timeout_s:
            raise ValueError(f'FOLLOW_{name}_RECEIVE_STALE')
    delta_s = abs(pose_ns - scan_ns) / 1e9
    if delta_s > max_pose_scan_delta_s:
        raise ValueError('FOLLOW_POSE_SCAN_TIME_MISMATCH')

    # Opt-in classification in base_link coordinates, before map projection.
    # Keep boundary and exterior returns. Never expand the existing footprint.
    excluded=[]
    if exclude_body_internal:
        kept=[]
        for point in points:
            bx,by=point
            if abs(bx)<footprint_x_m and abs(by)<footprint_y_m:
                excluded.append([float(bx),float(by)])
            else:
                kept.append(point)
        points=kept
        if len(points)==0:
            raise ValueError('FOLLOW_SCAN_NO_EXTERNAL_POINTS')

    x, y, yaw = (float(v) for v in values[:3])
    co, si = math.cos(yaw), math.sin(yaw)
    map_points = np.asarray(tuple((x + co * bx - si * by, y + si * bx + co * by)
                                  for bx, by in points), dtype=float)
    pose = Pose(x, y, yaw)
    # Bound uncertainty from pose/scan time separation plus a small fixed
    # allowance for scan discretization and bridge timing.
    uncertainty = 0.02 + (0.10 + math.hypot(footprint_x_m, footprint_y_m) * 0.20) * delta_s
    clearance = minimum_clearance(map_points, (x, y, yaw)) - uncertainty
    return dict(pose=pose, points=map_points, scan_stamp=scan_ns,
                source=min(pose_ns, scan_ns), uncertainty=uncertainty,
                clearance=clearance, pose_scan_delta_s=delta_s,
                source_stamps={'follower_map': pose_ns, 'scan': scan_ns},
                body_filter_enabled=exclude_body_internal,
                excluded_body_points=excluded)


def bounded_pursuit_result(scene, v, w, reason, *, max_linear=0.04,
                           max_angular=0.20, min_clearance=0.30,
                           max_rpm=10.0, wheel_radius=0.065,
                           half_wheelbase=0.28, horizon_s=0.5, steps=10,
                           allow_leader_rotation_wait=False,
                           allow_leader_reverse_wait=False,
                           enforce_clearance=True,min_linear=0.):
    """Bound a Pure Pursuit command and stop on current or swept obstacle risk."""
    if not math.isfinite(min_linear) or not 0<=min_linear<=max_linear:
        raise ValueError('INVALID_MINIMUM_LINEAR')
    pose = scene['pose']
    p = (pose.x, pose.y, pose.yaw)
    d = float(scene['clearance'])
    base = dict(v=0.0, w=0.0, motion='STOP', reason=reason,
                source_stamp_ns=int(scene['source']),
                scan_stamp_ns=int(scene['scan_stamp']), clearance_m=d,
                swept_clearance_m=d, pose=list(p), uncertainty_m=scene['uncertainty'],
                input_source_stamps=scene['source_stamps'],
                pose_scan_delta_s=scene['pose_scan_delta_s'],
                geometry_complete=False, pose_source='map')
    base['body_filter_enabled']=scene.get('body_filter_enabled',False)
    base['excluded_body_point_count']=len(scene.get('excluded_body_points',[]))
    base['clearance_stop_enabled']=enforce_clearance
    waiting_reasons=('STOP_LEADER_STOPPED','STOP_FOLLOW_DISTANCE')
    if allow_leader_rotation_wait:waiting_reasons+=('STOP_LEADER_ROTATING',)
    if allow_leader_reverse_wait:waiting_reasons+=('STOP_LEADER_REVERSE',)
    if reason.startswith('STOP_') and reason not in waiting_reasons:
        return base
    if enforce_clearance and d <= min_clearance:
        base['reason'] = 'STOP_FOLLOW_CLEARANCE'
        return base
    if reason=='STOP_LEADER_ROTATING':
        base['reason']='FOLLOW_WAITING_LEADER_ROTATION'
        return base
    if reason=='STOP_LEADER_REVERSE':
        base['reason']='FOLLOW_WAITING_LEADER_REVERSE'
        return base
    # Explicit experimental candidate only. Never raise a wait/fault/zero
    # command; preserve requested path curvature when raising a nonzero speed.
    if reason=='TRACKED_PURSUIT' and 0<v<min_linear:
        scale=min_linear/v
        v,w=v*scale,w*scale
    v = max(0.0, min(float(max_linear), float(v)))
    w = max(-float(max_angular), min(float(max_angular), float(w)))
    if v <= 0.0:
        base['reason'] = ({'STOP_LEADER_STOPPED': 'FOLLOW_WAITING_LEADER',
                           'STOP_FOLLOW_DISTANCE': 'FOLLOW_WAITING_DISTANCE'}.get(reason, reason))
        return base
    # This motor profile is forward-only. Respect its differential wheel RPM
    # bound and the envelope's non-reversing wheel constraint.
    w = max(-v / half_wheelbase, min(v / half_wheelbase, w))
    rpm = max(abs(v - w * half_wheelbase), abs(v + w * half_wheelbase)) / wheel_radius * 60 / (2 * math.pi)
    if rpm > max_rpm:
        scale = max_rpm / rpm
        v *= scale
        w *= scale

    swept = d
    for index in range(1, steps + 1):
        future = advance(p, v, w, horizon_s * index / steps)
        swept = min(swept, minimum_clearance(scene['points'], future) - scene['uncertainty'])
    base['swept_clearance_m'] = swept
    if enforce_clearance and swept <= min_clearance:
        base['reason'] = 'STOP_FOLLOW_SWEPT_CLEARANCE'
        return base
    base.update(v=v, w=w, motion='DRIVE', reason='TRACKED_PURSUIT')
    return base
