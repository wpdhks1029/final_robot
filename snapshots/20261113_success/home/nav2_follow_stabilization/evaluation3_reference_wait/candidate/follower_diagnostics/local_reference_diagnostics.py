"""Read-only explanation of the saved-line alignment gate."""
import math


def alignment_diagnostics(reference,path,follower,leader,*,source_kind='live_path'):
    snapshot=reference['snapshot'];anchor=snapshot['reference_origin'];heading=snapshot['reference_heading']
    co,si=math.cos(heading),math.sin(heading)
    cross=lambda p:-si*(p[0]-anchor[0])+co*(p[1]-anchor[1])
    yaw=lambda p:math.degrees(math.atan2(math.sin(p[2]-heading),math.cos(p[2]-heading)))
    worst=max(path,key=lambda p:abs(cross(p))) if path else None
    values=dict(follower_lateral_m=cross(follower),leader_lateral_m=cross(leader),
                path_max_lateral_m=abs(cross(worst)) if worst is not None else None,
                follower_heading_error_deg=yaw(follower),leader_heading_error_deg=yaw(leader))
    failed=[]
    for key in ('follower_lateral_m','leader_lateral_m'):
        if abs(values[key])>.15:failed.append(key)
    if source_kind=='live_path' and values['path_max_lateral_m'] is not None and values['path_max_lateral_m']>.10:failed.append('path_max_lateral_m')
    for key in ('follower_heading_error_deg','leader_heading_error_deg'):
        if abs(values[key])>20:failed.append(key)
    return dict(**values,failed_checks=failed,saved_origin=list(anchor),saved_heading_deg=math.degrees(heading),
                follower_map_pose=list(follower[:3]),leader_map_pose=list(leader[:3]),
                path_points=len(path),worst_path_point=list(worst) if worst is not None else None,
                reference_input_kind=source_kind,lateral_limit_m=.15,path_limit_m=.10 if source_kind=='live_path' else None,heading_limit_deg=20,
                control_connected=False,reference_changed=False)
