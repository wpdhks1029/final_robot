"""Initial frame for the bounded follow-only diagnostic; no saved-line rewrite."""
import math
from directed_path_tracker import DirectedPathTracker,wrap

def live_follow_reference(points,follower,leader):
    if not all(math.isfinite(v) for v in (*follower,*leader)):
        raise ValueError('FOLLOW_REFERENCE_INVALID_POSE')
    selection=DirectedPathTracker().select(points,follower,.35,tip_heading=leader[2])
    if selection['path_distance_m']>.15:raise ValueError('FOLLOW_REFERENCE_TOO_FAR_FROM_PATH')
    heading=selection['path_heading_rad']
    if abs(wrap(follower[2]-heading))>math.radians(20):raise ValueError('FOLLOW_REFERENCE_HEADING_MISMATCH')
    if abs(wrap(leader[2]-heading))>math.radians(20):raise ValueError('FOLLOW_REFERENCE_LEADER_HEADING_MISMATCH')
    dx,dy=leader[0]-follower[0],leader[1]-follower[1]
    if math.cos(follower[2])*dx+math.sin(follower[2])*dy<=0:raise ValueError('FOLLOW_REFERENCE_LEADER_NOT_AHEAD')
    origin=tuple(selection['projection_map'])
    cross=-math.sin(heading)*(follower[0]-origin[0])+math.cos(heading)*(follower[1]-origin[1])
    return origin,heading,cross,selection
