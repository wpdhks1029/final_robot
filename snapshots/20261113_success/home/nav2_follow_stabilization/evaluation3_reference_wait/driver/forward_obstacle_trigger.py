"""Avoidance trigger only. Does not filter the collision-checking cloud."""
import math
import numpy as np

FRONT_TRIGGER_M=.70

def front_obstacle_count(points,pose):
    cloud=np.asarray(points,dtype=float)
    if cloud.size==0:return 0
    x,y,yaw=pose
    dx=cloud[:,0]-x;dy=cloud[:,1]-y;c,s=math.cos(yaw),math.sin(yaw)
    bx=c*dx+s*dy;by=-s*dx+c*dy
    # Forward distance from the front body edge, not the LiDAR or body centre.
    return int(np.sum((bx>.27)&(bx-.27<=FRONT_TRIGGER_M)&(abs(by)<.65)))
