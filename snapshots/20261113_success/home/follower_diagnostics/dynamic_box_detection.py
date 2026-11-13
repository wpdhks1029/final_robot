"""Footprint-relative forward trigger. Does not issue motion commands."""
import math
import numpy as np


def front_obstacle_count(points, pose, distance=.7):
    cloud = np.asarray(points, dtype=float).reshape(-1, 2)
    dx, dy = cloud[:, 0] - pose[0], cloud[:, 1] - pose[1]
    co, si = math.cos(pose[2]), math.sin(pose[2])
    bx, by = co * dx + si * dy, -si * dx + co * dy
    # Distance from the front bumper, with a 65cm lateral detection band.
    return int(np.sum(np.isfinite(bx) & np.isfinite(by) &
                      (bx > .27) & (bx <= .27 + distance) & (abs(by) < .65)))
