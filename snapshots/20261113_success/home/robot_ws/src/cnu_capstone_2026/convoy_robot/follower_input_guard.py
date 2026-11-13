"""ROS-independent validation for follower candidate nodes."""
import math


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def fresh(stamp, now_ns, timeout):
    value = stamp_ns(stamp)
    return value > 0 and 0 <= (now_ns - value) / 1e9 <= timeout


def valid_odom(msg, frame, child):
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    v = msg.twist.twist.linear
    w = msg.twist.twist.angular
    values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w,
              v.x, v.y, v.z, w.x, w.y, w.z)
    norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w
    return (msg.header.frame_id == frame and msg.child_frame_id == child
            and all(math.isfinite(x) for x in values)
            and abs(norm - 1.0) < 0.01)


def valid_path(msg):
    # Historical path points may be old; freshness applies to the path header.
    return (msg.header.frame_id == 'map' and len(msg.poses) >= 2
            and all(p.header.frame_id == 'map'
                    and math.isfinite(p.pose.position.x)
                    and math.isfinite(p.pose.position.y) for p in msg.poses))
