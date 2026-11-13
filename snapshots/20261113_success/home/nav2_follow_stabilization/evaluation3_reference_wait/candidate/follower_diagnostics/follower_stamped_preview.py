"""Source-age helpers for parallel stamped TEST commands; no ROS/hardware access."""
import math

FRAME='follower/base_link'
PURSUIT_TOPIC='/follower/cmd_vel_stamped_test'
GUARD_TOPIC='/follower/cmd_vel_guard_stamped_test'


def stamp_ns(stamp):return stamp.sec*10**9+stamp.nanosec


def oldest_pursuit_source(inputs,*,remote_clock_margin_ns=0):
    stamps=[]
    for name in ('path','leader','follower'):
        if name not in inputs:return 0
        value=stamp_ns(inputs[name][0])
        if value<=0:return 0
        # 원격 시각 허용 폭을 측정 시각에서 차감해 downstream의 age 검사를 보수적으로 유지.
        if name in ('path','leader'):
            value-=remote_clock_margin_ns
            if value<=0:return 0
        stamps.append(value)
    return min(stamps)


class StampedGuard:
    def __init__(self,core):
        self.core=core;self.source=None;self.timeout=.3

    def update(self,msg,now_ns,received):
        self.source=None;self.core.raw=None
        original=stamp_ns(msg.header.stamp)
        if msg.header.frame_id!=FRAME or original<=0 or not 0<=(now_ns-original)/1e9<=self.timeout:return
        self.core.update_raw(msg.twist,received)
        if self.core.raw is not None:self.source=(original,received)

    def command(self,now_ns,received):
        if self.source is None:return 0.,0.,0,'STOP_SOURCE_INVALID'
        original,rx=self.source
        if not (0<=(now_ns-original)/1e9<=self.timeout and 0<=received-rx<=self.timeout):
            return 0.,0.,0,'STOP_SOURCE_STALE'
        scan=self.core.scan
        if scan is None or not (0<=(now_ns-scan[0])/1e9<=self.timeout and 0<=received-scan[1]<=self.timeout):
            return 0.,0.,0,'STOP_SCAN_STALE_OR_INVALID'
        v,w,reason=self.core.command(now_ns,received)
        return v,w,min(original,scan[0]),reason
