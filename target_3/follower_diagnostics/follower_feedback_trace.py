"""Log valid measured PID216 samples only; no transport or ROS interfaces."""
import json


def delta32(current,previous):return ((current-previous+2**31)%2**32)-2**31


class FeedbackTrace:
    def __init__(self,stream):
        self.stream=stream;self.last_count=0;self.previous=None
        self.delta=[0,0];self.start=None

    def record(self,core,phase,stamp_ns):
        if core.valid_count==self.last_count or core.feedback is None:return False
        self.last_count=core.valid_count
        f=core.feedback;pos=[f['m1_position'],f['m2_position']]
        if self.previous is not None:
            for i in (0,1):self.delta[i]+=delta32(pos[i],self.previous[i])
        self.previous=pos
        if self.start is None:self.start=core.clock()
        row=dict(phase=phase,elapsed=core.clock()-self.start,stamp_ns=stamp_ns,
                 feedback_count=core.valid_count,position_unit='UNKNOWN',
                 m1_position=pos[0],m2_position=pos[1],
                 delta_m1=self.delta[0],delta_m2=self.delta[1],
                 m1_rpm=f['m1_rpm'],m2_rpm=f['m2_rpm'],
                 m1_status=f['m1_status'],m2_status=f['m2_status'],
                 odom_x=core.odom_x,odom_y=core.odom_y,odom_yaw=core.odom_yaw,
                 integration_skips=core.integration_skips)
        self.stream.write(json.dumps(row,allow_nan=False)+'\n');self.stream.flush()
        return True
