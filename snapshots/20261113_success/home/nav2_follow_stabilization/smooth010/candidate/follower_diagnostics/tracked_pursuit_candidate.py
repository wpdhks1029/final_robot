"""Opt-in map path tracking; existing speed gate and command limits retained."""
import math
from follower_pursuit_preview import PreviewCore
from directed_path_tracker import DirectedPathTracker

class TrackedPreviewCore(PreviewCore):
    def __init__(self,lookahead=.8):
        super().__init__()
        if not math.isfinite(lookahead) or not .2<=lookahead<=1.:raise ValueError('INVALID_TRACKED_LOOKAHEAD')
        self.lookahead=lookahead;self.path_tracker=DirectedPathTracker();self.last_selection=None
    def command(self,now_ns,received):
        v,_,reason=self.speed_gate(now_ns,received)
        if v<=0:return 0.,0.,reason
        pose=self.inputs['follower'][2]
        try:self.last_selection=self.path_tracker.select(self.inputs['path'][2],pose,self.lookahead,tip_heading=self.inputs['leader'][2][2])
        except ValueError as exc:return 0.,0.,'STOP_'+str(exc)
        target=self.last_selection['target_map'];dx,dy=target[0]-pose[0],target[1]-pose[1]
        co,si=math.cos(pose[2]),math.sin(pose[2]);x=co*dx+si*dy;y=-si*dx+co*dy
        if x<=0:return 0.,0.,'STOP_TARGET_BEHIND'
        if abs(math.atan2(y,x))>self.heading_limit:return 0.,0.,'STOP_ALIGN_REQUIRED'
        squared=x*x+y*y
        if squared<1e-6:return 0.,0.,'STOP_TARGET_TOO_CLOSE'
        w=max(-self.max_angular,min(self.max_angular,2*y/squared*v))
        return v,w,'TRACKED_PURSUIT'
