"""Bounded zero-output re-observation. No point masking or stale-input bypass."""
from evaluation3_clearance_policy import PLAN_CLEARANCE_M,STOP_CLEARANCE_M

class ContinuousNearReturnPause:
    def __init__(self):
        self.active=False;self.started=None;self.last_scan=None;self.clear_scans=0;self.episodes=0

    def apply(self,row,mono,motor):
        reason=row['reason'];event='waiting'
        if not self.active:
            if reason!='STOP_FOOTPRINT_CLEARANCE':return row
            self.episodes+=1
            if self.episodes>2:return dict(row,reason='STOP_NEAR_RETURN_REPEAT_LIMIT')
            self.active=True;self.started=mono;self.last_scan=None;self.clear_scans=0;event='started'
        # Unrelated failures keep their original stop semantics.
        if reason not in {'STOP_FOOTPRINT_CLEARANCE','STOP_SWEPT_CLEARANCE','CONTINUOUS_READY','CONTINUOUS_RAMP_WARMUP','FOLLOW_WAITING_LEADER'}:
            return row
        stationary=(motor.get('armed') is True and motor.get('feedback_fresh') is True
                    and motor.get('measured_rpm')==[0,0] and motor.get('zero_feedback_samples',0)>=3)
        safe=(stationary and reason in {'CONTINUOUS_READY','CONTINUOUS_RAMP_WARMUP','FOLLOW_WAITING_LEADER'}
              and row['clearance_m']>=PLAN_CLEARANCE_M and row['swept_clearance_m']>STOP_CLEARANCE_M+.05)
        stamp=row['scan_stamp_ns']
        if self.last_scan is None or stamp>self.last_scan:
            self.last_scan=stamp;self.clear_scans=self.clear_scans+1 if safe else 0
        elif stamp<self.last_scan or not safe:self.clear_scans=0
        meta=dict(event=event,clear_scans=self.clear_scans,required_clear_scans=5,
                  elapsed_s=mono-self.started,timeout_s=2.,episode=self.episodes,original_reason=reason)
        if mono-self.started>=2.:
            return dict(row,v=0.,w=0.,motion='STOP',reason='STOP_NEAR_RETURN_WAIT_TIMEOUT',near_return_pause=meta)
        if safe and self.clear_scans>=5:
            self.active=False;meta['event']='resumed'
            # Resume on the following tick, with a fresh command and sweep check.
            return dict(row,v=0.,w=0.,motion='STOP',reason='CONTINUOUS_RECHECK_COMPLETE',near_return_pause=meta)
        return dict(row,v=0.,w=0.,motion='STOP',reason='NEAR_RETURN_PAUSE',near_return_pause=meta)
