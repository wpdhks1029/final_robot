"""Zero-output temporal pause; never deletes or alters a measured point."""


class NearReturnPause:
    def __init__(self):
        self.active=False;self.started=None;self.last_scan=None;self.clear_scans=0

    def apply(self,row,mono,motor):
        reason=row['reason'];event=None
        if not self.active:
            if reason!='STOP_FOOTPRINT_CLEARANCE':return row
            self.active=True;self.started=mono;self.last_scan=None;self.clear_scans=0
            event='started'
        # This feature must never suppress an unrelated safety failure.
        allowed={'STOP_FOOTPRINT_CLEARANCE','STOP_SWEPT_CLEARANCE','STOP_APF_STEP_BLOCKED',
                 'LOCAL_AVOIDANCE_READY','BRAKE_BEFORE_PHASE_CHANGE','BRAKE_BEFORE_APF_SELECTION'}
        if reason not in allowed:return row
        safe=(reason in {'LOCAL_AVOIDANCE_READY','BRAKE_BEFORE_PHASE_CHANGE','BRAKE_BEFORE_APF_SELECTION'}
              and row['clearance_m']>=.35 and row['swept_clearance_m']>.30)
        stamp=row['scan_stamp_ns']
        if self.last_scan is None or stamp>self.last_scan:
            self.last_scan=stamp;self.clear_scans=self.clear_scans+1 if safe else 0
        elif not safe:self.clear_scans=0
        metadata=dict(event=event or 'waiting',clear_scans=self.clear_scans,required_clear_scans=5,
                      elapsed_s=mono-self.started,timeout_s=2.,original_reason=reason)
        if mono-self.started>=2.:
            return dict(row,v=0.,w=0.,motion='STOP',reason='STOP_NEAR_RETURN_WAIT_TIMEOUT',near_return_pause=metadata)
        if (safe and self.clear_scans>=5 and motor.get('measured_rpm')==[0,0]
                and motor.get('zero_feedback_samples',0)>=3 and reason=='LOCAL_AVOIDANCE_READY'):
            self.active=False;metadata['event']='resumed'
            return dict(row,near_return_pause=metadata)
        return dict(row,v=0.,w=0.,motion='STOP',reason='NEAR_RETURN_PAUSE',near_return_pause=metadata)
