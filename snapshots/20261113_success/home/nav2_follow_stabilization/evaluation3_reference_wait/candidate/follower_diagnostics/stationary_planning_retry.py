"""Bounded fresh-snapshot retries while both robots are measured stopped."""
import math

class StationaryPlanningRetry:
    def __init__(self,mono):
        self.attempts=0;self.not_before=mono+1.5;self.last_scan=None;self.scans=0;self.clear_scans=0

    def ready(self,mono,stamp,motor,leader_v,obstacle_count=None):
        stopped=(motor.get('armed') is True and motor.get('feedback_fresh') is True
                 and motor.get('measured_rpm')==[0,0] and motor.get('zero_feedback_samples',0)>=3
                 and math.isfinite(leader_v) and abs(leader_v)<=.01)
        if not stopped:self.scans=0;self.clear_scans=0
        elif self.last_scan is None or stamp>self.last_scan:
            self.scans+=1
            self.clear_scans=self.clear_scans+1 if obstacle_count is not None and obstacle_count<5 else 0
        elif stamp<self.last_scan:self.scans=0;self.clear_scans=0
        self.last_scan=stamp
        return stopped and mono>=self.not_before and self.scans>=5 and self.attempts<3

    def submitted(self):self.attempts+=1

    def retry(self,reason,mono,stamp):
        allowed={'NO_VALIDATED_CONTINUOUS_ROUTE','REPLANNED_ROUTE_CHANGED','NO_OBSTACLE_CLUSTER'}
        if reason not in allowed or self.attempts>=3:return False
        self.not_before=mono+1.5;self.last_scan=stamp;self.scans=0;self.clear_scans=0
        return True
