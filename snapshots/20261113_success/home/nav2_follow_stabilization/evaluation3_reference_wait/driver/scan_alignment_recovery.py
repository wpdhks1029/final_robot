"""Separate fresh-data waiting from measured braking; always zero while waiting."""
class ScanAlignmentRecovery:
    def __init__(self):self.start=None;self.data_wait=None;self.brake_wait=None
    def missing(self,mono):
        if self.start is None:self.start=mono
        if self.data_wait is None:self.data_wait=mono
        if mono-self.start>=2.4 or mono-self.data_wait>=.4:raise ValueError('FILTERED_SCAN_ALIGNMENT_TIMEOUT')
    def matched(self,mono,motor):
        self.data_wait=None
        if self.brake_wait is None:self.brake_wait=mono
        if mono-self.start>=2.4 or mono-self.brake_wait>=2.:raise ValueError('FILTERED_SCAN_BRAKE_CONFIRM_TIMEOUT')
        return (motor.get('feedback_fresh') is True and motor.get('armed') is True
                and motor.get('measured_rpm')==[0,0] and motor.get('zero_feedback_samples',0)>=3)
    def reset(self):self.start=self.data_wait=self.brake_wait=None
