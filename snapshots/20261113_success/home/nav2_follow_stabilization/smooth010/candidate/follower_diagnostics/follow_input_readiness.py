"""Consecutive validated input readiness; never grants motion authorization."""


class InputReadiness:
    def __init__(self, required_s=5.0, max_check_gap_s=0.3):
        self.required_s = required_s
        self.max_check_gap_s = max_check_gap_s
        self.since = None
        self.last = None

    def update(self, valid, mono):
        gap_ok = self.last is None or 0 <= mono - self.last <= self.max_check_gap_s
        self.last = mono
        if not valid:
            self.since = None
            return False
        if self.since is None or not gap_ok:
            self.since = mono
        return mono - self.since >= self.required_s
