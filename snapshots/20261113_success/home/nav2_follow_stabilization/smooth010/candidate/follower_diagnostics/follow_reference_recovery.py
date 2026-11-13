"""Bounded remote-reference wait; never authorizes motion by itself."""
from follow_input_readiness import InputReadiness


REMOTE_LOSS = frozenset(('FOLLOW_REFERENCE_STALE_OR_MISSING',
    'STOP_STALE_PATH', 'STOP_STALE_LEADER',
    'STOP_MISSING_OR_INVALID_PATH', 'STOP_MISSING_OR_INVALID_LEADER'))


def recoverable_reference_loss(reason, rejection=None):
    if reason not in REMOTE_LOSS:
        return False
    if reason.startswith('STOP_MISSING_OR_INVALID_') and rejection:
        return rejection.get('reason') == 'INVALID_SOURCE_TIME'
    return True


class FollowReferenceRecovery:
    def __init__(self, timeout_s=10., stable_s=1.):
        self.timeout_s = timeout_s
        self.readiness = InputReadiness(stable_s, .25)
        self.started = None
        self.count = 0
        self.reset_samples()

    @property
    def active(self):
        return self.started is not None

    def reset_samples(self):
        self.samples = {key:set() for key in ('leader','path','follower_map','scan')}

    def wait(self, mono):
        if not self.active:
            self.started = mono
            self.count += 1
        self.readiness.update(False, mono)
        self.reset_samples()
        self.check_timeout(mono)

    def check_timeout(self, mono):
        if self.active and mono-self.started >= self.timeout_s:
            raise ValueError('FOLLOW_REFERENCE_RECOVERY_TIMEOUT')

    def recovered(self, mono, stamps, zero_feedback):
        self.check_timeout(mono)
        if not self.active:
            return True
        if any(type(stamps.get(k)) is not int or stamps[k]<=0 for k in self.samples):
            self.wait(mono)
            return False
        for key, values in self.samples.items():
            if values and stamps[key] < max(values):
                self.wait(mono)
                return False
        if self.readiness.last is not None and mono-self.readiness.last > .25:
            self.reset_samples()
        # Bounded sets: only the latest three distinct stamps are needed.
        for key, values in self.samples.items():
            values.add(stamps[key])
            if len(values)>3:
                values.remove(min(values))
        stable = self.readiness.update(True, mono)
        if stable and zero_feedback and all(len(v)>=3 for v in self.samples.values()):
            self.started = None
            return True
        return False
