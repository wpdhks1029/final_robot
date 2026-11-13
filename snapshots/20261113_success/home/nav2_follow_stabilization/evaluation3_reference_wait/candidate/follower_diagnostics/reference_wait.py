"""Unbounded zero-speed reference wait; freshness thresholds still gate motion."""
MAX_MOTION_SOURCE_AGE_S=.6  # reserve .2s before the motor's .8s source limit

class ReferenceUnavailable(ValueError):pass

def reference_stamps(nav,core,now,mono,require_path):
    rows={'leader':nav.inputs.get('leader'),'follower_map':nav.inputs.get('follower')}
    if require_path:rows['path']=nav.inputs.get('path')
    result={}
    for key,row in rows.items():
        if row is None:raise ReferenceUnavailable(key.upper()+'_MISSING')
        stamp=row[0].sec*10**9+row[0].nanosec-(50_000_000 if key in ('leader','path') else 0)
        if not 0<=(now-stamp)/1e9<=MAX_MOTION_SOURCE_AGE_S or not 0<=mono-row[1]<=MAX_MOTION_SOURCE_AGE_S:
            raise ReferenceUnavailable(key.upper()+'_NOT_FRESH_FOR_MOTION')
        result[key]=stamp
    if core.scan is None:raise ValueError('SCAN_MISSING_OR_INVALID')
    result['scan']=core.scan[0]
    return result

class ReferenceWait:
    def __init__(self):
        self.started=None;self.stable_since=None;self.last_poll=None;self.samples={};self.episodes=0;self.total_wait_s=0.
    @property
    def active(self):return self.started is not None
    def enter(self,mono,*,invalid=False):
        first=not self.active
        if first:self.started=mono;self.episodes+=1
        if invalid:self.stable_since=None;self.last_poll=None;self.samples={}
        return first
    def ready(self,mono,stamps,zero_feedback):
        if not self.active:return True
        if self.last_poll is not None and mono-self.last_poll>.25:self.stable_since=None;self.samples={}
        if any(type(v) is not int or v<=0 for v in stamps.values()):self.enter(mono,invalid=True);return False
        if any(k in self.samples and self.samples[k] and v<max(self.samples[k]) for k,v in stamps.items()):
            self.enter(mono,invalid=True);return False
        if self.stable_since is None:self.stable_since=mono
        self.last_poll=mono
        for key,value in stamps.items():
            samples=self.samples.setdefault(key,set());samples.add(value)
            if len(samples)>3:samples.remove(min(samples))
        return zero_feedback and mono-self.stable_since>=1. and all(len(self.samples[k])>=3 for k in stamps)
    def finish(self,mono):
        elapsed=mono-self.started;self.total_wait_s+=elapsed;self.started=None;self.stable_since=None;self.last_poll=None;self.samples={}
        return elapsed
