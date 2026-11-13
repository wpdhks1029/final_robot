"""Opt-in phase-bounded pivot contract. Importing opens no hardware/ROS."""
import json,math,re,struct
from follower_drive_candidate import BoundedDrivePort,DriveContract
from follower_motor_candidate import read_request,velocity_packet
from follower_motor_mock import strict_pairs
from local_avoidance_limits import SOURCE_TIMEOUT_S

PROFILE='LOCAL_STOP_TURN_GO_V1'
COMMAND_TOPIC='/follower/motor_candidate/local_avoidance_command'


def parse_command(text):
    if not isinstance(text,str) or len(text)>2048:raise ValueError('Invalid envelope size')
    obj=json.loads(text,object_pairs_hook=strict_pairs)
    fields={'version','controller','epoch','frame','stamp_ns','source_stamp_ns','scan_stamp_ns',
            'motion','v','w','clearance_m','swept_clearance_m'}
    if not isinstance(obj,dict) or set(obj)!=fields:raise ValueError('Unexpected local avoidance fields')
    if type(obj['version']) is not int or obj['version']!=3 or obj['controller']!=PROFILE:raise ValueError('Wrong profile/version')
    if obj['frame']!='follower/base_link' or not isinstance(obj['epoch'],str) or not re.fullmatch('[0-9a-f]{16}',obj['epoch']):raise ValueError('Invalid frame/epoch')
    for key in ('stamp_ns','source_stamp_ns','scan_stamp_ns'):
        if type(obj[key]) is not int or obj[key]<=0:raise ValueError('Invalid measurement stamp')
    for key in ('v','w','clearance_m','swept_clearance_m'):
        if type(obj[key]) not in (int,float) or not math.isfinite(obj[key]):raise ValueError('Invalid numeric value')
    if obj['motion'] not in ('STOP','DRIVE','TURN'):raise ValueError('Unknown motion phase')
    return obj


class PivotDrivePort(BoundedDrivePort):
    def __init__(self,device,clock=__import__('time').monotonic,*,max_rpm=10):
        super().__init__(device,clock,max_rpm=max_rpm);self.motion_kind='STOP'
    def close_gate(self):
        super().close_gate();self.motion_kind='STOP'
    def write(self,packet):
        if packet in (read_request(1),velocity_packet(0,0,1)):return self.device.write(packet)
        if (len(packet)!=13 or packet[:5]!=b'\xb7\xb8\x01\xcf\x07' or packet[5]!=1
                or packet[8]!=1 or packet[11]!=1 or sum(packet)&255):raise ValueError('Invalid pivot packet')
        if self.deadline is None or not self.clock()<self.deadline:raise ValueError('Motion gate locked/expired')
        left,right=struct.unpack_from('<h',packet,6)[0],struct.unpack_from('<h',packet,9)[0]
        if max(abs(left),abs(right))>self.max_rpm:raise ValueError('Wheel RPM bound exceeded')
        if self.motion_kind=='TURN':
            if left!=right:raise ValueError('Pivot requires equal raw RPM; no reversing translation')
        elif self.motion_kind=='CURVE':
            # Raw left sign is inverted; net body translation must be forward.
            if right-left<0:raise ValueError('Curve cannot reverse body translation')
        elif self.motion_kind=='DRIVE':
            if not -self.max_rpm<=left<=0 or not 0<=right<=self.max_rpm:raise ValueError('Drive phase is forward only')
        else:raise ValueError('No active motion phase')
        return self.device.write(packet)


class LocalAvoidanceContract(DriveContract):
    def __init__(self,port,*,enabled=False,clock=__import__('time').monotonic,max_rpm=10,trial_seconds=30,distance_test_only=False,extended_follow=False):
        if not isinstance(port,PivotDrivePort) or max_rpm!=(15 if extended_follow else 10) or trial_seconds not in (0,30,60,360):
            raise ValueError('Local avoidance requires pivot port, matching 10/15RPM profile and 0 (unlimited)/30/60/360s')
        if trial_seconds==0 and distance_test_only:
            raise ValueError('Distance tests require a finite session limit')
        super().__init__(port,enabled=enabled,clock=clock,max_rpm=max_rpm,
                         trial_seconds=60 if trial_seconds==0 else min(trial_seconds,60))
        self.core.zero_stop_reply_retry_enabled=True
        self.trial_seconds=trial_seconds;self.last_nonzero_motion=None;self.last_turn_sign=0
        self.phase_waiting_for_zero=False
        self.zero_reply_hold_active=False
        self.continuous20=False
        self.extended_follow=bool(extended_follow)
        self.clearance_stop_enabled=True
        self.distance_test_only=distance_test_only;self.distance_travelled=0.;self.distance_previous=None
    def arm(self,ros_ns):
        epoch=super().arm(ros_ns)
        if epoch:self.last_nonzero_motion=None;self.last_turn_sign=0;self.port.motion_kind='STOP'
        if epoch and self.trial_seconds==0:
            self.deadline=None
            self.port.open_gate(self.core.clock()+self.core.config.command_timeout)
        if epoch and self.distance_test_only:
            self.distance_travelled=0.;self.distance_previous=(self.core.odom_x,self.core.odom_y)
            self.deadline=self.core.clock()+min(45.,self.trial_seconds);self.port.open_gate(self.deadline)
        return epoch
    def command(self,text,ros_ns,*,require_source=True):
        if not self.core.armed or self.core.closed:self.rejected+=1;return False
        obj=None
        try:
            obj=parse_command(text);arm=self.core.arm_ros_ns
            if self.extended_follow and (self.continuous20 or self.distance_test_only):
                raise ValueError('Extended follow cannot use other profiles')
            # Zero output can safely maintain the newly armed session while
            # waiting for post-arm sensor/odometry support. Motion never uses
            # pre-arm measurements. Header epoch/time checks still apply below.
            zero_stop=obj['motion']=='STOP' and obj['v']==0. and obj['w']==0.
            lower=1 if zero_stop else arm
            if any(not lower<=obj[key]<=obj['stamp_ns'] or not 0<=(ros_ns-obj[key])/1e9<=SOURCE_TIMEOUT_S
                   for key in ('source_stamp_ns','scan_stamp_ns')):raise ValueError('Stale/future/pre-arm source')
            v,w,motion=obj['v'],obj['w'],obj['motion']
            if self.distance_test_only and motion!='STOP' and (motion!='DRIVE' or not 0<v<=.04 or w!=0.):
                raise ValueError('Distance test requires forward <=0.04m/s and zero yaw')
            if motion=='STOP':
                if v!=0. or w!=0.:raise ValueError('STOP must be zero')
            else:
                if self.clearance_stop_enabled and not self.distance_test_only and (obj['clearance_m']<=(.1 if self.continuous20 else .3) or obj['swept_clearance_m']<=(.1 if self.continuous20 else .3)):raise ValueError('Clearance not established')
                if self.continuous20 and motion=='TURN':raise ValueError('Continuous profile forbids pivot')
                if motion=='TURN':
                    if v!=0. or not 0<abs(w)<=.2:raise ValueError('TURN requires zero translation and bounded yaw')
                elif not (0<v<=(.025 if self.continuous20 else (.10 if self.extended_follow else .06)) and abs(w)<=.2 and (self.continuous20 or v>=abs(w)*.28) and (not self.continuous20 or abs(w)<=8*v+1e-9)):raise ValueError('Invalid forward DRIVE')
                rpm=max(abs(v-w*.28),abs(v+w*.28))/.065*60/(2*math.pi)
                if rpm>self.core.config.max_rpm+1e-9:raise ValueError('Wheel limit')
                sign=1 if w>0 else -1
                changing=(self.last_nonzero_motion!=motion or (motion=='TURN' and self.last_turn_sign not in (0,sign)))
                self.phase_waiting_for_zero=changing and self.zero_samples<3
                if self.phase_waiting_for_zero:
                    # Status may lag the newest wheel measurement. Validate the
                    # original envelope, then hold a real zero command locally;
                    # never execute or queue the requested nonzero transition.
                    v=w=0.;motion='STOP'
            if obj['motion']=='STOP':self.phase_waiting_for_zero=False
            self.zero_reply_hold_active=(motion!='STOP' and self.core.waiting_since is not None
                                         and self.core.last_sent==(0,0))
            if self.zero_reply_hold_active:
                # Do not replace an outstanding zero transaction with a motion
                # intent. Keep commanding zero until a complete measured reply;
                # this also keeps the bounded zero-only retry eligible.
                # A later fresh envelope is required to resume motion.
                v=w=0.;motion='STOP'
            self.port.motion_kind='CURVE' if self.continuous20 and motion=='DRIVE' else motion
            ok=self.core.accept_command(v,w,frame=obj['frame'],stamp_ns=obj['stamp_ns'],ros_now_ns=ros_ns,epoch=obj['epoch'])
            if ok and motion!='STOP':
                if self.last_nonzero_motion!=motion or (motion=='TURN' and self.last_turn_sign not in (0,1 if w>0 else -1)):self.zero_samples=0
                self.last_nonzero_motion=motion
                if motion=='TURN':self.last_turn_sign=1 if w>0 else -1
            if not ok:raise ValueError(self.core.fault or 'Core rejected command')
        except (ValueError,TypeError,KeyError,OverflowError,RecursionError) as exc:
            self.last_command_rejection=dict(reason=str(exc),received_ros_ns=ros_ns,profile=PROFILE)
            self.core.latch_fault('INVALID_LOCAL_AVOIDANCE_ENVELOPE');self.port.close_gate();self.rejected+=1;return False
        self.accepted+=1;return True
    def tick(self,ros_ns):
        if self.trial_seconds==0 and (self.continuous20 or self.distance_test_only):
            self.core.latch_fault('UNLIMITED_SESSION_PROFILE_MISMATCH');self.port.close_gate()
        if self.trial_seconds==0 and self.core.armed and self.core.fault is None:
            # No wall-clock session limit. Preserve a short byte-level lease
            # rather than opening an infinite motion gate. The core retains
            # its independent command/header/feedback/response deadlines.
            now=self.core.clock();command=self.core.command
            recent=(0<=now-self.core.armed_at<=self.core.config.command_timeout if command is None
                else 0<=now-command[2]<=self.core.config.command_timeout
                    and 0<=(ros_ns-command[3])/1e9<=self.core.config.command_timeout)
            if recent and self.core.feedback_fresh(now):
                self.port.open_gate(now+self.core.config.command_timeout)
        if self.distance_test_only and self.core.armed:
            p=(self.core.odom_x,self.core.odom_y)
            if self.distance_previous is not None:self.distance_travelled+=math.dist(p,self.distance_previous)
            self.distance_previous=p
            if self.distance_travelled>=1.2:self.core.latch_fault('DISTANCE_TEST_1_2M_LIMIT');self.port.close_gate()
        if self.core.armed and self.deadline is not None and self.core.clock()>=self.deadline:
            self.core.latch_fault('LOCAL_AVOIDANCE_TRIAL_EXPIRED')
        super().tick(ros_ns)
    def status(self):
        return dict(super().status(),motion_profile=PROFILE,motion_phase=self.port.motion_kind,
                    session_unlimited=self.trial_seconds==0,
                    trial_remaining_s=None if self.trial_seconds==0 else
                        (0. if not self.core.armed or self.deadline is None else max(0.,self.deadline-self.core.clock())),
                    clearance_stop_enabled=self.clearance_stop_enabled,extended_follow_speed=self.extended_follow,
                    zero_stop_prearm_supported=True,
                    zero_stop_response_retry_supported=True,
                    zero_response_retry_version=2,
                    phase_zero_wait_supported=True,phase_waiting_for_zero=self.phase_waiting_for_zero,
                    zero_reply_hold_supported=True,zero_reply_hold_active=self.zero_reply_hold_active,
                    continuous20=self.continuous20,continuous_stop_clearance_m=.1 if self.continuous20 else .3,distance_test_only=self.distance_test_only,distance_test_travelled_m=self.distance_travelled,
                    zero_feedback_samples=self.zero_samples,input_source_timeout_s=SOURCE_TIMEOUT_S,
                    command_timeout_s=self.core.config.command_timeout,feedback_timeout_s=self.core.config.feedback_timeout,
                    response_timeout_s=self.core.config.response_timeout)
