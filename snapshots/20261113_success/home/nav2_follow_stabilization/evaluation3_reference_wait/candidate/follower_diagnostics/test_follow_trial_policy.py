import unittest
from types import SimpleNamespace as S
from unittest.mock import Mock,patch
from run_guarded_follow_trial import ready_reason,main,trial_succeeded


class Tests(unittest.TestCase):
    def setup(self):
        motor=dict(mode='HARDWARE_CANDIDATE',input_kind='guard',enabled=True,armed=False,
                   fault=None,io_failed=False,feedback_fresh=True,measured_rpm=[0,0])
        guard=S(header=S(frame_id='follower/base_link',stamp=S(sec=1,nanosec=0)),
                twist=S(linear=S(x=.05),angular=S(z=0.)))
        core=Mock();core.command.return_value=(.05,0.,'PREVIEW_TRACK')
        return motor,guard,core

    def test_ready(self):
        s,g,c=self.setup();self.assertEqual(ready_reason((s,1.),(g,1.),c,1_100_000_000,1.1),'READY')

    def test_disabled_fault_already_armed_rejected(self):
        for k,v in [('enabled',False),('fault','SERIAL_WRITE_ERROR'),('armed',True),('feedback_fresh',False)]:
            s,g,c=self.setup();s[k]=v
            self.assertNotEqual(ready_reason((s,1.),(g,1.),c,1_100_000_000,1.1),'READY')

    def test_stopped_leader_no_arm_readiness(self):
        s,g,c=self.setup();c.command.return_value=(0.,0.,'STOP_LEADER_STOPPED')
        self.assertEqual(ready_reason((s,1.),(g,1.),c,1_100_000_000,1.1),'STOP_LEADER_STOPPED')

    def test_old_guard_and_no_guard_rejected(self):
        s,g,c=self.setup()
        self.assertNotEqual(ready_reason((s,1.4),(g,1.4),c,1_500_000_000,1.4),'READY')
        self.assertNotEqual(ready_reason((s,1.),None,c,1_100_000_000,1.1),'READY')

    def test_default_no_arm_or_ros(self):
        with patch('sys.argv',['trial']):self.assertEqual(main(),0)

    def test_post_stop_fault_cannot_report_success(self):
        self.assertTrue(trial_succeeded('TRIAL_TIME_LIMIT',True,True,set()))
        self.assertFalse(trial_succeeded('TRIAL_TIME_LIMIT',True,True,{'RESPONSE_TIMEOUT'}))
        self.assertFalse(trial_succeeded('TRIAL_TIME_LIMIT',True,False,set()))
        self.assertFalse(trial_succeeded('MOTOR_FAULT_OR_FEEDBACK_LOST',True,True,set()))


if __name__=='__main__':unittest.main()
