import unittest

from local_avoidance_drive import PROFILE
from run_local_avoidance_trial import motor_problem,motor_zero_feedback_trusted


class FollowDriverProfileTests(unittest.TestCase):
    def test_faulted_zero_rpm_does_not_confirm_physical_stop(self):
        good=dict(armed=False,fault=None,io_failed=False,feedback_fresh=True,
                  measured_rpm=[0,0],device_status=dict(m1=0,m2=0))
        self.assertTrue(motor_zero_feedback_trusted(good))
        for change in (dict(fault='DEVICE_STATUS_NONZERO'),
                       dict(device_status=dict(m1=0,m2=1)),
                       dict(feedback_fresh=False),dict(armed=True)):
            self.assertFalse(motor_zero_feedback_trusted(dict(good,**change)))

    def test_clearance_policy_must_match_driver_and_operator(self):
        state=self.status(False)
        self.assertEqual(motor_problem((state,5.),5.01,360,disable_clearance_stop=True),
                         'DRIVER_CLEARANCE_POLICY_MISMATCH')
        state['clearance_stop_enabled']=False
        self.assertIsNone(motor_problem((state,5.),5.01,360,disable_clearance_stop=True))
        self.assertEqual(motor_problem((state,5.),5.01,360),
                         'DRIVER_CLEARANCE_POLICY_MISMATCH')

    def status(self, continuous20):
        return dict(continuous20=continuous20,
                    continuous_stop_clearance_m=.1 if continuous20 else .3,
                    mode='HARDWARE_CANDIDATE', input_kind='local_avoidance',
                    motion_profile=PROFILE, input_source_timeout_s=.8,
                    trial_max_rpm=10, trial_max_seconds=360, enabled=True,
                    zero_stop_prearm_supported=True,
                    zero_stop_response_retry_supported=True,
                    zero_response_retry_version=2,
                    zero_reply_hold_supported=True,
                    fault=None, io_failed=False, feedback_fresh=True)

    def test_follow_only_rejects_continuous20_driver_with_specific_recovery_reason(self):
        problem = motor_problem((self.status(True), 5.), 5.01, 360,
                                continuous20=False, follow_only=True)
        self.assertEqual(problem,
                         'FOLLOW_ONLY_REQUIRES_STANDARD_DRIVER_WITHOUT_CONTINUOUS20_ACK')

    def test_follow_only_accepts_matching_standard_driver_profile(self):
        self.assertIsNone(motor_problem((self.status(False), 5.), 5.01, 360,
                                        continuous20=False, follow_only=True))


if __name__ == '__main__':
    unittest.main()
