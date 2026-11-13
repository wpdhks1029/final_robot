import unittest
from follow_reference_recovery import FollowReferenceRecovery, recoverable_reference_loss
from nav2_convoy_gate_core import Nav2ConvoyGateCore


class RecoveryTests(unittest.TestCase):
    def stamps(self, i):
        return dict(leader=100+i,path=100+i,follower_map=100+i,scan=100+i)

    def test_requires_stability_distinct_sources_and_zero(self):
        r=FollowReferenceRecovery();r.wait(0.)
        for i in range(11):
            self.assertFalse(r.recovered(.1*i,self.stamps(i),False))
        self.assertTrue(r.recovered(1.1,self.stamps(11),True))
        self.assertFalse(r.active)

    def test_replayed_identical_stamps_do_not_resume(self):
        r=FollowReferenceRecovery();r.wait(0.)
        for i in range(20):
            self.assertFalse(r.recovered(.1*i,self.stamps(0),True))

    def test_repeated_loss_resets_stability_but_not_timeout(self):
        r=FollowReferenceRecovery();r.wait(0.)
        for i in range(8):r.recovered(.1*i,self.stamps(i),True)
        r.wait(.8)
        self.assertFalse(r.recovered(.9,self.stamps(9),True))
        self.assertEqual(r.started,0.)
        with self.assertRaisesRegex(ValueError,'RECOVERY_TIMEOUT'):r.wait(10.)

    def test_check_gap_and_source_rollback_reset_readiness(self):
        r=FollowReferenceRecovery();r.wait(0.)
        for i in range(8):r.recovered(.1*i,self.stamps(i),True)
        self.assertFalse(r.recovered(1.2,self.stamps(12),True))
        self.assertFalse(r.recovered(1.3,self.stamps(1),True))

    def test_local_faults_geometry_and_tracking_errors_are_terminal(self):
        for reason in ('FOLLOW_SCAN_STALE','FOLLOW_POSE_STALE','FOLLOW_INPUT_INVALID',
                       'STOP_TARGET_BEHIND','STOP_ALIGN_REQUIRED','DEVICE_STATUS_NONZERO'):
            self.assertFalse(recoverable_reference_loss(reason))
        self.assertFalse(recoverable_reference_loss('STOP_MISSING_OR_INVALID_PATH',
            dict(reason='INVALID_GEOMETRY_OR_FRAME')))
        self.assertTrue(recoverable_reference_loss('STOP_MISSING_OR_INVALID_PATH',
            dict(reason='INVALID_SOURCE_TIME')))

    def test_existing_leader_gate_stops_on_wait_status(self):
        gate=Nav2ConvoyGateCore();gate.graph_ok=True;gate.update_nav(.04,0.,1.)
        gate.update_follower(dict(trial_phase='FOLLOW',reason='STOP_FOLLOW_REFERENCE_WAIT',
                                 follower_requested_v=0.),1.)
        output=gate.evaluate(1.)
        self.assertEqual((output.linear_x,output.angular_z),(0.,0.))
        self.assertEqual(output.reason,'STOP_FOLLOWER_SAFETY_STOP')


if __name__=='__main__':unittest.main()
