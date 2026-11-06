import unittest
from types import SimpleNamespace
from unittest.mock import Mock
from follower_drive_node import publish_position_guarded


class Tests(unittest.TestCase):
    def test_invalid_feedback_latches_and_disarms_without_publish_retry(self):
        core=SimpleNamespace(latch_fault=Mock(),feedback={'m1_status':1},valid_count=12)
        contract=SimpleNamespace(core=core,disarm=Mock())
        position=Mock();position.publish.side_effect=ValueError('Invalid position/time/device status')
        self.assertFalse(publish_position_guarded(position,contract,None))
        core.latch_fault.assert_called_once_with('POSITION_FEEDBACK_INVALID')
        contract.disarm.assert_called_once_with()
        self.assertEqual(contract.last_position_error['feedback']['m1_status'],1)
        position.publish.assert_called_once()

    def test_valid_feedback_unchanged(self):
        contract=Mock();position=Mock();position.publish.return_value=True
        self.assertTrue(publish_position_guarded(position,contract,123))
        contract.disarm.assert_not_called()
        contract.core.latch_fault.assert_not_called()

    def test_unexpected_programming_error_not_hidden(self):
        position=Mock();position.publish.side_effect=RuntimeError('unexpected')
        with self.assertRaises(RuntimeError):publish_position_guarded(position,Mock(),None)
