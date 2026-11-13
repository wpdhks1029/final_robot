import unittest
from unittest.mock import patch
from follower_drive_node import main
from follower_drive_candidate import BoundedDrivePort


class Tests(unittest.TestCase):
    def test_default_opens_nothing(self):
        with patch('sys.argv',['drive']),patch.object(BoundedDrivePort,'open') as opened:
            self.assertEqual(main(),0);opened.assert_not_called()

    def test_hardware_requires_ack_before_ros(self):
        with patch('sys.argv',['drive','--run-hardware']),patch.object(BoundedDrivePort,'open') as opened:
            with self.assertRaises(SystemExit):main()
            opened.assert_not_called()

    def test_backend_selection_is_exclusive(self):
        with patch('sys.argv',['drive','--run-hardware','--run-memory']),patch.object(BoundedDrivePort,'open') as opened:
            with self.assertRaises(SystemExit):main()
            opened.assert_not_called()


if __name__=='__main__':unittest.main()
