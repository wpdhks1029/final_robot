import unittest
from unittest.mock import patch
from inspect_wheel_position import delta32,main,ReadOnlyPort


class Tests(unittest.TestCase):
    def test_forward_reverse(self):
        self.assertEqual(delta32(70,10),60)
        self.assertEqual(delta32(-50,10),-60)

    def test_signed_wrap(self):
        self.assertEqual(delta32(-2**31,2**31-1),1)
        self.assertEqual(delta32(2**31-1,-2**31),-1)

    def test_default_no_hardware(self):
        with patch('sys.argv',['position']),patch.object(ReadOnlyPort,'open') as opened:
            self.assertEqual(main(),0);opened.assert_not_called()


if __name__=='__main__':unittest.main()
