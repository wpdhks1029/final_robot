import unittest
from unittest.mock import Mock
from follower_guard_gateway import wait_for_publishers


class Tests(unittest.TestCase):
    def test_delayed_discovery_beyond_two_seconds(self):
        now=[0.];node=Mock()
        node.get_publishers_info_by_topic.side_effect=lambda topic:[object()] if now[0]>=4 else []
        def spin():now[0]+=.1
        result=wait_for_publishers(node,spin,clock=lambda:now[0])
        self.assertGreaterEqual(now[0],4);self.assertTrue(all(n==1 for n in result.values()))

    def test_missing_source_times_out(self):
        now=[0.];node=Mock();node.get_publishers_info_by_topic.return_value=[]
        def spin():now[0]+=.1
        with self.assertRaises(RuntimeError):wait_for_publishers(node,spin,timeout=.5,clock=lambda:now[0])

    def test_duplicate_still_refused(self):
        node=Mock();node.get_publishers_info_by_topic.return_value=[1,2]
        with self.assertRaises(RuntimeError):wait_for_publishers(node,lambda:None,clock=lambda:0.)


if __name__=='__main__':unittest.main()
