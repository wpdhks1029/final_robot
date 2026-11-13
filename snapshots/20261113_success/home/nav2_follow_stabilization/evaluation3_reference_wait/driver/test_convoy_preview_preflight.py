import unittest
from types import SimpleNamespace as S
from convoy_preview_preflight import wait_for_inputs,INPUTS,OUTPUTS


class Tests(unittest.TestCase):
    def test_delayed_discovery_beyond_two_seconds(self):
        now=[0.]
        node=S(get_publishers_info_by_topic=lambda t:[1] if t in INPUTS and now[0]>=6. else [])
        def spin():now[0]+=.1
        result=wait_for_inputs(node,spin,clock=lambda:now[0])
        self.assertTrue(now[0]>=6.)
        self.assertTrue(all(n==1 for n in result.values()))

    def test_existing_output_and_duplicate_input_rejected(self):
        for topic,count in [(OUTPUTS[0],1),(INPUTS[0],2)]:
            now=[0.]
            node=S(get_publishers_info_by_topic=lambda t:[1]*count if t==topic else [])
            def spin():now[0]+=.1
            with self.assertRaises(RuntimeError):wait_for_inputs(node,spin,clock=lambda:now[0])

    def test_missing_input_times_out_without_starting(self):
        now=[0.];node=S(get_publishers_info_by_topic=lambda t:[])
        def spin():now[0]+=.1
        with self.assertRaisesRegex(RuntimeError,'timed out'):
            wait_for_inputs(node,spin,clock=lambda:now[0])
        self.assertGreaterEqual(now[0],12.)


if __name__=='__main__':unittest.main()
