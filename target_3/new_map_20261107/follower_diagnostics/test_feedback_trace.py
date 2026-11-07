import io
import json
import unittest
from types import SimpleNamespace as S
from follower_feedback_trace import FeedbackTrace


class Tests(unittest.TestCase):
    def core(self,p1=-10,p2=10,count=1):
        return S(valid_count=count,clock=lambda:1.,feedback=dict(m1_position=p1,m2_position=p2,
                    m1_rpm=0,m2_rpm=0,m1_status=0,m2_status=0),
                 odom_x=0.,odom_y=0.,odom_yaw=0.,integration_skips=0)

    def test_new_measurements_only_and_delta(self):
        stream=io.StringIO();t=FeedbackTrace(stream);c=self.core()
        self.assertTrue(t.record(c,'WAIT',100));self.assertFalse(t.record(c,'WAIT',101))
        self.assertTrue(t.record(self.core(-25,26,2),'DONE',102))
        rows=[json.loads(s) for s in stream.getvalue().splitlines()]
        self.assertEqual(len(rows),2);self.assertEqual(t.delta,[-15,16])
        self.assertEqual(rows[-1]['position_unit'],'UNKNOWN')
        self.assertEqual(rows[-1]['m1_rpm'],0)

    def test_signed_wrap(self):
        t=FeedbackTrace(io.StringIO());t.record(self.core(2**31-1,-2**31),'WAIT',100)
        t.record(self.core(-2**31,2**31-1,2),'DONE',101)
        self.assertEqual(t.delta,[1,-1])

    def test_missing_not_fabricated(self):
        t=FeedbackTrace(io.StringIO());c=self.core(count=0);c.feedback=None
        self.assertFalse(t.record(c,'WAIT',100));self.assertIsNone(t.previous)


if __name__=='__main__':unittest.main()
