import unittest
from unittest.mock import patch
from pathlib import Path
import tempfile
import os
from follower_position_preview import PositionPreview,latest_trace


def row(i,l=0,r=0):
    return dict(elapsed=i*.1,feedback_count=i+1,m1_position=l,m2_position=r,m1_status=0,m2_status=0)


class Tests(unittest.TestCase):
    def test_latest_trace_selects_once_and_ignores_other_files(self):
        with tempfile.TemporaryDirectory() as directory:
            a=Path(directory)/'measurement_feedback_1.jsonl';a.touch()
            b=Path(directory)/'measurement_feedback_2.jsonl';b.touch()
            os.utime(a,ns=(100,100));os.utime(b,ns=(200,200))
            (Path(directory)/'unrelated.jsonl').touch()
            self.assertEqual(latest_trace(directory),b)

    def test_missing_trace_refused(self):
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaises(ValueError):latest_trace(directory)

    def test_forward_and_window(self):
        c=PositionPreview()
        for i in range(11):c.update(row(i,-i,i))
        self.assertGreater(c.x,.06);self.assertAlmostEqual(c.y,0)
        self.assertGreater(c.v,0);self.assertAlmostEqual(c.w,0)

    def test_reverse_position_not_masked(self):
        c=PositionPreview()
        for i in range(11):c.update(row(i,i,-i))
        self.assertLess(c.v,0);self.assertLess(c.x,0)

    def test_stationary_jitter_visible_but_not_rectified(self):
        c=PositionPreview()
        for i in range(21):c.update(row(i,-(i%2),i%2))
        self.assertAlmostEqual(c.x,0.,places=5)

    def test_no_output_until_window_ready(self):
        c=PositionPreview()
        for i in range(4):self.assertFalse(c.update(row(i)))
        self.assertEqual(c.accepted,0)

    def test_gap_rebaseline_and_warmup(self):
        c=PositionPreview()
        for i in range(10):c.update(row(i))
        self.assertFalse(c.update(row(20,-10,10)))
        self.assertEqual(c.gaps,1);self.assertEqual(c.x,0)
        self.assertFalse(c.update(row(21,-10,10)))

    def test_replay_refused(self):
        c=PositionPreview();c.update(row(1))
        with self.assertRaises(ValueError):c.update(row(1))

    def test_status_refused(self):
        c=PositionPreview();r=row(1);r['m1_status']=1
        with self.assertRaises(ValueError):c.update(r)


if __name__=='__main__':unittest.main()
