import unittest
from types import SimpleNamespace as S
from unittest.mock import Mock
from follower_comparison_observer import ComparisonObserver,summary


class Tests(unittest.TestCase):
    def test_summary(self):
        rows=[dict(rx_s=1.,x=0.,y=0.),dict(rx_s=1.1,x=.1,y=0.)]
        self.assertIn('delta_xy=(0.100000,0.000000)',summary(rows))
        self.assertEqual(summary([]),'NO DATA')

    def test_finished_capture_does_not_record(self):
        o=ComparisonObserver.__new__(ComparisonObserver);o.finished=True
        o.writer=Mock();o.receive('topic',object());o.writer.writerow.assert_not_called()

    def test_finish_once_closes_only_record_not_node(self):
        o=ComparisonObserver.__new__(ComparisonObserver)
        o.finished=False;o.rows={};o.path='fake.csv';stream=Mock();o.stream=stream;o.node=Mock()
        o.finish_capture();o.finish_capture()
        stream.close.assert_called_once();o.node.destroy_node.assert_not_called()

    def test_raw_received_fields(self):
        o=ComparisonObserver.__new__(ComparisonObserver);o.finished=False;o.start=0.
        o.rows={'raw':[]};o.writer=Mock();o.stream=Mock()
        m=S(header=S(stamp=S(sec=1,nanosec=2),frame_id='odom'),child_frame_id='base',
            pose=S(pose=S(position=S(x=.1,y=.2),orientation=S(x=0.,y=0.,z=0.,w=1.))),
            twist=S(twist=S(linear=S(x=.01),angular=S(z=0.))))
        o.receive('raw',m)
        self.assertEqual(o.rows['raw'][0]['stamp_ns'],1000000002)
        self.assertEqual(o.rows['raw'][0]['vx'],.01)


if __name__=='__main__':unittest.main()
