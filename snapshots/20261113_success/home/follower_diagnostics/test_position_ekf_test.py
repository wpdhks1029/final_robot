import sys
import unittest
from types import SimpleNamespace as S
from unittest.mock import Mock,MagicMock,patch
from pathlib import Path

from follower_position_preview import PositionPublisher,TOPIC


class Tests(unittest.TestCase):
    def node(self):
        n=Mock();n.resolve_topic_name.side_effect=lambda t:t
        n.get_publishers_info_by_topic.return_value=[]
        return n

    def test_test_only_topic_and_fresh_measurements(self):
        node=self.node()
        with patch.dict(sys.modules,{'nav_msgs.msg':S(Odometry=MagicMock)}):publisher=PositionPublisher(node)
        self.assertEqual(node.create_publisher.call_args.args[1],TOPIC)
        core=S(valid_count=0,clock=lambda:0.)
        self.assertFalse(publisher.publish(core,'stamp'));publisher.pub.publish.assert_not_called()
        for i in range(1,8):
            core.valid_count=i;core.clock=lambda i=i:i*.1
            core.feedback=dict(m1_position=0,m2_position=0,m1_status=0,m2_status=0)
            publisher.publish(core,'stamp')
        self.assertTrue(publisher.pub.publish.called)
        m=publisher.pub.publish.call_args.args[0]
        self.assertEqual(m.header.frame_id,'follower_position_preview/odom')
        self.assertEqual(m.twist.twist.linear.x,0.)

    def test_duplicate_preview_refused(self):
        node=self.node();node.get_publishers_info_by_topic.return_value=[object()]
        with patch.dict(sys.modules,{'nav_msgs.msg':S(Odometry=Mock)}):
            with self.assertRaises(RuntimeError):PositionPublisher(node)

    def test_ekf_test_config_no_dynamic_tf_or_real_frames(self):
        import yaml
        p=yaml.safe_load(Path(__file__).with_name('follower_position_ekf_test.yaml').read_text())['/**']['ros__parameters']
        self.assertIs(p['publish_tf'],False)
        self.assertEqual(p['odom0'],TOPIC)
        self.assertEqual(p['imu0'],'/follower/imu_position_test')
        self.assertEqual(p['base_link_frame'],'follower_position_preview/base_link')


if __name__=='__main__':unittest.main()
