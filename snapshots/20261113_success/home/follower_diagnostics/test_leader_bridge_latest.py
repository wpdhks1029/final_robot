import ast
import math
import sys
from pathlib import Path
from types import SimpleNamespace as S
import unittest
from unittest.mock import Mock
sys.path.insert(0,str(Path.home()/'robot_ws/test_tools'))
import test_follower_input_safety as f

ROOT=Path(__file__).parent

def load():
    tree=ast.parse((ROOT/'leader_bridge_latest_candidate.py').read_text())
    parts=[x for x in tree.body if isinstance(x,ast.ClassDef) or isinstance(x,ast.FunctionDef) and x.name=='latest_state_qos']
    class Node(f.Node):
        def __init__(self,*a):super().__init__(*a);self.publishers=[];self.timers=[]
        def create_timer(self,*a):self.timers.append(a)
        def create_publisher(self,*a):
            p=Mock();self.publishers.append((a,p));return p
    env=dict(Node=Node,math=math,Buffer=Mock,TransformListener=Mock(),TransformException=f.MissingTF,
             Odometry=f.odom,Path=lambda:S(header=f.header(),poses=[]),
             PoseStamped=lambda:S(header=f.header(),pose=f.odom().pose.pose),
             PoseWithCovarianceStamped=f.odom,
             Duration=lambda **kw:S(**kw),rclpy=S(time=S(Time=lambda:None)),
             QoSProfile=lambda **kw:S(**kw),ReliabilityPolicy=S(BEST_EFFORT='BEST_EFFORT'),
             DurabilityPolicy=S(VOLATILE='VOLATILE'),HistoryPolicy=S(KEEP_LAST='KEEP_LAST'))
    exec(compile(ast.Module(body=parts,type_ignores=[]),'candidate','exec'),env)
    n=env['RealLeaderBridge']();return n,env

def tf(sec=100,ns=0,x=1.):
    return S(header=S(stamp=f.stamp(sec,ns)),transform=S(translation=S(x=x,y=0.,z=0.),rotation=S(x=0.,y=0.,z=0.,w=1.)))

class Tests(unittest.TestCase):
    def test_default_rate_halves_period_and_preserves_timeouts(self):
        n,e=load()
        self.assertEqual(n.publish_rate,20.)
        self.assertEqual(n.timers[0][0],.05)
        self.assertEqual(n.tf_timeout,.5);self.assertEqual(n.odom_timeout,.5)

    def setup_node(self):
        n,e=load();m=f.odom(frame='odom');m.twist.twist.linear.x=.04;n.filtered_odom_callback(m)
        n.tf_buffer.lookup_transform.return_value=tf();return n,e
    def test_initial_pose_clears_history_without_changing_motion_input(self):
        n,e=self.setup_node();n.publish_leader_state()
        original=n.latest_filtered_odom
        n.initial_pose_callback(S(header=S(frame_id='map')))
        self.assertEqual(n.path.poses,[]);self.assertIsNone(n.last_observed_xy)
        self.assertIs(n.latest_filtered_odom,original)
        n.publish_leader_state();self.assertEqual(len(n.path.poses),1)

    def test_wrong_initial_pose_frame_does_not_clear(self):
        n,e=self.setup_node();n.publish_leader_state()
        n.initial_pose_callback(S(header=S(frame_id='other')))
        self.assertEqual(len(n.path.poses),1)

    def test_large_pose_jump_starts_new_path(self):
        n,e=self.setup_node();n.publish_leader_state()
        n.tf_buffer.lookup_transform.return_value=tf(x=1.2);n.publish_leader_state()
        self.assertEqual(len(n.path.poses),2)
        n.tf_buffer.lookup_transform.return_value=tf(x=2.0);n.publish_leader_state()
        self.assertEqual(len(n.path.poses),1)
        self.assertEqual(n.path.poses[0].pose.position.x,2.)
        n.tf_buffer.lookup_transform.return_value=tf(x=2.11);n.publish_leader_state()
        self.assertEqual(len(n.path.poses),2)

    def test_latest_input_output_and_dynamic_tf_qos(self):
        n,e=self.setup_node()
        self.assertEqual(n.subscriptions[0][3].depth,1)
        for args,p in n.publishers:self.assertEqual(args[2].reliability,'BEST_EFFORT');self.assertEqual(args[2].depth,1)
        call=e['TransformListener'].call_args
        self.assertEqual(call.kwargs['qos'].depth,10);self.assertNotIn('static_qos',call.kwargs)
    def test_older_twist_stamp_preserved_for_both_outputs(self):
        n,e=self.setup_node();n.tf_buffer.lookup_transform.return_value=tf(ns=50_000_000)
        n.publish_leader_state()
        odom=n.leader_odom_pub.publish.call_args.args[0]
        path=n.leader_path_pub.publish.call_args.args[0]
        self.assertEqual(odom.header.stamp.nanosec,0);self.assertEqual(path.header.stamp.nanosec,0)
        self.assertEqual(odom.twist.twist.linear.x,.04)
        self.assertEqual(n.tf_buffer.lookup_transform.call_args.kwargs['timeout'].seconds,0.)
    def test_older_tf_stamp_preserved(self):
        n,e=self.setup_node();n.latest_filtered_odom.header.stamp.nanosec=50_000_000
        n.publish_leader_state();self.assertEqual(n.leader_odom_pub.publish.call_args.args[0].header.stamp.nanosec,0)
    def test_missing_tf_does_not_publish(self):
        n,e=self.setup_node();n.tf_buffer.lookup_transform.side_effect=f.MissingTF('missing')
        n.publish_leader_state();n.leader_odom_pub.publish.assert_not_called();n.leader_path_pub.publish.assert_not_called()
    def test_stale_future_missing_stamps_pause_outputs(self):
        for field in ('tf','odom'):
            for sec in (0,98,101):
                n,e=self.setup_node()
                if field=='tf':n.tf_buffer.lookup_transform.return_value=tf(sec=sec)
                else:n.latest_filtered_odom.header.stamp.sec=sec
                n.publish_leader_state();n.leader_odom_pub.publish.assert_not_called()
    def test_path_spacing_and_limit_unchanged(self):
        n,e=self.setup_node();n.max_path_points=2
        for x,count in [(0.,1),(.05,1),(.11,2),(.22,2)]:
            n.tf_buffer.lookup_transform.return_value=tf(x=x);n.publish_leader_state()
            self.assertEqual(len(n.path.poses),count)
        self.assertEqual(n.path.poses[-1].pose.position.x,.22)

if __name__=='__main__':unittest.main()
