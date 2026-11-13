import unittest
from types import SimpleNamespace as S

from local_trial_preflight import graph_issues,laser_mount_issue,mount_snapshot


class FakeGraph:
    def __init__(self,publishers,subscriptions,gids=None):
        self.publishers=publishers
        self.subscriptions=subscriptions
        self.gids=gids or {}
    def get_publishers_info_by_topic(self,topic):
        return [S(node_name=n,endpoint_gid=self.gids.get((topic,'publisher',i)))
                for i,n in enumerate(self.publishers.get(topic,[]))]
    def get_subscriptions_info_by_topic(self,topic):
        return [S(node_name=n,endpoint_gid=self.gids.get((topic,'subscription',i)))
                for i,n in enumerate(self.subscriptions.get(topic,[]))]
    def get_name(self):
        return 'local_avoidance_trial_operator'


class TrialPreflightTests(unittest.TestCase):
    def good_graph(self):
        return FakeGraph(
            {
                '/cmd_vel':['nav2_convoy_gate'],
                '/cmd_vel_smoothed':['velocity_smoother'],
                '/follower/motor_candidate/continuous20_command':['local_avoidance_trial_operator'],
                '/follower/odom_raw':['follower_drive_candidate'],
                '/follower/scan':['follower_lidar'],
                '/leader/odom':['real_leader_bridge'],
                '/follower/motor_candidate/status':['follower_drive_candidate'],
                '/follower/odometry/filtered':['follower_ekf_filter_node'],
                '/follower/imu/data':['follower_imu'],
            },
            {
                '/cmd_vel':['md200t_controller'],
                '/cmd_vel_smoothed':['nav2_convoy_gate'],
                '/follower/motor_candidate/continuous20_command':['follower_drive_candidate'],
            })

    def check(self,graph):
        return graph_issues(graph,nav2_leader=True,
            command_topic='/follower/motor_candidate/continuous20_command',
            node_name=graph.get_name(),plan_only=False,pose_source='filtered')

    def test_matching_hardware_graph_passes(self):
        self.assertEqual(self.check(self.good_graph()),[])

    def test_direct_follow_requires_local_relay_topics(self):
        graph=self.good_graph()
        graph._use_direct_unicast=True
        graph._require_nav2_goal_status=True
        graph.publishers.pop('/leader/odom')
        graph.publishers.update({
            '/leader/odom_unicast':['follower_unicast_relay'],
            '/leader/path_unicast':['follower_unicast_relay'],
            '/leader/nav2_goal_status_unicast':['follower_unicast_relay'],
            '/follower/odom_map':['real_follower_bridge'],
        })
        def check():
            return graph_issues(graph,nav2_leader=True,
                command_topic='/follower/motor_candidate/continuous20_command',
                node_name=graph.get_name(),plan_only=False,pose_source='filtered',follow_only=True)
        self.assertEqual(check(),[])
        graph.publishers.pop('/leader/odom_unicast')
        self.assertTrue(any(i['topic']=='/leader/odom_unicast' for i in check()))

    def test_measured_path_follow_does_not_require_controller_private_plan(self):
        graph=self.good_graph()
        graph._require_nav2_plan=False
        self.assertEqual(self.check(graph),[])
        graph.publishers['/received_global_plan']=['controller_server']
        self.assertEqual(self.check(graph),[])

    def test_auto_goal_mode_requires_navigate_to_pose_status(self):
        graph=self.good_graph()
        graph._require_nav2_goal_status=True
        graph.publishers['/navigate_to_pose/_action/status']=['bt_navigator']
        self.assertEqual(self.check(graph),[])
        graph.publishers.pop('/navigate_to_pose/_action/status')
        self.assertTrue(any(item['topic']=='/navigate_to_pose/_action/status'
                            for item in self.check(graph)))

    def test_pure_follow_requires_map_pose_but_not_filtered_odom_or_imu(self):
        graph=FakeGraph(
            {'/cmd_vel':['nav2_convoy_gate'],
             '/cmd_vel_smoothed':['velocity_smoother'],
             '/received_global_plan':['controller_server'],
             '/follower/motor_candidate/local_avoidance_command':['local_avoidance_trial_operator'],
             '/follower/odom_map':['real_follower_bridge'],
             '/follower/scan':['follower_lidar'],
             '/leader/odom':['real_leader_bridge'],
             '/leader/path':['real_leader_bridge'],
             '/follower/motor_candidate/status':['follower_drive_candidate']},
            {'/cmd_vel':['md200t_controller'],
             '/cmd_vel_smoothed':['nav2_convoy_gate'],
             '/follower/motor_candidate/local_avoidance_command':['follower_drive_candidate']})
        issues=graph_issues(graph,nav2_leader=True,
            command_topic='/follower/motor_candidate/local_avoidance_command',
            node_name=graph.get_name(),plan_only=False,pose_source='filtered',
            follow_only=True)
        self.assertEqual(issues,[])

    def test_reports_wrong_or_duplicate_cmd_vel_owner(self):
        graph=self.good_graph()
        graph.publishers['/cmd_vel']=['old_nav2','nav2_convoy_gate']
        issues=self.check(graph)
        self.assertIn({
            'topic':'/cmd_vel','endpoint':'publisher',
            'expected_nodes':['nav2_convoy_gate'],
            'actual_nodes':['nav2_convoy_gate','old_nav2']},issues)

    def test_reports_missing_filtered_imu(self):
        graph=self.good_graph()
        del graph.publishers['/follower/imu/data']
        issues=self.check(graph)
        self.assertTrue(any(x['topic']=='/follower/imu/data' for x in issues))

    def test_resolves_unknown_remote_gate_label_by_participant_gid(self):
        graph=self.good_graph()
        participant=[1,2,3,4,5,6,7,8,0,0,0,0,0,0]
        gate_pub=participant+[0x11,0x03]+[0]*8
        gate_sub=participant+[0x12,0x04]+[0]*8
        leader_sub=participant+[0x15,0x04]+[0]*8
        graph.publishers['/cmd_vel']=['_NODE_NAME_UNKNOWN_']
        graph.subscriptions['/cmd_vel_smoothed']=['_NODE_NAME_UNKNOWN_']
        graph.gids={
            ('/cmd_vel','publisher',0):gate_pub,
            ('/cmd_vel_smoothed','subscription',0):gate_sub,
            ('/leader/odom','subscription',0):leader_sub,
        }
        graph.subscriptions['/leader/odom']=['nav2_convoy_gate','rviz']
        self.assertEqual(self.check(graph),[])

    def test_unknown_gate_label_from_other_participant_still_fails(self):
        graph=self.good_graph()
        participant=[1,2,3,4,5,6,7,8,0,0,0,0,0,0]
        other=[9,9,9,9,9,9,9,9,0,0,0,0,0,0]
        graph.publishers['/cmd_vel']=['_NODE_NAME_UNKNOWN_']
        graph.gids={
            ('/cmd_vel','publisher',0):other+[0x11,0x03]+[0]*8,
            ('/leader/odom','subscription',0):participant+[0x15,0x04]+[0]*8,
        }
        graph.subscriptions['/leader/odom']=['nav2_convoy_gate']
        issues=self.check(graph)
        self.assertTrue(any(x['topic']=='/cmd_vel' for x in issues))

    def test_stopped_or_unseen_leader_graph_fails_closed(self):
        graph=self.good_graph()
        for topic in ('/cmd_vel','/cmd_vel_smoothed','/leader/odom'):
            graph.publishers.pop(topic,None)
        graph.subscriptions.pop('/cmd_vel_smoothed',None)
        graph.subscriptions.pop('/leader/odom',None)
        issues=self.check(graph)
        failed_topics={item['topic'] for item in issues}
        self.assertTrue({'/cmd_vel','/cmd_vel_smoothed','/leader/odom'} <= failed_topics)

    def test_laser_mount_validation_and_snapshot(self):
        transform=S(
            header=S(frame_id='follower/base_link'),child_frame_id='follower/laser',
            transform=S(translation=S(x=.26,y=0.,z=.24),
                        rotation=S(x=0.,y=0.,z=0.,w=1.)))
        self.assertIsNone(laser_mount_issue(transform))
        self.assertEqual(mount_snapshot(transform)['translation_m'],[.26,0.,.24])
        transform.transform.translation.x=.30
        self.assertEqual(laser_mount_issue(transform),'LASER_MOUNT_VALUE_MISMATCH')
        self.assertEqual(laser_mount_issue(None),'LASER_MOUNT_TF_MISSING')


if __name__=='__main__':
    unittest.main()
