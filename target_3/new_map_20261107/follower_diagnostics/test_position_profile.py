import ast
import importlib.util
import math
from pathlib import Path
import sys
from types import SimpleNamespace as S
import unittest
from unittest.mock import MagicMock,Mock,patch

from position_profile_contract import validate
from follower_position_preview import PositionCandidatePublisher
from follower_motor_locked import main as locked_main

ROOT=Path(__file__).parent


class Tests(unittest.TestCase):
    def test_contract_good(self):validate(1,0,'follower/odom','follower/base_link',.05)

    def test_contract_rejects_duplicate_stale_wrong_frames(self):
        cases=[(0,0,'follower/odom','follower/base_link',.05),
               (2,0,'follower/odom','follower/base_link',.05),
               (1,1,'follower/odom','follower/base_link',.05),
               (1,0,'preview/odom','follower/base_link',.05),
               (1,0,'follower/odom','preview/base_link',.05),
               (1,0,'follower/odom','follower/base_link',-.01),
               (1,0,'follower/odom','follower/base_link',.31),
               (1,0,'follower/odom','follower/base_link',math.nan)]
        for args in cases:
            with self.subTest(args=args),self.assertRaises(RuntimeError):validate(*args)

    def test_yaml_only_changes_input_topic(self):
        import yaml
        original=Path.home()/'robot_ws/src/cnu_capstone_2026/config/follower_ekf_A_selected.yaml'
        a=yaml.safe_load(original.read_text())['follower_ekf_filter_node']['ros__parameters']
        b=yaml.safe_load((ROOT/'follower_ekf_position_candidate.yaml').read_text())['follower_ekf_filter_node']['ros__parameters']
        self.assertEqual(b['odom0'],'/follower/odom_position_candidate')
        b['odom0']=a['odom0'];self.assertEqual(a,b)

    def test_candidate_native_frames_preserve_test_profile(self):
        n=Mock();n.resolve_topic_name.side_effect=lambda t:t;n.get_publishers_info_by_topic.return_value=[]
        with patch.dict(sys.modules,{'nav_msgs.msg':S(Odometry=MagicMock)}):o=PositionCandidatePublisher(n)
        self.assertEqual(n.create_publisher.call_args.args[1],'/follower/odom_position_candidate')
        c=S(valid_count=0,clock=lambda:0.)
        for i in range(1,8):
            c.valid_count=i;c.clock=lambda i=i:i*.1
            c.feedback=dict(m1_position=0,m2_position=0,m1_status=0,m2_status=0)
            o.publish(c,'stamp')
        msg=o.pub.publish.call_args.args[0]
        self.assertEqual(msg.header.frame_id,'follower/odom');self.assertEqual(msg.child_frame_id,'follower/base_link')

    def test_two_position_profiles_cli_refused(self):
        with patch('sys.argv',['locked','--publish-position-test','--publish-position-candidate']):
            with self.assertRaises(SystemExit):locked_main()

    def start_function(self):
        tree=ast.parse((ROOT/'follower_position_observation.launch.py').read_text())
        function=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='start')
        env=dict(ROOT=ROOT,sys=sys,LaunchConfiguration=lambda key:S(perform=lambda context:context[key]),
                 LogInfo=lambda **kwargs:('log',kwargs),PythonLaunchDescriptionSource=lambda path:path,
                 IncludeLaunchDescription=lambda source,launch_arguments:dict(launch_arguments))
        exec(compile(ast.Module(body=[function],type_ignores=[]),'<launch-test>','exec'),env)
        return env['start']

    def test_launch_default_no_preflight_or_hardware(self):
        check=Mock()
        with patch.dict(sys.modules,{'position_profile_contract':S(preflight=check)}):
            result=self.start_function()({'enable_position_profile':'false'})
        check.assert_not_called();self.assertEqual(result[0][0],'log')

    def test_launch_enable_validates_and_selects_external_position(self):
        check=Mock()
        with patch.dict(sys.modules,{'position_profile_contract':S(preflight=check)}):
            result=self.start_function()({'enable_position_profile':'true','ekf_frequency':'20.0'})
        check.assert_called_once()
        self.assertEqual(result[0]['enable_feedback'],'false')
        self.assertEqual(result[0]['external_feedback_confirmed'],'true')
        self.assertEqual(result[0]['odom_profile'],'position_candidate')
        self.assertEqual(result[0]['ekf_frequency'],'20.0')


if __name__=='__main__':unittest.main()
