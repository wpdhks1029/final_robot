import importlib.util
from pathlib import Path
from unittest.mock import patch,Mock
import unittest
from launch import LaunchContext

ROOT=Path(__file__).parent
spec=importlib.util.spec_from_file_location('fixed_launch',ROOT/'follower_position_fixed_lidar.launch.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)

def context(enabled):
    c=LaunchContext();c.launch_configurations.update(enable_position_profile=enabled,ekf_frequency='0.0');return c

class Tests(unittest.TestCase):
    def test_disabled_never_preflights_or_starts(self):
        with patch('position_profile_contract.preflight') as pre,patch.object(m,'Node') as node:
            self.assertEqual(len(m.start(context('false'))),1);pre.assert_not_called();node.assert_not_called()
    def test_invalid_enable_rejected(self):
        with self.assertRaises(RuntimeError):m.start(context('yes'))
    def test_only_candidate_lidar_and_same_sensor_profile(self):
        with patch('position_profile_contract.preflight') as pre,patch.object(m,'Node') as node,patch.object(m,'IncludeLaunchDescription') as include:
            self.assertEqual(len(m.start(context('true'))),2);pre.assert_called_once()
            args=dict(include.call_args.kwargs['launch_arguments'])
            self.assertEqual(args['enable_lidar'],'false');self.assertEqual(args['enable_ekf'],'true');self.assertEqual(args['enable_imu'],'true');self.assertEqual(args['enable_feedback'],'false')
            kw=node.call_args.kwargs;self.assertEqual(kw['parameters'][1],{'auto_intensity':False});self.assertEqual(kw['name'],'follower_lidar')
            self.assertIn(('scan','/follower/scan'),kw['remappings'])
    def test_failed_preflight_starts_nothing(self):
        with patch('position_profile_contract.preflight',side_effect=RuntimeError('duplicate EKF')),patch.object(m,'Node') as node:
            with self.assertRaises(RuntimeError):m.start(context('true'))
            node.assert_not_called()

if __name__=='__main__':unittest.main()
