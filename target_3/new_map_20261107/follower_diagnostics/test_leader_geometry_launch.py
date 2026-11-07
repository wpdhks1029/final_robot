import ast
from pathlib import Path
from types import SimpleNamespace as S
import unittest
import os
import sys

ROOT=Path(__file__).parent

class Tests(unittest.TestCase):
    def launch(self):
        file=ROOT/'leader_nav_geometry_candidate.launch.py';tree=ast.parse(file.read_text())
        method=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='generate_launch_description')
        ns=dict(__file__=str(file),os=os,sys=sys,LaunchDescription=lambda x:x,
                DeclareLaunchArgument=lambda name,**kw:('arg',name,kw),
                IncludeLaunchDescription=lambda x:('include',x),PythonLaunchDescriptionSource=lambda p:p,
                LaunchConfiguration=lambda n:('config',n),ParameterValue=lambda value,**kw:('value',value,kw),
                Node=lambda **kw:kw)
        exec(compile(ast.Module(body=[method],type_ignores=[]),str(file),'exec'),ns)
        return ns['generate_launch_description']()
    def test_geometry_and_motion_gate_preserve_motor_limits(self):
        actions=self.launch();self.assertEqual(actions[0],('arg','allow_motion',{'default_value':'false'}))
        controller=next(x for x in actions if isinstance(x,dict) and x['name']=='md200t_controller')
        p=controller['parameters'][0]
        self.assertEqual(p['wheel_track'],.560);self.assertEqual(p['wheel_radius'],.085)
        self.assertEqual(p['feedback_speed_scale'],1.18);self.assertEqual(p['feedback_yaw_scale'],1.)
        self.assertEqual(p['wheel_yaw_rate_variance'],.0009)
        self.assertEqual(p['max_rpm'],25);self.assertEqual(p['baudrate'],57600)
        self.assertEqual(p['cmd_vel_timeout'],.5)
        self.assertTrue(controller['arguments'][0].endswith('leader_md200t_polling_split_scale_candidate.py'))
    def test_separate_ekf_file_uses_only_wheel_rates(self):
        import yaml
        ekf=next(x for x in self.launch() if isinstance(x,dict) and x['name']=='ekf_filter_node')
        config=Path(ekf['parameters'][0]);self.assertEqual(config.name,'leader_ekf_wheel_yaw_candidate.yaml')
        p=yaml.safe_load(config.read_text())['ekf_filter_node']['ros__parameters']
        self.assertEqual([i for i,x in enumerate(p['odom0_config']) if x],[6,7,11])
        self.assertFalse('imu0' in p)

if __name__=='__main__':unittest.main()
