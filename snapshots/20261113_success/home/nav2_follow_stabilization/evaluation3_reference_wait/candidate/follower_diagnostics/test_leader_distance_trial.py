import ast,unittest
from pathlib import Path

class Tests(unittest.TestCase):
    def test_only_forward_leader_publisher_no_follower_or_arm(self):
        tree=ast.parse(Path(__file__).with_name('run_leader_distance_trial.py').read_text())
        calls=[n for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute)]
        pubs=[n for n in calls if n.func.attr=='create_publisher']
        self.assertEqual(len(pubs),1);self.assertEqual(pubs[0].args[1].value,'/cmd_vel')
        self.assertFalse(any(n.func.attr=='create_client' for n in calls))
        send=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='send')
        attrs=[n.attr for n in ast.walk(send) if isinstance(n,ast.Attribute)]
        self.assertIn('linear',attrs);self.assertNotIn('angular',attrs)
    def test_measured_forward_scale_preserves_yaw_and_command_geometry(self):
        tree=ast.parse(Path(__file__).with_name('leader_nav_yaw_calibrated.launch.py').read_text())
        values={}
        for n in ast.walk(tree):
            if isinstance(n,ast.Dict):
                for k,v in zip(n.keys,n.values):
                    if isinstance(k,ast.Constant) and isinstance(v,ast.Constant):values[k.value]=v.value
        self.assertEqual(values['feedback_yaw_scale'],1.18)
        self.assertEqual(values['feedback_speed_scale'],1.173)
        self.assertEqual(values['wheel_radius'],.085);self.assertEqual(values['wheel_track'],.560)
        self.assertEqual(values['max_rpm'],25)

if __name__=='__main__':unittest.main()
