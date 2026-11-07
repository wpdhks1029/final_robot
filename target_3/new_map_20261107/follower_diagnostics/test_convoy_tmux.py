import contextlib,io,shlex,unittest
from unittest.mock import patch
from convoy_tmux import commands,shell_command,main
class Tests(unittest.TestCase):
    def test_preview_never_starts_process(self):
        with patch('sys.argv',['tool','--role','follower']),patch('convoy_tmux.subprocess.run') as run,contextlib.redirect_stdout(io.StringIO()):
            main();run.assert_not_called()
    def test_leader_three_windows(self):self.assertEqual([x[0] for x in commands('leader','core',30,'/home/u')],['base','localization','bridge'])
    def test_follower_core_preserves_fixed_lidar(self):
        c=commands('follower','core',60,'/home/u');self.assertEqual(len(c),4);self.assertIn('follower_position_fixed_lidar.launch.py',' '.join(c[1][1]));self.assertEqual(c[0][1][-1],'60')
    def test_control_two_windows(self):self.assertEqual([x[0] for x in commands('follower','control',30,'/home/u')],['preview','gateway'])
    def test_no_arm_or_trial_operator(self):
        for role,stage in [('leader','core'),('follower','core'),('follower','control')]:
            text=str(commands(role,stage,30,'/home/u'));self.assertNotIn('service call',text);self.assertNotIn('run_joint_follow_trial',text)
    def test_invalid_duration_or_leader_stage(self):
        with self.assertRaises(ValueError):commands('follower','core',120,'/home/u')
        with self.assertRaises(ValueError):commands('leader','control',30,'/home/u')
    def test_shell_preserves_literal_arguments(self):
        argv=['python3','/tmp/a $(no)/app.py','--value','`uname`']
        outer=shlex.split(shell_command(argv,'/home/user name',4));self.assertEqual(outer[:2],['/bin/bash','-c'])
        self.assertEqual(shlex.split(outer[2].split('exec ',1)[1]),argv)
        self.assertIn('ROS_DOMAIN_ID=10',outer[2])
if __name__=='__main__':unittest.main()
