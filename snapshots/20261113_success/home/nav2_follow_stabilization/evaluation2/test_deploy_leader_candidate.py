import contextlib
import io
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch
import deploy_leader_candidate as deploy


class DeploymentTests(unittest.TestCase):
    def test_local_simulation_of_remote_script_preserves_existing_and_refuses_overwrite(self):
        original_run=subprocess.run
        with tempfile.TemporaryDirectory() as folder:
            home=Path(folder);old=home/'leader_diagnostics/nav2_follow_integration'
            old.mkdir(parents=True);(old/'nav2_convoy_gate.py').write_text('existing integration')
            def simulated_ssh(args,**kwargs):
                self.assertEqual(args[:2],['ssh','sunuk@10.253.245.81'])
                return original_run(['bash','-c',args[2]],env={**os.environ,'HOME':folder},
                    input=kwargs['input'],check=kwargs['check'],capture_output=True)
            with patch('sys.argv',['deploy','--apply']),patch.object(deploy.subprocess,'run',simulated_ssh),contextlib.redirect_stdout(io.StringIO()):
                deploy.main();deploy.main()
                self.assertEqual((old/'nav2_convoy_gate.py').read_text(),'existing integration')
                target=next((home/'leader_diagnostics').glob('eval2_distance_*'))
                self.assertTrue((target/'forward_3m.bash').is_file())
                (target/'run_leader_distance_trial.py').write_text('changed after deployment')
                with self.assertRaises(subprocess.CalledProcessError):deploy.main()
                self.assertEqual((target/'run_leader_distance_trial.py').read_text(),'changed after deployment')
