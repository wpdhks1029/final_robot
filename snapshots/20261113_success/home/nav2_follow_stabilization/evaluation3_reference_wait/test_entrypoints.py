"""Run bash entrypoints with a fake Python command and fake HOME; no ROS."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
ROOT=Path(__file__).resolve().parent


class EntryTests(unittest.TestCase):
    def setup_folder(self,d):
        root=Path(d);envdir=root/'nav2_follow_integration/baseline/follower_diagnostics';envdir.mkdir(parents=True)
        (envdir/'integration_async_env.bash').write_text('export ROS_DOMAIN_ID=10\n')
        bindir=root/'bin';bindir.mkdir();py=bindir/'python3'
        py.write_text('#!/usr/bin/env bash\nprintf "%s\\0" "$@" > "$EVAL3_CAPTURE_ARGS"\n');py.chmod(0o755)
        dest=root/'evaluation3';dest.mkdir()
        for name in ('start_evaluation3.bash','start_follower_driver.bash','capture_reference.bash'):shutil.copy2(ROOT/name,dest/name)
        env={**os.environ,'HOME':d,'PATH':str(bindir)+':'+os.environ['PATH'],'EVAL3_CAPTURE_ARGS':str(root/'args')}
        return root,dest,env
    def args(self,root):return (root/'args').read_bytes().decode().strip('\0').split('\0')
    def test_requires_separate_evaluation3_reference_before_motion(self):
        with tempfile.TemporaryDirectory() as d:
            root,dest,env=self.setup_folder(d)
            result=subprocess.run(['bash',str(dest/'start_evaluation3.bash')],env=env,capture_output=True)
            self.assertEqual(result.returncode,2);self.assertFalse((root/'args').exists())
    def test_start_selects_private_tf_and_preserves_continuous_profile(self):
        with tempfile.TemporaryDirectory() as d:
            root,dest,env=self.setup_folder(d);ref=root/'new_reference.json';ref.write_text('{}')
            (dest/'latest_reference_path.txt').write_text(str(ref)+'\n')
            subprocess.run(['bash',str(dest/'start_evaluation3.bash')],env=env,check=True)
            a=self.args(root)
            self.assertIn('--continuous20',a);self.assertIn('--dynamic-box',a)
            self.assertEqual(a[a.index('--tf-static-topic')+1],'/follower/tf_static_internal')
            self.assertEqual(a[a.index('--seconds')+1],'0')
            self.assertEqual(a[a.index('--reference-plan')+1],str(ref))
            self.assertNotIn('--disable-clearance-stop',a)
            self.assertIn('--reference-wait',a)
    def test_driver_uses_current_fault_guards_and_ten_rpm_mode(self):
        with tempfile.TemporaryDirectory() as d:
            root,dest,env=self.setup_folder(d)
            subprocess.run(['bash',str(dest/'start_follower_driver.bash')],env=env,check=True)
            a=self.args(root)
            self.assertIn('/driver/follower_drive_node.py',a[1])
            self.assertEqual(a[a.index('--trial-max-rpm')+1],'10')
            self.assertEqual(a[a.index('--trial-seconds')+1],'0')
            self.assertIn('--ack-continuous20-candidate',a)
            self.assertNotIn('--disable-clearance-stop',a)
    def test_capture_invokes_readonly_tool_and_separate_output(self):
        with tempfile.TemporaryDirectory() as d:
            root,dest,env=self.setup_folder(d)
            subprocess.run(['bash',str(dest/'capture_reference.bash')],env=env,check=True)
            a=self.args(root)
            self.assertTrue(a[1].endswith('/capture_shared_map_reference.py'))
            self.assertIn('--output',a);self.assertNotIn('--run',a)
            self.assertTrue(a[a.index('--output')+1].startswith(str(dest/'references')))


if __name__=='__main__':unittest.main()
