"""No ROS/hardware: verified source hashes, launch assembly and wrapper routing."""
import ast
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
from types import SimpleNamespace as S
import unittest
from unittest.mock import patch
ROOT=Path(__file__).resolve().parent


class RecoveryTests(unittest.TestCase):
    def test_bridge_is_exact_github_success_blob(self):
        manifest=json.loads((ROOT/'github_tree_manifest.json').read_text())
        for name in ('real_follower_bridge_tf_candidate.py','follower_input_guard.py'):
            entry=next(e for e in manifest['entries'] if e['path']=='target_3/new_map_20261107/robot_ws/src/cnu_capstone_2026/convoy_robot/'+name)
            data=(ROOT/name).read_bytes()
            self.assertEqual(hashlib.sha1(b'blob '+str(len(data)).encode()+b'\0'+data).hexdigest(),entry['sha'])
    def test_stack_keeps_all_three_tf_consumers_on_original_topology(self):
        def include(source,launch_arguments):return S(kind='include',source=source,args=dict(launch_arguments))
        modules={'launch':S(LaunchDescription=lambda entities:entities),
                 'launch.actions':S(DeclareLaunchArgument=lambda *a,**k:S(kind='arg'),IncludeLaunchDescription=include),
                 'launch.launch_description_sources':S(PythonLaunchDescriptionSource=lambda x:x),
                 'launch.substitutions':S(LaunchConfiguration=lambda x:('config',x)),
                 'launch_ros.actions':S(Node=lambda **k:S(kind='node',options=k))}
        with patch.dict(sys.modules,modules):
            spec=importlib.util.spec_from_file_location('stack',ROOT/'follower_success_tf_stack.launch.py')
            m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);entities=m.generate_launch_description()
        includes=[x for x in entities if x.kind=='include'];nodes=[x.options for x in entities if x.kind=='node']
        self.assertEqual(len(includes),2)
        self.assertTrue(includes[0].source.endswith('/follower_diagnostics/follower_position_fixed_lidar.launch.py'))
        self.assertTrue(includes[1].source.endswith('/follower_diagnostics/follower_localization.launch.py'))
        self.assertEqual(includes[1].args['enable_localization'],'true')
        self.assertEqual(len(nodes),1);self.assertEqual(nodes[0]['name'],'real_follower_bridge')
        self.assertNotIn('remappings',nodes[0]);self.assertNotIn('parameters',nodes[0])
    def test_success_bridge_still_requires_fresh_ekf_measurement_and_reception(self):
        tree=ast.parse((ROOT/'real_follower_bridge_tf_candidate.py').read_text())
        cls=next(n for n in tree.body if isinstance(n,ast.ClassDef))
        fn=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='ekf_is_fresh')
        env={'time':S(monotonic=lambda:10.)}
        exec(compile(ast.fix_missing_locations(ast.Module(body=[fn],type_ignores=[])),'bridge-test','exec'),env)
        for age,rx,expected in ((.1,.1,True),(.6,.1,False),(.1,.6,False),(-.1,.1,False)):
            now=100_000_000_000;stamp=now-round(age*1e9)
            o=S(latest_filtered_odom=S(header=S(stamp=S(sec=stamp//10**9,nanosec=stamp%10**9))),last_ekf_rx=10.-rx,ekf_timeout=.5,get_clock=lambda:S(now=lambda:S(nanoseconds=now)))
            self.assertEqual(env['ekf_is_fresh'](o),expected)
    def test_general_follow_default_private_and_recovery_public_static_topic(self):
        for recovery,topic in ((False,'/follower/tf_static_internal'),(True,'/tf_static')):
            with tempfile.TemporaryDirectory() as d:
                home=Path(d);envdir=home/'nav2_follow_integration/baseline/follower_diagnostics';envdir.mkdir(parents=True)
                (envdir/'integration_async_env.bash').write_text('export ROS_DOMAIN_ID=10\n')
                target=home/'nav2_follow_stabilization/smooth010';target.mkdir(parents=True)
                shutil.copy2('/home/sunuk/nav2_follow_stabilization/smooth010/start_follow.bash',target/'start_follow.bash')
                bin=home/'bin';bin.mkdir();py=bin/'python3';py.write_text('#!/usr/bin/env bash\nprintf "%s\\0" "$@" > "$CAPTURE_ARGS"\n');py.chmod(0o755)
                env={**os.environ,'HOME':d,'PATH':str(bin)+':'+os.environ['PATH'],'CAPTURE_ARGS':str(home/'args')};env.pop('CONVOY_TF_STATIC_TOPIC',None)
                command=ROOT/'start_follow.bash' if recovery else target/'start_follow.bash'
                subprocess.run(['bash',str(command),'.1' if False else '0.1'],env=env,check=True)
                args=(home/'args').read_bytes().decode().strip('\0').split('\0')
                self.assertEqual(args[args.index('--tf-static-topic')+1],topic)
                self.assertEqual(args[args.index('--follow-max-linear')+1],'0.1')
                self.assertEqual(args[args.index('--seconds')+1],'0')


if __name__=='__main__':unittest.main()
