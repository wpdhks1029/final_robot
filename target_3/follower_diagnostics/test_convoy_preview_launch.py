import ast
from pathlib import Path
import sys
from types import SimpleNamespace as S
import unittest
from unittest.mock import Mock

ROOT=Path(__file__).parent


class Tests(unittest.TestCase):
    def start(self):
        tree=ast.parse((ROOT/'follower_convoy_preview.launch.py').read_text())
        fn=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='start')
        check=Mock();env=dict(ROOT=ROOT,sys=sys,preflight=check,
            LaunchConfiguration=lambda k:S(perform=lambda c:c[k]),LogInfo=lambda **k:k,
            ExecuteProcess=lambda **k:k,RegisterEventHandler=lambda e:e,
            OnProcessExit=lambda **k:k,EmitEvent=lambda **k:k,Shutdown=lambda **k:k)
        exec(compile(ast.Module(body=[fn],type_ignores=[]),'<preview-launch>','exec'),env)
        return env['start'],check

    def test_default_disabled(self):
        start,check=self.start();result=start({'enable_preview':'false'})
        check.assert_not_called();self.assertEqual(len(result),1)

    def test_only_two_preview_scripts(self):
        start,check=self.start();result=start({'enable_preview':'true'})
        check.assert_called_once()
        processes=[a for a in result if 'cmd' in a]
        self.assertEqual(len(processes),2)
        self.assertTrue(processes[0]['cmd'][-1].endswith('follower_pursuit_preview.py'))
        self.assertTrue(processes[1]['cmd'][-1].endswith('follower_obstacle_preview.py'))
        self.assertTrue(all('motor' not in a['cmd'][-1] for a in processes))

    def test_preflight_failure_starts_nothing(self):
        start,check=self.start();check.side_effect=RuntimeError('duplicate')
        with self.assertRaises(RuntimeError):start({'enable_preview':'true'})


if __name__=='__main__':unittest.main()
