"""Execute the driver's actual settings loader without ROS or serial access."""
import ast
from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace as S
import unittest
ROOT=Path(__file__).resolve().parent
SCRIPT=ROOT/'driver/follower_drive_node.py'


def branches():
    main=next(n for n in ast.parse(SCRIPT.read_text()).body if isinstance(n,ast.FunctionDef) and n.name=='main')
    return [n for n in ast.walk(main) if isinstance(n,ast.If) and ast.unparse(n.test)=='a.run_hardware']


def run_statements(body,env):
    exec(compile(ast.fix_missing_locations(ast.Module(body=body,type_ignores=[])),str(SCRIPT),'exec'),env)


class SettingsTests(unittest.TestCase):
    def test_actual_hardware_loader_and_validation_with_packaged_yaml(self):
        branch=next(n for n in branches() if any(isinstance(x,ast.Import) and any(i.name=='yaml' for i in x.names) for x in n.body))
        statements=[]
        for node in branch.body:
            if isinstance(node,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='port' for t in node.targets):break
            statements.append(node)
        env={'__file__':str(SCRIPT),'Path':Path}
        run_statements(statements,env)
        self.assertEqual(env['cfg']['driver_id'],1)
        self.assertEqual(env['cfg']['baudrate'],19200)
        self.assertNotIn('port',env)  # No port is opened by this test.
    def test_actual_optional_calibration_loader_preserves_existing_scale(self):
        @dataclass
        class Config:raw_linear_feedback_scale:float=1.
        contract=S(core=S(config=Config()))
        branch=next(n for n in branches() if any(isinstance(x,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='calibration' for t in x.targets) for x in n.body))
        run_statements(branch.body,{'__file__':str(SCRIPT),'Path':Path,'json':json,'contract':contract})
        self.assertEqual(contract.core.config.raw_linear_feedback_scale,1.1474)
    def test_packaged_files_match_source_manifest(self):
        for name,row in json.loads((ROOT/'driver_settings_manifest.json').read_text()).items():
            self.assertEqual(hashlib.sha256((ROOT/'driver'/name).read_bytes()).hexdigest(),row['sha256'])
            self.assertEqual((ROOT/'driver'/name).read_bytes(),Path(row['source']).read_bytes())


if __name__=='__main__':unittest.main()
