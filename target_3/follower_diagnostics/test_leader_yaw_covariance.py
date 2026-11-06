import ast
import math
from pathlib import Path
from types import SimpleNamespace as S
import unittest
from unittest.mock import Mock

source=Path(__file__).with_name('leader_md200t_polling_covariance_candidate.py')
tree=ast.parse(source.read_text())
cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='YawRateCovariancePublisher')
ns={'math':math};exec(compile(ast.Module(body=[cls],type_ignores=[]),str(source),'exec'),ns)
Wrapper=ns['YawRateCovariancePublisher']

class Tests(unittest.TestCase):
    def test_zero_is_floored_and_stale_uncertainty_never_reduced(self):
        for before,expected in [(0.,.0009),(.0001,.0009),(.5,.5)]:
            cov=[0.]*36;cov[7]=.0004;cov[35]=before
            msg=S(header=S(stamp=123),twist=S(covariance=cov,twist=S(v=.047,w=.067)))
            pub=Mock();Wrapper(pub,.0009).publish(msg)
            self.assertEqual(msg.twist.covariance[35],expected)
            self.assertEqual(msg.twist.covariance[7],.0004)
            self.assertEqual(msg.header.stamp,123);self.assertEqual(msg.twist.twist.w,.067)
            pub.publish.assert_called_once_with(msg)
    def test_invalid_variance_rejected(self):
        for value in (0.,-1.,float('nan'),float('inf')):
            with self.assertRaises(ValueError):Wrapper(Mock(),value)
    def test_invalid_measurement_uncertainty_not_hidden(self):
        for value in (-1.,float('nan'),float('inf')):
            pub=Mock();m=S(twist=S(covariance=[0.]*35+[value]))
            with self.assertRaises(ValueError):Wrapper(pub,.0009).publish(m)
            pub.publish.assert_not_called()

if __name__=='__main__':unittest.main()
