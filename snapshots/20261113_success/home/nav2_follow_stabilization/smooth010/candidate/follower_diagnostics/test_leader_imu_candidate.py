"""실제 센서·ROS 없이 IMU 메시지 변환 비교."""
import ast
import math
from pathlib import Path
from types import SimpleNamespace as S
import unittest

class ConversionTests(unittest.TestCase):
    def callback(self,filename,gz):
        tree=ast.parse(Path(__file__).with_name(filename).read_text())
        cls=next(n for n in tree.body if isinstance(n,ast.ClassDef))
        methods=[n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name in ('timer_callback','euler_to_quaternion')]
        for method in methods:method.decorator_list=[]
        module=ast.Module(body=methods,type_ignores=[]);ast.fix_missing_locations(module)
        def message():return S(header=S(),orientation=S(),angular_velocity=S(),linear_acceleration=S())
        ns={'math':math,'Imu':message};exec(compile(module,filename,'exec'),ns)
        output=[]
        values={'g':[0.1,-0.2,gz],'a':[0.,0.,1.],'e':[0.,0.,17.]}
        node=S(send_command=lambda cmd:values[cmd],euler_to_quaternion=ns['euler_to_quaternion'],
               frame_id='imu_link',get_clock=lambda:S(now=lambda:S(to_msg=lambda:123)),
               publisher_=S(publish=output.append))
        ns['timer_callback'](node)
        return output[0]
    def test_low_positive_and_negative_rotation_preserved(self):
        for degrees in (0.1,-0.1,0.8,-0.8):
            new=self.callback('leader_imu_no_deadband_candidate.py',degrees)
            old=self.callback('leader_imu_current.py',degrees)
            self.assertEqual(old.angular_velocity.z,0.)
            self.assertAlmostEqual(new.angular_velocity.z,math.radians(degrees)*1.027)
    def test_zero_is_still_zero(self):
        self.assertEqual(self.callback('leader_imu_no_deadband_candidate.py',0.).angular_velocity.z,0.)
    def test_other_fields_and_large_rotation_unchanged(self):
        a=self.callback('leader_imu_current.py',3.)
        b=self.callback('leader_imu_no_deadband_candidate.py',3.)
        self.assertEqual(vars(a),vars(b))

if __name__=='__main__':unittest.main()
