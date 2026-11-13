import unittest
from audit_md200t_settings import compare_settings


class ComparisonTests(unittest.TestCase):
    def row(self,pid,value):
        return dict(pid=pid,result='OK',raw_unsigned=value)

    def test_detects_encoder_setting_change_without_writing_or_authorizing(self):
        r=compare_settings([self.row(156,4096),self.row(170,4096)],
                           [self.row(156,4096),self.row(170,1024)])
        self.assertEqual(r['different_pids'],[170])
        self.assertFalse(r['motion_authorized'])

    def test_missing_or_unsupported_values_remain_unknown(self):
        r=compare_settings([self.row(170,4096),self.row(21,10)],
                           [dict(pid=170,result='NO_REPLY_UNKNOWN')])
        self.assertEqual(r['unknown_pids'],[21,170])
        self.assertEqual(r['different_pids'],[])

    def test_matching_settings_do_not_certify_sensor_health(self):
        r=compare_settings([self.row(156,4096)],[self.row(156,4096)])
        self.assertEqual(r['settings'][0]['result'],'MATCH')
        self.assertFalse(r['physical_sensor_health_verified'])

    def test_duplicate_register_results_are_not_silently_overwritten(self):
        with self.assertRaisesRegex(ValueError,'DUPLICATE_PID'):
            compare_settings([self.row(156,4096)],[self.row(156,4096),self.row(156,1024)])


if __name__=='__main__':unittest.main()
