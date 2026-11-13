import unittest
from unittest.mock import patch
import subprocess
import contextlib,io
from wireless_power_profile import active_uuid,power_value,runtime_power,plan,main
U='12345678-1234-1234-1234-123456789abc'
class Tests(unittest.TestCase):
    def test_active_profile(self):self.assertEqual(active_uuid(U+':wlan0\n'+U+':eth0','wlan0'),U)
    def test_missing_duplicate_rejected(self):
        for s in ['',U+':eth0',U+':wlan0\n'+U+':wlan0']:
            with self.assertRaises(ValueError):active_uuid(s,'wlan0')
    def test_profile_values(self):
        for i in range(4):self.assertEqual(power_value(str(i)+' (label)'),i)
        self.assertEqual(power_value('disabled'),2)
        self.assertEqual(power_value('0(default)'),0)
        self.assertEqual(power_value('2(disable)'),2)
        self.assertEqual(power_value('enable'),3)
        with self.assertRaises(ValueError):power_value('unknown')
    def test_runtime_values(self):
        self.assertEqual(runtime_power('Power save: off'),'off')
        with self.assertRaises(ValueError):runtime_power('unsupported')
    def test_only_power_settings_changed(self):
        c=plan('wlan0',U);self.assertEqual(c[0][-2:],['802-11-wireless.powersave','2']);self.assertEqual(c[1],['sudo','iw','dev','wlan0','set','power_save','off'])
        self.assertFalse(any('down' in x or 'restart' in x for x in c))
    def test_invalid_interface_and_restore_values_rejected(self):
        for iface in ['wlan0;reboot','$(cmd)','a b','']:
            with self.assertRaises(ValueError):plan(iface,U)
        with self.assertRaises(ValueError):plan('wlan0',U,4)
    def test_missing_networkmanager_does_not_mutate(self):
        with patch('sys.argv',['wifi','--role','follower','--iface','wlan0','--apply']),patch('wireless_power_profile.shutil.which',return_value=None),patch('wireless_power_profile.read') as read,patch('wireless_power_profile.subprocess.run') as run,contextlib.redirect_stdout(io.StringIO()):
            main();read.assert_not_called();run.assert_not_called()
    def test_missing_networkmanager_and_failed_iw_no_traceback(self):
        with patch('sys.argv',['wifi','--role','follower','--iface','wlan0','--apply']),patch('wireless_power_profile.shutil.which',side_effect=[None,'/usr/sbin/iw']),patch('wireless_power_profile.read',side_effect=subprocess.CalledProcessError(1,'iw')),contextlib.redirect_stdout(io.StringIO()):
            main()
    def test_rollback_plan(self):self.assertEqual(plan('wlan0',U,3,'on')[0][-1],'3')
if __name__=='__main__':unittest.main()
