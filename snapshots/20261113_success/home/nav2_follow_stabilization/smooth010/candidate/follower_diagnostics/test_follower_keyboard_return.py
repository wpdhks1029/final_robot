import contextlib
import io
import json
import sys
import unittest
from types import SimpleNamespace as S
from unittest.mock import patch

import follower_keyboard_return as keyboard
from test_local_avoidance_operator import Flow


class KeyboardTests(unittest.TestCase):
    def test_intent_expires_and_unknown_keys_stop(self):
        intent=keyboard.KeyIntent()
        for key,w in [('i',0.),('j',.10),('l',-.10)]:
            intent.update(key,1.)
            self.assertEqual(intent.command(1.1)[2],w)
            self.assertEqual(intent.command(1.31),('STOP',0.,0.))
        intent.update('i',2.)
        intent.update('m',2.1)  # No reverse key.
        self.assertEqual(intent.command(2.1),('STOP',0.,0.))

    def test_original_sources_preserved_and_prearm_motion_waits(self):
        intent=keyboard.KeyIntent();intent.update('i',1.)
        obj=keyboard.envelope(intent,1.1,2_000_000_000,'epoch',1_900_000_000,
                              1_800_000_000,1_500_000_000)
        self.assertEqual(obj['source_stamp_ns'],1_800_000_000)
        self.assertEqual(obj['scan_stamp_ns'],1_800_000_000)
        self.assertEqual(obj['motion'],'DRIVE')
        obj=keyboard.envelope(intent,1.1,2_000_000_000,'epoch',1_900_000_000,
                              1_800_000_000,1_950_000_000)
        self.assertEqual((obj['motion'],obj['v'],obj['w']),('STOP',0.,0.))
        with self.assertRaises(ValueError):
            keyboard.envelope(intent,1.1,3_000_000_000,'epoch',1_900_000_000,
                              1_800_000_000,1_500_000_000)

    def flow(self, keys, duplicate=False):
        f=Flow();modules,_=f.modules()
        f.get_name=lambda:'follower_keyboard_return'
        state=f.state
        f.state=lambda:dict(state(),trial_max_seconds=360,clearance_stop_enabled=False)
        if duplicate:
            f.get_publishers_info_by_topic=lambda topic:[S(node_name='other'),S(node_name=f.get_name())]
        reader=iter(keys)
        terminal=S(isatty=lambda:True,fileno=lambda:0,read=lambda n:next(reader,'x'))
        with patch.dict(sys.modules,modules),patch('sys.stdin',terminal), \
             patch('sys.argv',['keyboard','--run','--ack-clear-area-power-cut-ready']), \
             patch.object(keyboard.time,'monotonic',side_effect=lambda:f.t), \
             patch.object(keyboard.select,'select',return_value=([terminal],[],[])), \
             patch.object(keyboard.termios,'tcgetattr',return_value=[]), \
             patch.object(keyboard.termios,'tcsetattr') as restored, \
             patch.object(keyboard.tty,'setcbreak'),contextlib.redirect_stdout(io.StringIO()), \
             contextlib.redirect_stderr(io.StringIO()):
            code=keyboard.main()
        return code,f,restored

    def test_explicit_arm_motion_and_disarm_on_exit(self):
        code,f,restored=self.flow('aiijkx')
        self.assertEqual(code,0)
        self.assertEqual(sum(e[0]=='arm' for e in f.events),1)
        self.assertEqual(sum(e[0]=='disarm' for e in f.events),1)
        commands=[e[1] for e in f.events if e[0]=='envelope']
        self.assertTrue(any(e['motion']=='DRIVE' for e in commands))
        self.assertTrue(any(e['motion']=='TURN' for e in commands))
        self.assertTrue(any(e['motion']=='STOP' for e in commands))
        self.assertTrue(all(e['v']>=0 for e in commands))
        restored.assert_called_once()

    def test_no_arm_key_sends_no_commands(self):
        code,f,restored=self.flow('ijx')
        self.assertEqual(code,0)
        self.assertFalse(any(e[0] in ('arm','envelope') for e in f.events))

    def test_duplicate_writer_never_arms(self):
        code,f,restored=self.flow('aix',duplicate=True)
        self.assertEqual(code,1)
        self.assertFalse(any(e[0]=='arm' for e in f.events))

    def test_default_does_not_start_ros(self):
        with patch('sys.argv',['keyboard']),contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(keyboard.main(),0)


if __name__=='__main__':unittest.main()
