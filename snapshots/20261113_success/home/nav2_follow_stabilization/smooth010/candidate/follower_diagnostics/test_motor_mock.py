"""Mock wire/transport tests; no ROS initialization or physical serial access."""
import json
import unittest

from follower_motor_mock import parse_envelope,MemoryTransport
from follower_motor_candidate import read_request,velocity_packet
from md200t_pid216_v2 import decode


class MockTests(unittest.TestCase):
    def envelope(self):
        return {'version':1,'epoch':'0123456789abcdef','frame':'follower/base_link',
                'stamp_ns':100000000000,'v':.02,'w':0.}

    def test_valid_envelope(self):
        obj=self.envelope();self.assertEqual(parse_envelope(json.dumps(obj)),obj)

    def test_missing_and_extra_keys(self):
        for change in ('missing','extra'):
            obj=self.envelope()
            if change=='missing':del obj['epoch']
            else:obj['unexpected']=1
            with self.assertRaises(ValueError):parse_envelope(json.dumps(obj))

    def test_duplicate_keys(self):
        text=json.dumps(self.envelope())[:-1]+',"v":0.1}'
        with self.assertRaises(ValueError):parse_envelope(text)

    def test_invalid_versions(self):
        for value in (True,2,'1'):
            obj=self.envelope();obj['version']=value
            with self.assertRaises(ValueError):parse_envelope(json.dumps(obj))

    def test_invalid_epochs(self):
        for value in ('',1,'0123456789abcdeg','0123456789abcdefx'):
            obj=self.envelope();obj['epoch']=value
            with self.assertRaises(ValueError):parse_envelope(json.dumps(obj))

    def test_invalid_stamps(self):
        for value in (0,-1,True,1.5,'100'):
            obj=self.envelope();obj['stamp_ns']=value
            with self.assertRaises(ValueError):parse_envelope(json.dumps(obj))

    def test_invalid_velocities(self):
        for value in (True,'0.1',float('nan'),float('inf')):
            obj=self.envelope();obj['v']=value
            with self.assertRaises(ValueError):parse_envelope(json.dumps(obj))

    def test_invalid_frame_and_size(self):
        obj=self.envelope();obj['frame']='base_link'
        with self.assertRaises(ValueError):parse_envelope(json.dumps(obj))
        with self.assertRaises(ValueError):parse_envelope(' '*2049)

    def test_read_response(self):
        transport=MemoryTransport();transport.write(read_request())
        data=decode(transport.read(transport.in_waiting))
        self.assertEqual((data['m1_rpm'],data['m2_rpm']),(0,0))

    def test_velocity_response(self):
        transport=MemoryTransport();transport.write(velocity_packet(-3,3))
        data=decode(transport.read(transport.in_waiting))
        self.assertEqual((data['m1_rpm'],data['m2_rpm']),(-3,3))

    def test_feedback_drop_still_changes_model_only(self):
        transport=MemoryTransport();transport.feedback_enabled=False
        transport.write(velocity_packet(-3,3))
        self.assertEqual(transport.rpm,(-3,3));self.assertEqual(transport.in_waiting,0)

    def test_malformed_packet_rejected(self):
        transport=MemoryTransport()
        for raw in (b'',b'bad',read_request(2)):
            with self.assertRaises((ValueError,IndexError)):transport.write(raw)

    def test_closed_transport(self):
        transport=MemoryTransport();transport.close()
        with self.assertRaises(OSError):transport.write(read_request())


if __name__=='__main__':unittest.main(verbosity=2)
