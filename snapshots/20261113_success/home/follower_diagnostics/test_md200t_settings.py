import unittest
from unittest.mock import Mock,patch
from read_md200t_settings import request,extract,value,ReadOnlyPort,main,FIELDS
from follower_motor_candidate import velocity_packet


def frame(pid,data):
    r=bytes([0xB8,0xB7,1,pid,len(data)])+data
    return r+bytes([(-sum(r))&255])


class Tests(unittest.TestCase):
    def test_all_requests_pid4_only(self):
        for pid in [*FIELDS,216]:
            p=request(pid);self.assertEqual(p[:5],b'\xb7\xb8\x01\x04\x01')
            self.assertEqual(p[5],pid);self.assertEqual(sum(p)&255,0)
        with self.assertRaises(ValueError):request(3)

    def test_port_rejects_motion_and_writes(self):
        device=Mock();port=ReadOnlyPort(device)
        for p in [velocity_packet(0,0),velocity_packet(-3,3),frame(79,b'\x03')]:
            with self.assertRaises(ValueError):port.write(p)
        device.write.assert_not_called();port.write(request(79));device.write.assert_called_once()

    def test_partial_noise_and_checksum(self):
        good=frame(156,b'\x00\x10');bad=bytearray(good);bad[-1]^=1
        b=bytearray(b'noise')+bad+good[:4]
        self.assertEqual(extract(b),[]);b.extend(good[4:])
        self.assertEqual(extract(b),[good])

    def test_value_and_unknown(self):
        self.assertEqual(value(156,frame(156,b'\x00\x10'))['raw_unsigned'],4096)
        self.assertEqual(value(156,None)['result'],'NO_REPLY_UNKNOWN')
        self.assertEqual(value(156,frame(156,b'\x00'))['result'],'UNEXPECTED_LENGTH_UNKNOWN')

    def test_default_no_open(self):
        with patch('sys.argv',['read']),patch.object(ReadOnlyPort,'open') as opened:
            self.assertEqual(main(),0);opened.assert_not_called()


if __name__=='__main__':unittest.main()
