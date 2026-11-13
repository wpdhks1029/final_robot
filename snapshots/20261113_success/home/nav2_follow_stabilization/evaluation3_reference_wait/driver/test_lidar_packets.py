import unittest
from analyze_lidar_packets import decode

def packet():
    d=bytearray.fromhex('aa55000201008100');payload=bytes([7,0x82,0x05,10,0xd7,0x04]);c=0
    for i in range(0,8,2):c^=int.from_bytes(d[i:i+2],'little')
    for i in range(0,len(payload),3):c^=payload[i];c^=int.from_bytes(payload[i+1:i+3],'little')
    return d+c.to_bytes(2,'little')+payload
class Tests(unittest.TestCase):
    def test_valid_raw_values_and_flags(self):
        r=decode(packet().hex());self.assertTrue(r['checksum_ok']);self.assertEqual(r['points'][0]['raw_distance'],1408);self.assertEqual(r['points'][0]['noise_flag'],2);self.assertEqual(r['points'][1]['noise_flag'],3)
    def test_corrupted_byte_fails(self):
        p=packet();p[11]^=4;self.assertFalse(decode(p.hex())['checksum_ok'])
    def test_truncated_fails(self):
        with self.assertRaises(ValueError):decode(packet()[:-1].hex())
    def test_wrong_header_fails(self):
        p=packet();p[0]=0
        with self.assertRaises(ValueError):decode(p.hex())
if __name__=='__main__':unittest.main()
