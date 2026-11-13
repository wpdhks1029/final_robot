import unittest
from unittest.mock import patch
import read_md200t_settings as reader
from read_md200t_faults import FAULT_FIELDS,describe


class FaultReadTests(unittest.TestCase):
    def test_current_motor2_sensor_fault_and_history_are_separate(self):
        frame=bytes([0xB8,0xB7,1,99,8])+bytes([0,0,0,0,1,2,0,0])+bytes([0])
        r=describe(99,frame,{'result':'OK'})
        self.assertEqual(r['current_motor_status']['m2']['flags'],['ALARM'])
        self.assertEqual(r['current_motor_status']['m2']['flags2'],['ENC_FAIL'])
        frame=bytes([0xB8,0xB7,1,229,12])+bytes([130,0,0,0,0,0,7,0,0,1,0,0])+bytes([0])
        r=describe(229,frame,{'result':'OK'})
        self.assertEqual(r['historical_counts']['HALL_FAIL'],7)
        self.assertEqual(r['historical_counts']['ENC_FAIL'],1)
        self.assertNotIn('current_motor_status',r)

    def test_fault_queries_are_only_pid4_read_requests(self):
        with patch.object(reader,'FIELDS',dict(FAULT_FIELDS)):
            for pid in (*FAULT_FIELDS,216):
                packet=reader.request(pid)
                self.assertEqual(packet[:6],bytes([0xB7,0xB8,1,4,1,pid]))
                self.assertEqual(sum(packet)&255,0)
            for pid in (10,12,13,207):
                with self.assertRaises(ValueError):reader.request(pid)

    def test_missing_and_unrecognized_length_remain_unknown(self):
        with patch.object(reader,'FIELDS',dict(FAULT_FIELDS)):
            self.assertEqual(reader.value(60,None)['result'],'NO_REPLY_UNKNOWN')
            frame=bytes([0xB8,0xB7,1,229,9])+bytes(10)
            result=reader.value(229,frame)
            self.assertEqual(result['result'],'UNEXPECTED_LENGTH_UNKNOWN')
            self.assertIn('rx',result)


if __name__=='__main__':unittest.main()
