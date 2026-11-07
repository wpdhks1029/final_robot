import unittest
from analyze_position_trace import analyze


def row(t,phase,l,r):
    return dict(elapsed=t,phase=phase,m1_position=l,m2_position=r,
                m1_rpm=0,m2_rpm=0,m1_status=0,m2_status=0,odom_x=0.)


class Tests(unittest.TestCase):
    def test_delta_windows_and_raw_not_hidden(self):
        result=analyze([row(0,'WAIT',10,20),row(1,'DONE',-6,36),row(2,'DONE',-6,36),
                        row(5,'DONE',-7,37),row(6,'DONE',-6,37),row(7,'DONE',-7,37)])
        self.assertEqual(result['first_done']['left_units'],16)
        self.assertEqual(result['final_raw']['right_units'],17)
        self.assertEqual(len(result['done_change_events']),3)
        self.assertEqual(result['done_position_range_units']['left'],[16,17])
        self.assertAlmostEqual(result['first_done']['left_angle_deg'],96.)

    def test_missing_done_refused(self):
        with self.assertRaises(ValueError):analyze([row(0,'WAIT',0,0),row(1,'PULSE',-1,1)])

    def test_bad_order_refused(self):
        with self.assertRaises(ValueError):analyze([row(1,'WAIT',0,0),row(0,'DONE',-1,1)])

    def test_wrap(self):
        result=analyze([row(0,'WAIT',-2**31,2**31-1),row(1,'DONE',2**31-1,-2**31)])
        self.assertEqual(result['final_raw']['left_units'],1)
        self.assertEqual(result['final_raw']['right_units'],1)


if __name__=='__main__':unittest.main()
