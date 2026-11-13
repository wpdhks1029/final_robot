import unittest
from watch_follow_alignment import phase_comparisons

class Tests(unittest.TestCase):
    def streams(self):
        ns=100_000_000_000
        return {key:[dict(stamp_ns=ns+i*100_000_000,yaw=i*.001,roll=0.,pitch=0.) for i in range(21)] for key in ('raw','filtered','imu','map')}
    def row(self,i,phase,stage=None):return dict(stamp_ns=100_000_000_000+i*100_000_000,trial_phase=phase,avoidance_stage=stage,motor_armed=True,transform=[0.,0.,i*.001])
    def test_curve_and_follow_stages_remain_separate(self):
        rows=[self.row(i,'AVOID','MOVE_OUT') for i in range(1,5)]+[self.row(i,'AVOID','PASS_BOX') for i in range(5,9)]+[self.row(i,'FOLLOW') for i in range(9,14)]
        groups=phase_comparisons(rows,self.streams())
        self.assertEqual([(g['trial_phase'],g['avoidance_stage']) for g in groups],[('AVOID','MOVE_OUT'),('AVOID','PASS_BOX'),('FOLLOW',None)])
        self.assertEqual(groups[-1]['begin_ns'],rows[8]['stamp_ns'])
        self.assertTrue(all('delta_deg' in x for x in groups[0]['yaw_comparison']['sources'].values()))
    def test_unarmed_and_unknown_periods_do_not_merge_different_motion_windows(self):
        row=self.row(4,'FOLLOW');row['motor_armed']=False
        rows=[self.row(1,'FOLLOW'),self.row(2,'FOLLOW'),row,self.row(5,'FOLLOW'),self.row(6,'FOLLOW')]
        self.assertEqual(len(phase_comparisons(rows,self.streams())),2)

if __name__=='__main__':unittest.main()
