import ast,math,unittest
from pathlib import Path
from watch_follow_alignment import alignment_sample,summarize,common_yaw_comparison

class Tests(unittest.TestCase):
    def test_se2_transform_uses_poses_at_measured_time(self):
        ns=100_000_000_000;h=[(ns,1.,(1.,2.,.1,0.,0.)),(ns+100_000_000,1.1,(1.1,2.,.1,0.,0.))]
        x=1.05;y=2.;a=.3;c,s=math.cos(a),math.sin(a)
        r=alignment_sample(dict(stamp_ns=ns+50_000_000,x=3+c*x-s*y,y=4+s*x+c*y,yaw=.4),h)
        for actual,expected in zip(r['transform'],(3.,4.,.3)):self.assertAlmostEqual(actual,expected)
        self.assertEqual(r['pose_support_ns'],ns)
    def test_large_missing_interval_still_rejected(self):
        ns=100_000_000_000;h=[(ns,1.,(0.,0.,0.,0.,0.)),(ns+310_000_000,1.31,(0.,0.,0.,0.,0.))]
        with self.assertRaises(ValueError):alignment_sample(dict(stamp_ns=ns+150_000_000,x=0.,y=0.,yaw=0.),h)
    def test_summary_detects_transform_drift_without_calibrating(self):
        rows=[dict(stamp_ns=1,transform=[0.,0.,0.]),dict(stamp_ns=1_000_000_001,transform=[.06,0.,math.radians(4)])]
        r=summarize(rows);self.assertEqual(r['reason'],'OBSERVED_MAP_ALIGNMENT_VARIATION');self.assertAlmostEqual(r['transform_yaw_span_deg'],4.)
        self.assertEqual(rows[-1]['transform'][0],.06)
    def test_yaw_comparison_uses_four_source_overlap_not_only_raw_imu_edges(self):
        ns=100_000_000_000
        rows=lambda a,b:[dict(stamp_ns=ns+i*100_000_000,yaw=0.,roll=0.,pitch=0.) for i in range(a,b+1)]
        streams=dict(raw=rows(0,20),filtered=rows(1,19),imu=rows(0,20),map=rows(2,18))
        r=common_yaw_comparison(streams)
        self.assertEqual(r['begin_ns'],ns+200_000_000);self.assertEqual(r['end_ns'],ns+1_800_000_000)
        self.assertTrue(all('delta_deg' in x for x in r['sources'].values()))

    def test_observer_has_no_motion_interfaces(self):
        tree=ast.parse(Path(__file__).with_name('watch_follow_alignment.py').read_text())
        forbidden={'create_publisher','create_client','call_async','arm','set_parameters'}
        calls={n.func.attr for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute)}
        self.assertFalse(calls&forbidden)

if __name__=='__main__':unittest.main()
