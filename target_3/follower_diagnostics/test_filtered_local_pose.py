import json,math,unittest
from pathlib import Path
from local_stop_turn_go import LocalCore
import test_apf_pivot_candidate as _memory

class Tests(unittest.TestCase):
    def core(self,source='filtered'):
        c=LocalCore(pose_source=source);c.freeze((0,0,0),(0,0,0))
        for i in range(21):
            stamp=100_000_000_000+i*50_000_000
            c.raw(stamp,1+i*.05,(0.,0.,i*.02,0.,.4))
            c.filtered(stamp,1+i*.05,(0.,0.,i*.025,0.,.5))
            c.imu(stamp,1+i*.05,(0.,0.,i*.025,0.,.5))
        c.inputs['leader']=(101_000_000_000,2.,(2.,0.,0.,0.,0.))
        # A fixed reference marker at (1,0), observed at actual heading0.5rad.
        c.offer_scan(101_000_000_000,2.,[(math.cos(.5),-math.sin(.5))])
        return c
    def test_pose_and_scan_both_use_filtered_history_without_raw_mix(self):
        c=self.core();s=c.scene(101_010_000_000,2.01)
        self.assertAlmostEqual(s['pose'].yaw,.5)
        self.assertAlmostEqual(s['points'][0,0],1.);self.assertAlmostEqual(s['points'][0,1],0.)
        raw=self.core('raw').scene(101_010_000_000,2.01)
        self.assertGreater(abs(raw['points'][0,1]),.09)
        self.assertIn('filtered',s['source_stamps']);self.assertIn('raw',s['source_stamps'])
    def test_filtered_loss_never_falls_back_to_raw(self):
        c=self.core();c.inputs['filtered']=(100_000_000_000,1.,(0.,0.,0.,0.,0.))
        with self.assertRaisesRegex(ValueError,'FILTERED_STALE'):c.scene(101_010_000_000,2.01)
    def test_raw_health_remains_required(self):
        c=self.core();c.inputs.pop('raw')
        with self.assertRaisesRegex(ValueError,'RAW_MISSING'):c.scene(101_010_000_000,2.01)
    def test_imu_loss_and_heading_divergence_abort_filtered_mode(self):
        c=self.core();c.inputs.pop('imu')
        with self.assertRaisesRegex(ValueError,'IMU_MISSING'):c.scene(101_010_000_000,2.01)
        c=self.core();c.imu_anchor=0.;c.inputs['imu']=(101_000_000_000,2.,(0.,0.,0.,0.,0.))
        with self.assertRaisesRegex(ValueError,'FILTERED_IMU_HEADING_DIVERGENCE'):c.scene(101_010_000_000,2.01)
    def test_recorded_left_right_orientation_streams_are_consistent(self):
        for name in ('follower_yaw_trial_1793777697627706832.json','follower_yaw_trial_1793777854458447417.json'):
            streams=json.loads((Path(__file__).parent/'logs'/name).read_text())['streams']
            end=min(streams[k][-1]['stamp_ns'] for k in ('raw','filtered','imu'))
            merged=sorted((r['stamp_ns'],k,r) for k in ('raw','filtered','imu') for r in streams[k] if r['stamp_ns']<=end)
            c=LocalCore(pose_source='filtered');anchored=False
            for stamp,key,row in merged:
                if key=='imu':c.imu(stamp,stamp/1e9,(row['roll'],row['pitch'],row['yaw'],0.,0.))
                else:getattr(c,key)(stamp,stamp/1e9,(row['x'],row['y'],row['yaw'],row['v'],row['w']))
                if not anchored and all(k in c.inputs for k in ('raw','filtered','imu')):
                    c.freeze(c.inputs['filtered'][2],(0.,0.,0.));anchored=True
            self.assertTrue(anchored)
            stamp=c.inputs['filtered'][0]
            c.offer_scan(stamp,end/1e9,[(5.,5.)])
            c.inputs['leader']=(end,end/1e9,(5.,0.,0.,0.,0.))
            scene=c.scene(end,end/1e9)
            self.assertLess(abs(scene['heading_check']['filtered_imu_error_deg']),2.)
    def test_newer_measured_sample_closes_scan_bracket_without_extending_hold(self):
        c=self.core();c.offer_scan(101_080_000_000,2.10,[(math.cos(.54),-math.sin(.54))])
        with self.assertRaisesRegex(ValueError,'FILTERED_ODOM_SCAN_BRACKET_MISSING'):c.scene(101_100_000_000,2.10)
        self.assertEqual(c.last_alignment_failure['kind'],'WAITING_FOR_NEWER_POSE')
        self.assertAlmostEqual(c.last_alignment_failure['scan_ahead_of_pose_s'],.08)
        c.filtered(101_100_000_000,2.1,(0.,0.,.55,0.,.5))
        c.imu(101_100_000_000,2.1,(0.,0.,.55,0.,.5))
        c.raw(101_100_000_000,2.1,(0.,0.,.44,0.,.4))
        s=c.scene(101_120_000_000,2.12)
        self.assertEqual(s['source_stamps']['scan_pose_support'],101_000_000_000)
        self.assertAlmostEqual(s['points'][0,0],1.);self.assertAlmostEqual(s['points'][0,1],0.)
    def test_historical_missing_interval_is_not_treated_as_future_callback_wait(self):
        c=self.core()
        c.filtered(101_400_000_000,2.4,(0.,0.,.5,0.,0.))
        c.raw(101_400_000_000,2.4,(0.,0.,.4,0.,0.))
        c.imu(101_400_000_000,2.4,(0.,0.,.5,0.,0.))
        c.inputs['leader']=(101_400_000_000,2.4,(2.,0.,0.,0.,0.))
        c.offer_scan(101_080_000_000,2.4,[(1.,0.)])
        with self.assertRaisesRegex(ValueError,'FILTERED_ODOM_SCAN_BRACKET_MISSING'):c.scene(101_410_000_000,2.41)
        self.assertEqual(c.last_alignment_failure['kind'],'POSE_HISTORY_GAP')

class FilteredMemory(_memory.APFMemoryClosedLoop):
    def core_for_test(self):return LocalCore(pose_source='filtered')
    def offer_pose_for_test(self,core,stamp,mono,data):
        x,y,a,v,w=data
        # Wheel yaw deliberately under-reports by20%; selected fused pose is
        # the physical simulation pose for both tracking and scan projection.
        core.raw(stamp,mono,(x,y,a*.8,v,w*.8));core.filtered(stamp,mono,data)
        core.imu(stamp,mono,(0.,0.,a,0.,w))

if __name__=='__main__':unittest.main()
