import json
from pathlib import Path
import unittest
from test_scan_anomaly_analysis import scan,event
from watch_scan_anomaly_shadow import Shadow

class Tests(unittest.TestCase):
    def test_waits_for_two_future_scans(self):
        s=Shadow();e=event()
        for r in e['previous']:self.assertEqual(s.add(r,[]),[])
        self.assertEqual(s.add(e['scan'],e['hits']),[])
        self.assertEqual(s.add(e['following'][0],[]),[])
        out=s.add(scan(1_200_000_000,[8.,0.,0.,0.,8.]),[])
        self.assertEqual(out[0]['hits'][0]['finding'],'ISOLATED_TRANSIENT_CANDIDATE')
    def test_repeated_thin_object_supported(self):
        s=Shadow();e=event();s.add(e['scan'],e['hits'])
        s.add(scan(1_100_000_000),e['hits']);out=s.add(scan(1_200_000_000),e['hits'])
        self.assertEqual(out[0]['hits'][0]['finding'],'SUPPORTED_RETURN')
    def test_shutdown_without_future_is_insufficient(self):
        s=Shadow();e=event();s.add(e['scan'],e['hits'])
        self.assertEqual(s.finish()[0]['hits'][0]['finding'],'INSUFFICIENT_CONTEXT')
    def test_time_reversal_clears_context(self):
        s=Shadow();e=event();s.add(e['scan'],e['hits']);s.add(scan(500_000_000),[])
        self.assertEqual(s.invalid_order,1);self.assertEqual(len(s.previous),0);self.assertEqual(s.pending,[])
    def test_recorded_three_events(self):
        p=Path(__file__).parent/'logs/scan_anomalies_1793618024228190275.json'
        if not p.exists():self.skipTest('실측 파일 없음')
        d=json.loads(p.read_text());rows={};hits={}
        for e in d['events']:
            for r in e['previous']+[e['scan']]+e['following']:rows[r['source_ns']]=r
            hits[e['scan']['source_ns']]=e['hits']
        s=Shadow();out=[]
        for stamp,r in sorted(rows.items()):out.extend(s.add(r,hits.get(stamp,[])))
        out.extend(s.finish());self.assertEqual(len(out),3)
        self.assertTrue(all(h['finding']=='ISOLATED_TRANSIENT_CANDIDATE' and h['control_action']=='UNCHANGED_STOP' for e in out for h in e['hits']))
if __name__=='__main__':unittest.main()
