import copy,math,unittest
from analyze_scan_anomalies import analyze_event,nearby_support

def scan(t=1_000_000_000,r=None):
    return dict(source_ns=t,angle_min=math.radians(173),angle_increment=math.radians(1),range_min=.28,range_max=16.,ranges=r or [8.,0.,.34,0.,8.])
def event():
    return dict(scan=scan(),hits=[dict(index=2,angle_deg=175.,range_m=.34)],previous=[scan(900_000_000,[8.,0.,0.,0.,8.])],following=[scan(1_100_000_000,[8.,0.,0.,0.,8.])])
class Tests(unittest.TestCase):
    def test_transient_still_stops(self):
        h=analyze_event(event())['hits'][0];self.assertEqual(h['finding'],'ISOLATED_TRANSIENT_CANDIDATE');self.assertEqual(h['control_action'],'UNCHANGED_STOP')
    def test_thin_persistent(self):
        e=event();e['following'][0]['ranges'][2]=.35;self.assertEqual(analyze_event(e)['hits'][0]['finding'],'SUPPORTED_RETURN')
    def test_neighbors(self):
        e=event();e['scan']['ranges'][1]=.36;self.assertEqual(analyze_event(e)['hits'][0]['finding'],'SUPPORTED_RETURN')
    def test_missing_context(self):
        e=event();e['following']=[];self.assertEqual(analyze_event(e)['hits'][0]['finding'],'INSUFFICIENT_CONTEXT')
    def test_old_context(self):
        e=event();e['following'][0]['source_ns']=2_000_000_000;self.assertEqual(analyze_event(e)['hits'][0]['finding'],'INSUFFICIENT_CONTEXT')
    def test_invalid_not_empty_space(self):
        s=scan(r=[None,0.,float('nan'),.1,20.]);v=nearby_support(s,math.radians(175),.34,angular_window_deg=3.);self.assertEqual(v['far_count'],0);self.assertEqual(v['unknown_count'],5)
    def test_wrap(self):
        s=scan(r=[.34]);s['angle_min']=math.radians(-179);self.assertEqual(nearby_support(s,math.radians(179.5),.34)['near_indices'],[0])
    def test_unchanged(self):
        e=event();b=copy.deepcopy(e);analyze_event(e);self.assertEqual(e,b)
if __name__=='__main__':unittest.main()
