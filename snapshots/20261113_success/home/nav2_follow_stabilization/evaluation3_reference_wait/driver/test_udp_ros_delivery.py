import unittest
from check_udp_ros_delivery import encode,decode,summary,WIRE

def row(name,t,**extra):return dict(name=name,rx_mono_ns=round(t*1e9),rx_wall_ns=100_000_000_000+round(t*1e9),**extra)
class Tests(unittest.TestCase):
    def test_format(self):
        for k in (0,1):
            b=encode(k,17,25,1000,2000);self.assertEqual(len(b),38);self.assertEqual(decode(b),(k,17,25,1000,2000))
    def test_bad_packets(self):
        g=encode(0,1,2,3,4)
        for b in [g[:-1],g+b'0',b'NOPE'+g[4:],WIRE.pack(b'FDG1',2,0,1,2,3,4),WIRE.pack(b'FDG1',1,2,1,2,3,4)]:
            with self.assertRaises(ValueError):decode(b)
    def test_warmup(self):
        s=summary([row('leader',0),row('leader',2),row('leader',21),row('leader',21.05)],0,20)
        self.assertEqual(s['streams']['leader']['received'],2);self.assertAlmostEqual(s['streams']['leader']['max_gap_s'],.05)
    def test_healthy_udp_during_ros_gap(self):
        r=[row('leader',21),row('leader',21.35)]
        for i in range(9):r.extend([row('udp_peer_probe',20.95+i*.05,seq=i,session=1),row('observer_tick',20.95+i*.05)])
        r.append(row('udp_echo',21.2,rtt_s=.012));c=summary(r,0,20)['ros_long_gap_correlations'][0]
        self.assertAlmostEqual(c['ros_gap_s'],.35);self.assertAlmostEqual(c['overlapping_udp_gap_s'],.05);self.assertAlmostEqual(c['overlapping_observer_gap_s'],.05);self.assertEqual(c['nearby_max_rtt_s'],.012)
    def test_common_gap(self):
        r=[row(n,t,seq=i,session=1) for n in ('leader','udp_peer_probe','observer_tick') for i,t in enumerate([21,21.4])]
        c=summary(r,0,20)['ros_long_gap_correlations'][0];self.assertAlmostEqual(c['overlapping_udp_gap_s'],.4);self.assertAlmostEqual(c['overlapping_observer_gap_s'],.4)
    def test_missing_udp_unknown(self):
        c=summary([row('leader',21),row('leader',21.4)],0,20)['ros_long_gap_correlations'][0]
        self.assertIsNone(c['overlapping_udp_gap_s']);self.assertIsNone(c['nearby_max_rtt_s'])
    def test_reordered_sequence(self):
        r=[row('udp_peer_probe',21,seq=1,session=1),row('udp_peer_probe',21.1,seq=3,session=1)]
        self.assertEqual(summary(r,0,20)['streams']['udp_peer_probe']['unseen_sequences_within_observed_span'],1)
        r.append(row('udp_peer_probe',21.2,seq=2,session=1));self.assertEqual(summary(r,0,20)['streams']['udp_peer_probe']['unseen_sequences_within_observed_span'],0)
    def test_empty(self):self.assertEqual(summary([],0,20)['streams'],{})
if __name__=='__main__':unittest.main()
