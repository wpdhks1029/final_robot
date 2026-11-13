import unittest
from types import SimpleNamespace as S
from follower_stamped_preview import oldest_pursuit_source,StampedGuard
from follower_obstacle_preview import ObstacleCore


def msg(stamp=1_000_000_000,frame='follower/base_link',v=.1):
    return S(header=S(frame_id=frame,stamp=S(sec=stamp//10**9,nanosec=stamp%10**9)),
             twist=S(linear=S(x=v,y=0.,z=0.),angular=S(x=0.,y=0.,z=0.)))


class Tests(unittest.TestCase):
    def make(self):
        c=ObstacleCore();c.scan=(1_000_000_000,1.,1.,100)
        return StampedGuard(c)

    def test_oldest_source_not_publish_time(self):
        inputs={name:(msg(ns).header.stamp,0,None) for name,ns in
                [('path',1_000_000_000),('leader',1_100_000_000),('follower',1_200_000_000)]}
        self.assertEqual(oldest_pursuit_source(inputs),1_000_000_000)
        del inputs['path'];self.assertEqual(oldest_pursuit_source(inputs),0)

    def test_clear_forward_preserves_source(self):
        c=self.make();c.update(msg(),1_100_000_000,1.1)
        v,w,stamp,reason=c.command(1_150_000_000,1.15)
        self.assertEqual(v,.1);self.assertEqual(stamp,1_000_000_000)

    def test_remote_clock_margin_moves_effective_source_earlier_only(self):
        inputs={name:(msg(ns).header.stamp,0,None) for name,ns in
                [('path',1_020_000_000),('leader',1_020_000_000),('follower',1_000_000_000)]}
        self.assertEqual(oldest_pursuit_source(inputs,remote_clock_margin_ns=50_000_000),970_000_000)

    def test_old_scan_is_oldest(self):
        c=self.make();c.update(msg(1_100_000_000),1_100_000_000,1.1)
        self.assertEqual(c.command(1_150_000_000,1.15)[2],1_000_000_000)

    def test_republishing_stale_input_cannot_refresh_source(self):
        c=self.make();c.update(msg(),1_100_000_000,1.1)
        c.update(msg(),1_400_000_000,1.4)
        self.assertEqual(c.command(1_400_000_000,1.4)[:3],(0.,0.,0))

    def test_future_wrong_frame_missing_scan_stops(self):
        for m in [msg(1_200_000_000),msg(frame='map'),msg(0)]:
            c=self.make();c.update(m,1_100_000_000,1.1)
            self.assertEqual(c.command(1_100_000_000,1.1)[:3],(0.,0.,0))
        c=self.make();c.update(msg(),1_100_000_000,1.1);c.core.scan=None
        self.assertEqual(c.command(1_100_000_000,1.1)[:3],(0.,0.,0))

    def test_obstacle_zero_retains_fresh_source(self):
        c=self.make();c.core.scan=(1_000_000_000,1.,.1,100)
        c.update(msg(),1_100_000_000,1.1)
        self.assertEqual(c.command(1_100_000_000,1.1)[:3],(0.,0.,1_000_000_000))


if __name__=='__main__':unittest.main()
