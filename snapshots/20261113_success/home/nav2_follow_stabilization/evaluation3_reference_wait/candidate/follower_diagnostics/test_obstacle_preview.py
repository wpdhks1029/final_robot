"""Rectangular scan gate semantics; no ROS initialization, serial or motion."""
import math
from types import SimpleNamespace as S
import unittest

from follower_obstacle_preview import ObstacleCore,validate_output_topic,OUTPUT_TOPIC


def stamp(sec=100):return S(sec=sec,nanosec=0)
def twist(v=.1,w=0.):return S(linear=S(x=v,y=0.,z=0.),angular=S(x=0.,y=0.,z=w))
def scan(r=.8,angle=0.,frame='follower/laser'):
    return S(header=S(stamp=stamp(),frame_id=frame),ranges=[r],angle_min=angle,
             angle_increment=.01,range_min=.28,range_max=16.)
def transform(yaw=0.):
    return S(header=S(frame_id='follower/base_link'),child_frame_id='follower/laser',
             transform=S(translation=S(x=.26,y=0.,z=.24),
                         rotation=S(x=0.,y=0.,z=math.sin(yaw/2),w=math.cos(yaw/2))))


class GuardTests(unittest.TestCase):
    def make(self,msg=None,tf=None):
        c=ObstacleCore();c.update_raw(twist(),10.)
        c.update_scan(msg if msg is not None else scan(),tf if tf is not None else transform(),
                      100_100_000_000,10.)
        return c

    def command(self,c):return c.command(100_100_000_000,10.)
    def stopped(self,c):self.assertEqual(self.command(c)[:2],(0.,0.))

    def test_clear_scan_passes(self):
        c=self.make();self.assertEqual(self.command(c),(.1,0.,'PREVIEW_PASS'))

    def test_forward_offset_applied(self):
        c=self.make(scan(.5));self.assertAlmostEqual(c.scan[2],.49)
        v,_,why=self.command(c);self.assertGreater(v,0);self.assertLess(v,.1)
        self.assertEqual(why,'PREVIEW_SLOW')

    def test_front_stop(self):self.stopped(self.make(scan(.30)))
    def test_side_stop(self):self.stopped(self.make(scan(.5,math.pi/2)))

    def test_rotation_transform(self):
        c=self.make(scan(.8),transform(math.pi))
        self.assertAlmostEqual(c.scan[2],.27);self.stopped(c)

    def test_corner_distance(self):
        c=ObstacleCore();self.assertAlmostEqual(c.clearance(.47,.50),math.hypot(.2,.2))
        self.assertEqual(c.clearance(0.,0.),0.)

    def test_near_return_never_masked(self):
        c=self.make(scan(.3));self.assertEqual(self.command(c)[2],'STOP_FOOTPRINT_CLEARANCE')

    def test_zero_upstream_preserved(self):
        c=self.make(scan(.3));c.update_raw(twist(0.),10.)
        self.assertEqual(self.command(c),(0.,0.,'STOP_UPSTREAM'))

    def test_command_expiry(self):
        c=self.make();c.raw=(9.,.1,0.);self.stopped(c)

    def test_scan_measurement_expiry(self):
        c=self.make();self.assertEqual(c.command(100_600_000_001,10.)[:2],(0.,0.))

    def test_scan_reception_expiry(self):
        c=self.make();s,_,d,n=c.scan;c.scan=(s,9.,d,n);self.stopped(c)

    def test_invalid_new_scan_clears_old(self):
        for mode in ('frame','future','old','zero','empty','nan','infinite','below_min'):
            c=self.make();msg=scan()
            if mode=='frame':msg.header.frame_id='laser_frame'
            elif mode in ('future','old','zero'):msg.header.stamp.sec={'future':101,'old':98,'zero':0}[mode]
            else:msg.ranges={'empty':[],'nan':[float('nan')],'infinite':[float('inf')],'below_min':[.1]}[mode]
            c.update_scan(msg,transform(),100_100_000_000,10.);self.stopped(c)

    def test_missing_or_invalid_tf(self):
        for mode in ('none','wrong_frame','nan','quaternion'):
            c=self.make();tf=transform()
            if mode=='none':tf=None
            elif mode=='wrong_frame':tf.header.frame_id='base_link'
            elif mode=='nan':tf.transform.translation.x=float('nan')
            else:tf.transform.rotation.w=0.
            c.update_scan(scan(),tf,100_100_000_000,10.);self.stopped(c)

    def test_raw_invalid(self):
        for msg in (twist(-.1),twist(float('nan')),twist(0.,.1)):
            c=self.make();c.update_raw(msg,10.);self.stopped(c)

    def test_caps(self):
        c=self.make();c.update_raw(twist(1.,1.),10.)
        self.assertEqual(self.command(c)[:2],(.1,.2))

    def test_output_remap_rejected(self):
        validate_output_topic(OUTPUT_TOPIC)
        for topic in ('/cmd_vel','/follower/cmd_vel','/follower/cmd_vel_raw'):
            with self.assertRaises(RuntimeError):validate_output_topic(topic)


if __name__=='__main__':unittest.main(verbosity=2)
