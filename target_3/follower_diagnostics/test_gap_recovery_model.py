"""가상 직선 모델로 간격 회복 방향만 확인. 실제 속도·제동 성능 검증이 아님."""
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0,str(Path.home()/'robot_ws/test_tools'))
import test_follower_input_safety as fake
from follower_pursuit_preview import PreviewCore
from follower_motor_candidate import MotorCore,Config


def simulate(leader_v,seconds=9.5):
    core=PreviewCore();motor=MotorCore(config=Config(max_rpm=10))
    leader=fake.odom();follower=fake.odom('follower/base_link')
    lx=1.73;fx=0.;fv=0.;dt=.05;gaps=[]
    for i in range(round(seconds/dt)):
        ns=100_000_000_000+int((i+1)*dt*10**9);mono=1.+(i+1)*dt
        for m in (leader,follower):
            m.header.stamp.sec=ns//10**9;m.header.stamp.nanosec=ns%10**9
        leader.pose.pose.position.x=lx;leader.twist.twist.linear.x=leader_v
        follower.pose.pose.position.x=fx;follower.twist.twist.linear.x=fv
        path=fake.S(header=leader.header,poses=[
            fake.S(header=leader.header,pose=fake.S(position=fake.S(x=1.63,y=0.))),
            fake.S(header=leader.header,pose=fake.S(position=fake.S(x=lx,y=0.)))])
        core.path_callback(path,ns,mono)
        core.odom_callback('leader',leader,ns,mono);core.odom_callback('follower',follower,ns,mono)
        v,w,reason=core.command(ns,mono)
        left,right=motor.rpm(v,w)
        fv=((-left)+right)/2*2*math.pi*.065/60
        fx+=fv*dt;lx+=leader_v*dt;gaps.append(lx-fx)
    return gaps


class Tests(unittest.TestCase):
    def test_more_speed_headroom_reduces_existing_gap_error(self):
        fast=simulate(.064);slow=simulate(.042)
        self.assertLess(slow[-1],fast[-1])
        self.assertLess(slow[-1],1.65)
        self.assertGreater(min(slow),1.5)

    def test_stopped_leader_does_not_start_follower_in_model(self):
        gaps=simulate(0.)
        self.assertTrue(all(abs(g-1.73)<1e-9 for g in gaps))


if __name__=='__main__':unittest.main()
