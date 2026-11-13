#!/usr/bin/env python3
"""Read-only current map/path comparison against the last trial endpoint."""
import json,math,time
from pathlib import Path
from types import SimpleNamespace as S
from live_rejoin_verification import LiveRejoinVerification

def main():
    import rclpy
    from nav_msgs.msg import Odometry,Path as RosPath
    from rclpy.qos import QoSProfile,ReliabilityPolicy
    root=Path(__file__).resolve().parent
    log=max((root/'logs').glob('local_avoidance_trial_*.jsonl'),key=lambda p:p.stat().st_mtime)
    rows=[json.loads(s) for s in log.read_text().splitlines()]
    initial=next(r for r in rows if r['event']=='planning_input')
    end=next(r for r in reversed(rows) if r['event']=='local_control')
    rclpy.init();node=rclpy.create_node('inspect_last_rejoin_read_only');inputs={}
    def odom(m):
        p=m.pose.pose.position;q=m.pose.pose.orientation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        inputs['follower']=(m.header.stamp,time.monotonic(),(p.x,p.y,yaw,0.,0.))
    def path(m):inputs['path']=(m.header.stamp,time.monotonic(),[(p.pose.position.x,p.pose.position.y) for p in m.poses])
    qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
    subs=[node.create_subscription(Odometry,'/follower/odom_map',odom,qos),node.create_subscription(RosPath,'/leader/path',path,qos)]
    print('조회 전용 8초. 로봇과 초기 pose를 그대로 유지하세요. 명령·arm 없음.',flush=True)
    until=time.monotonic()+8
    try:
        while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.1)
        result=LiveRejoinVerification().check(inputs,node.get_clock().now().nanoseconds,time.monotonic(),end['pose'],initial['reference_origin'],initial['reference_heading'],True)
        result.update(source_trial=str(log),comparison_scope='CURRENT_MAP_VS_RECORDED_ENDPOINT_NOT_HISTORICAL_PROOF',control_connected=False)
        out=root/'logs'/('rejoin_map_audit_'+str(time.time_ns())+'.json');out.write_text(json.dumps(result,allow_nan=False)+'\n')
        print(json.dumps({k:v for k,v in result.items() if k!='live_path_points'},ensure_ascii=False,indent=2));print('JSON='+str(out))
    finally:node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
