#!/usr/bin/env python3
"""실제 입력으로 gateway 시작 지연을 조회합니다. 발행자·서비스·모터 접근 없음."""
import argparse
import json
import time
from copy import deepcopy


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run', action='store_true')
    if not p.parse_args().run:
        p.print_help()
        return 0
    import rclpy
    from geometry_msgs.msg import TwistStamped
    from std_msgs.msg import String
    from follower_guard_gateway import Gateway, INPUT, PREFIX, OUTPUT
    rclpy.init(args=[])
    node = rclpy.create_node('gateway_startup_observer')
    core = Gateway()
    sub = node.create_subscription(TwistStamped, INPUT, lambda m: core.update_guard(m, time.monotonic()), 1)
    status = node.create_subscription(String, PREFIX+'/status', lambda m: core.update_status(m.data, time.monotonic()), 1)
    virtual_start = None
    virtual_ros = None
    first = None
    counts = {}
    ages = []
    try:
        print('조회 전용: 내부 복사본으로 시작 조건 계산. 실제 arm·명령 발행 없음.', flush=True)
        deadline = time.monotonic()+10
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
            now = node.get_clock().now().nanoseconds
            mono = time.monotonic()
            if virtual_start is None:
                if core.status is None or core.guard is None:
                    continue
                if mono-core.status[1] > .3 or mono-core.guard[1] > .3:
                    continue
                virtual_start, virtual_ros = mono, now
            shadow = deepcopy(core)
            if shadow.status is not None:
                shadow.status[0].update(armed=True, enabled=True, fault=None,
                                        feedback_fresh=True, arm_ros_ns=virtual_ros)
            envelope = shadow.envelope(now, mono)
            counts[shadow.reason] = counts.get(shadow.reason, 0)+1
            if core.guard is not None:
                ages.append((now-core.guard[0])/1e9)
            if envelope is not None and first is None:
                first = mono-virtual_start
            if mono-virtual_start >= 3:
                break
        print(json.dumps({
            'guard_publishers': [e.node_name for e in node.get_publishers_info_by_topic(INPUT)],
            'gateway_publishers': [e.node_name for e in node.get_publishers_info_by_topic(OUTPUT)],
            'drive_subscribers': [e.node_name for e in node.get_subscriptions_info_by_topic(OUTPUT)],
            'received_status': core.status is not None,
            'valid_guard_received': core.guard is not None,
            'virtual_first_eligible_delay_s': first,
            'within_0_3s': first is not None and first <= .3,
            'guard_source_age_min_max_s': [min(ages),max(ages)] if ages else None,
            'decisions': counts,
        }, indent=2), flush=True)
        print('실제 arm 없음. 가상 계산 결과만으로 구동 준비 완료를 뜻하지 않습니다.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
