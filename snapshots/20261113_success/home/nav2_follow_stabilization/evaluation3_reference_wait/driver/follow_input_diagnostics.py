"""Read-only source timestamps used by the Nav2 follower preflight."""
import time


def snapshot(samples, now_ns, now_mono):
    result = {}
    for name in ('follower', 'filtered', 'position', 'imu', 'scan', 'raw', 'leader'):
        row = samples.get(name)
        if row is None:
            result[name] = {'received': False}
            continue
        msg, rx = row
        stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        result[name] = dict(received=True, source_ns=stamp,
                            source_age_s=(now_ns-stamp)/1e9,
                            receive_age_s=now_mono-rx,
                            frame=msg.header.frame_id,
                            child_frame=getattr(msg, 'child_frame_id', None))
    return result


def main():
    import json
    import rclpy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu, LaserScan
    from tf2_msgs.msg import TFMessage
    from rclpy.qos import qos_profile_sensor_data
    rclpy.init()
    node = rclpy.create_node('follow_input_readonly')
    samples = {}
    counts = {}
    tf_edges = {}
    def receive(name, msg):
        samples[name] = (msg, time.monotonic())
        counts[name] = counts.get(name, 0) + 1
    subscriptions = []
    def receive_tf(msg):
        now_ns = node.get_clock().now().nanoseconds
        for transform in msg.transforms:
            parent, child = transform.header.frame_id, transform.child_frame_id
            if child not in ('follower/odom', 'follower/base_link', 'odom', 'base_link'):
                continue
            key = parent+' -> '+child
            previous = tf_edges.get(key, {})
            stamp = transform.header.stamp.sec*10**9+transform.header.stamp.nanosec
            tf_edges[key] = dict(count=previous.get('count', 0)+1,
                                 source_ns=stamp, received_ns=now_ns)
    subscriptions.append(node.create_subscription(
        TFMessage, '/tf', receive_tf, qos_profile_sensor_data))
    for name, topic in (('follower', '/follower/odom_map'),
                        ('filtered', '/follower/odometry/filtered'),
                        ('position', '/follower/odom_position_candidate'),
                        ('raw', '/follower/odom_raw'), ('leader', '/leader/odom')):
        subscriptions.append(node.create_subscription(
            Odometry, topic, lambda m, n=name: receive(n, m), qos_profile_sensor_data))
    for name, topic, kind in (('imu', '/follower/imu/data', Imu),
                              ('scan', '/follower/scan', LaserScan)):
        subscriptions.append(node.create_subscription(
            kind, topic, lambda m, n=name: receive(n, m), qos_profile_sensor_data))
    try:
        end = time.monotonic() + 8.
        print('8초 조회 전용: 위치 시각만 수신합니다. 주행·arm 요청 없음.', flush=True)
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.05)
        now_ns = node.get_clock().now().nanoseconds
        for row in tf_edges.values():
            row['source_age_s'] = (now_ns-row['source_ns'])/1e9
            row['receive_age_s'] = (now_ns-row['received_ns'])/1e9
        def endpoint(row):
            return dict(node=row.node_name, namespace=row.node_namespace,
                        reliability=row.qos_profile.reliability.name,
                        durability=row.qos_profile.durability.name)
        tf_graph = dict(publishers=[endpoint(r) for r in node.get_publishers_info_by_topic('/tf')],
                        amcl_subscribers=[endpoint(r) for r in node.get_subscriptions_info_by_topic('/tf')
                                          if r.node_namespace=='/follower' or 'amcl' in r.node_name])
        print(json.dumps(dict(counts=counts, tf_edges=tf_edges, tf_graph=tf_graph, inputs=snapshot(
            samples, now_ns, time.monotonic())),
            ensure_ascii=False, indent=2), flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
