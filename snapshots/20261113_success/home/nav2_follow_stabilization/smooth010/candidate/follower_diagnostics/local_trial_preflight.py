"""Pure preflight checks shared by the trial and its offline tests."""
import math


def _names(endpoints):
    return sorted(getattr(endpoint, 'node_name', '<unknown>') for endpoint in endpoints)


def _same_participant(left, right):
    """Resolve missing remote node labels only via a known DDS participant."""
    left_gid = getattr(left, 'endpoint_gid', None)
    right_gid = getattr(right, 'endpoint_gid', None)
    if left_gid is None or right_gid is None:
        return False
    try:
        # Fast DDS exposes a 24-byte ROS GID; the first 14 bytes identify the
        # participant, while the endpoint entity ID follows.
        return bytes(left_gid[:14]) == bytes(right_gid[:14])
    except (TypeError, ValueError):
        return False


def graph_issues(node, *, nav2_leader, command_topic, node_name, plan_only,
                 pose_source, follow_only=False):
    """Return explicit ROS graph mismatches; an empty list means ready."""
    issues = []

    def check(topic, direction, expected):
        getter = (node.get_publishers_info_by_topic if direction == 'publisher'
                  else node.get_subscriptions_info_by_topic)
        endpoints = getter(topic)
        actual = _names(endpoints)
        if expected == ['nav2_convoy_gate'] and len(endpoints) == 1 \
                and actual == ['_NODE_NAME_UNKNOWN_']:
            # Some Humble discovery snapshots omit only the remote node label.
            # Require the expected gate participant to be independently named
            # on its /leader/odom subscription and to own the same DDS participant.
            gate_anchors = [endpoint for endpoint in
                node.get_subscriptions_info_by_topic('/leader/odom')
                if getattr(endpoint, 'node_name', None) == 'nav2_convoy_gate']
            if any(_same_participant(endpoints[0], anchor)
                   for anchor in gate_anchors):
                return
        if actual != expected:
            issues.append({
                'topic': topic,
                'endpoint': direction,
                'expected_nodes': expected,
                'actual_nodes': actual,
            })

    if nav2_leader:
        check('/cmd_vel', 'publisher', ['nav2_convoy_gate'])
        check('/cmd_vel', 'subscription', ['md200t_controller'])
        check('/cmd_vel_smoothed', 'publisher', ['velocity_smoother'])
        check('/cmd_vel_smoothed', 'subscription', ['nav2_convoy_gate'])
        if getattr(node, '_require_nav2_plan', False):
            check('/received_global_plan', 'publisher', ['controller_server'])
        if getattr(node, '_require_nav2_goal_status', False):
            if getattr(node, '_use_direct_unicast', False):
                check('/leader/nav2_goal_status_unicast', 'publisher', ['follower_unicast_relay'])
            else:
                check('/navigate_to_pose/_action/status', 'publisher', ['bt_navigator'])
        check(command_topic, 'publisher', [node_name])
        check(command_topic, 'subscription', ['follower_drive_candidate'])
    else:
        check('/cmd_vel', 'publisher', [] if plan_only else [node_name])
        check(command_topic, 'publisher', [] if plan_only else [node_name])
        check('/cmd_vel', 'subscription', ['md200t_controller'])
        check(command_topic, 'subscription', ['follower_drive_candidate'])

    if follow_only:
        # Pure Pursuit runs from shared-map odometry, Nav2 route and a fresh
        # local scan stop envelope. It does not need the APF EKF/IMU inputs.
        source_topics = ['/follower/odom_map', '/follower/scan', '/leader/odom', '/leader/path',
                         '/follower/motor_candidate/status']
    else:
        source_topics = ['/follower/odom_raw', '/follower/scan', '/leader/odom',
                         '/follower/motor_candidate/status']
    if not follow_only and pose_source == 'filtered':
        source_topics.extend(['/follower/odometry/filtered', '/follower/imu/data'])
    for topic in source_topics:
        if getattr(node, '_use_direct_unicast', False):
            topic={'/leader/odom':'/leader/odom_unicast',
                   '/leader/path':'/leader/path_unicast'}.get(topic,topic)
        endpoints = node.get_publishers_info_by_topic(topic)
        if len(endpoints) != 1:
            issues.append({
                'topic': topic,
                'endpoint': 'publisher',
                'expected_count': 1,
                'actual_nodes': _names(endpoints),
            })
    return issues


def laser_mount_issue(transform):
    """Validate the exact fixed scan mount required by scan projection."""
    if transform is None:
        return 'LASER_MOUNT_TF_MISSING'
    if (getattr(transform.header, 'frame_id', None) != 'follower/base_link'
            or getattr(transform, 'child_frame_id', None) != 'follower/laser'):
        return 'LASER_MOUNT_FRAME_MISMATCH'
    t = transform.transform.translation
    q = transform.transform.rotation
    values = (t.x - .26, t.y, t.z - .24, q.x, q.y, q.z, abs(q.w) - 1.)
    if not all(math.isfinite(value) and abs(value) < .001 for value in values):
        return 'LASER_MOUNT_VALUE_MISMATCH'
    return None


def mount_snapshot(transform):
    if transform is None:
        return None
    t = transform.transform.translation
    q = transform.transform.rotation
    return {
        'parent_frame': transform.header.frame_id,
        'child_frame': transform.child_frame_id,
        'translation_m': [t.x, t.y, t.z],
        'rotation_xyzw': [q.x, q.y, q.z, q.w],
    }
