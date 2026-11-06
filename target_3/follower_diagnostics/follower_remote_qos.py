"""상태/preview는 최신 값 우선. 실제 모터 명령·arm/disarm QoS에는 사용하지 않는다."""


def latest_state_qos():
    from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy,HistoryPolicy
    return QoSProfile(history=HistoryPolicy.KEEP_LAST,depth=1,
                      reliability=ReliabilityPolicy.BEST_EFFORT,
                      durability=DurabilityPolicy.VOLATILE)


def latest_remote_qos():
    return latest_state_qos()


def measurement_history_qos():
    """Retain intermediate timestamped measurements for pose interpolation."""
    profile=latest_state_qos()
    profile.depth=10
    return profile
