"""Read-only scan evidence serialization. Never filters control input."""
from capture_scan_anomalies import pack


def scan_evidence(before,after,raw_history,*,pose_source='raw',raw_monitor_history=None):
    frames={}
    for message,rx_ns in before+after:
        frame=pack(message,rx_ns)
        frames.setdefault((frame['source_ns'],frame['frame']),frame)
    return dict(frames=[frames[k] for k in sorted(frames)],raw_odometry_history=raw_history if pose_source=='raw' else (raw_monitor_history or []),
                pose_odometry_history=raw_history,pose_source=pose_source,
                control_input_changed=False)
