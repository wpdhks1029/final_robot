"""Prepare diagnostic bag/config copies only; no ROS nodes or hardware access."""
import hashlib
import json
from pathlib import Path
import sqlite3

import yaml
from rclpy.serialization import deserialize_message, serialize_message
from nav_msgs.msg import Odometry
from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata


ROOT = Path(__file__).resolve().parent
SOURCE = ROOT / 'bags/lifted_ekf_20261026_175317'
PARAMS = Path('/home/sunuk/robot_ws/install/leader_robot_control/share/leader_robot_control/config/ekf.yaml')
VARIANTS = ROOT / 'ekf_replay_variants_v1'
CASES = ('finite_vx_covariance', 'vy_constraint', 'short_warmup')


def main():
    if VARIANTS.exists():
        raise SystemExit(f'Already exists; refusing to overwrite: {VARIANTS}')
    rows, topics = [], {}
    for file in sorted(SOURCE.glob('*.db3')):
        with sqlite3.connect(file.resolve().as_uri()+'?mode=ro', uri=True) as db:
            mapping = {}
            for tid, name, typ, fmt, qos in db.execute(
                    'select id,name,type,serialization_format,offered_qos_profiles from topics'):
                topics[name] = (typ, fmt, qos)
                mapping[tid] = name
            rows.extend((ts, mapping[tid], bytes(data)) for tid, ts, data in db.execute(
                'select topic_id,timestamp,data from messages order by timestamp'))
    rows.sort(key=lambda row: row[0])
    first_command = next(ts for ts, name, raw in rows if name == '/cmd_vel')
    cutoff = first_command - 200_000_000
    static = next(raw for _, name, raw in rows if name == '/tf_static')
    original_yaml = PARAMS.read_bytes()
    manifest = {
        'source': str(SOURCE), 'ekf_config': str(PARAMS),
        'config_sha256': hashlib.sha256(original_yaml).hexdigest(),
        'short_warmup_cutoff_ns': cutoff,
        'cases': {},
        'note': 'Software-only hypotheses; variances are trial values, not calibrated settings.',
    }
    VARIANTS.mkdir()
    for case in CASES:
        folder = VARIANTS / case
        folder.mkdir()
        config = yaml.safe_load(original_yaml)
        if case == 'vy_constraint':
            config['ekf_filter_node']['ros__parameters']['odom0_config'][7] = True
        (folder / 'ekf.yaml').write_text(yaml.safe_dump(config, sort_keys=False))
        writer = SequentialWriter()
        writer.open(StorageOptions(uri=str(folder / 'input'), storage_id='sqlite3'),
                    ConverterOptions('cdr', 'cdr'))
        for name, (typ, fmt, qos) in topics.items():
            writer.create_topic(TopicMetadata(name=name, type=typ,
                                serialization_format=fmt, offered_qos_profiles=qos))
        modified = kept = 0
        if case == 'short_warmup':
            writer.write('/tf_static', static, cutoff)
        for ts, name, raw in rows:
            if case == 'short_warmup' and ts < cutoff:
                continue
            if name == '/odom' and case in ('finite_vx_covariance', 'vy_constraint'):
                msg = deserialize_message(raw, Odometry)
                if case == 'finite_vx_covariance' and msg.twist.covariance[0] == 0:
                    msg.twist.covariance[0] = 0.0004
                    modified += 1
                elif case == 'vy_constraint':
                    # Counterfactual nonholonomic constraint, software-only.
                    msg.twist.covariance[7] = 0.0004
                    modified += 1
                raw = serialize_message(msg)
            writer.write(name, raw, ts)
            kept += 1
        del writer  # finalize metadata before the case can be replayed
        manifest['cases'][case] = {'kept_messages': kept, 'modified_odom_messages': modified}
        print('PREPARED', case, manifest['cases'][case])
    assert PARAMS.read_bytes() == original_yaml
    (VARIANTS / 'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    print('Diagnostic copies prepared; original bag/config unchanged. No ROS processes started.')


if __name__ == '__main__':
    main()
