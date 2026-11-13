"""Fresh-EKF software replay in Domain 217, localhost only; no hardware nodes.

Only /odom, /imu/data and /tf_static are played. Recorded /cmd_vel, /tf and
original filtered odometry are excluded. Default mode prints the plan only.
"""
import argparse
from datetime import datetime
import math
import os
from pathlib import Path
import signal
import sqlite3
import subprocess
import sys
import time


ROOT = Path(__file__).resolve().parent
SOURCE = ROOT / 'bags/lifted_ekf_20261026_175317'
PARAMS = Path('/home/sunuk/robot_ws/install/leader_robot_control/share/leader_robot_control/config/ekf.yaml')
INPUT_TOPICS = ('/odom', '/imu/data', '/tf_static')
OUTPUT_TOPICS = ('/odom', '/imu/data', '/tf_static', '/odometry/filtered', '/clock')
VARIANTS = ('baseline', 'finite_vx_covariance', 'vy_constraint', 'short_warmup')


def commands(source, output, params=PARAMS):
    return {
        'ekf': ['ros2', 'run', 'robot_localization', 'ekf_node', '--ros-args',
                '-r', '__node:=ekf_filter_node', '--params-file', str(params),
                '-p', 'use_sim_time:=true', '-p', 'publish_tf:=false'],
        'recorder': ['ros2', 'bag', 'record', '-s', 'sqlite3', '-o', str(output),
                     '--use-sim-time', '--include-unpublished-topics',
                     '--qos-profile-overrides-path', str(ROOT / 'lifted_bag_qos.yaml'),
                     *OUTPUT_TOPICS],
        'player': ['ros2', 'bag', 'play', str(source), '--clock', '100',
                   '--rate', '1.0', '--delay', '3', '--disable-keyboard-controls',
                   '--qos-profile-overrides-path', str(ROOT / 'lifted_bag_qos.yaml'),
                   '--topics', *INPUT_TOPICS],
    }


def validate_replay(output):
    """Reject incomplete TF/IMU comparisons before reporting displacement."""
    from rclpy.serialization import deserialize_message
    from tf2_msgs.msg import TFMessage
    from nav_msgs.msg import Odometry
    pairs, filtered = set(), []
    for file in output.glob('*.db3'):
        with sqlite3.connect(file.resolve().as_uri()+'?mode=ro', uri=True) as db:
            for name, cls in (('/tf_static', TFMessage), ('/odometry/filtered', Odometry)):
                topic = db.execute('select id from topics where name=?', (name,)).fetchone()
                if not topic:
                    continue
                for (raw,) in db.execute('select data from messages where topic_id=?', (topic[0],)):
                    msg = deserialize_message(raw, cls)
                    if name == '/tf_static':
                        pairs.update((t.header.frame_id, t.child_frame_id) for t in msg.transforms)
                    else:
                        filtered.append(msg)
    if ('base_link', 'imu_link') not in pairs:
        raise ValueError('Missing recorded base_link -> imu_link static TF; comparison invalid')
    if len(filtered) < 2:
        raise ValueError('Missing filtered odometry')
    max_wz = max(abs(m.twist.twist.angular.z) for m in filtered)
    headings = []
    for msg in filtered:
        q = msg.pose.pose.orientation
        headings.append(math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
    span = max(headings)-min(headings)
    # This source recording contains varying yaw/gyro. An all-zero EKF yaw and
    # yaw-rate is evidence that this particular IMU replay was not fused.
    if max_wz < 1e-5 and span < 1e-3:
        raise ValueError('No EKF angular response to varying recorded IMU; inspect TF/input handling')
    return f'REPLAY_INPUT_CHECK: static_TF={sorted(pairs)}, max_abs_wz={max_wz:.6f}, yaw_span={span:.6f}'


def stop_process(process):
    if process.poll() is not None:
        return
    for sig, seconds in ((signal.SIGINT, 10), (signal.SIGTERM, 5), (signal.SIGKILL, 3)):
        try:
            os.killpg(process.pid, sig)
            process.wait(timeout=seconds)
            return
        except ProcessLookupError:
            return
        except subprocess.TimeoutExpired:
            continue


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-software-replay', action='store_true')
    parser.add_argument('--variant', choices=VARIANTS, default='baseline')
    args = parser.parse_args()
    source, params = SOURCE, PARAMS
    if args.variant != 'baseline':
        variant = ROOT / 'ekf_replay_variants_v1' / args.variant
        source, params = variant / 'input', variant / 'ekf.yaml'
    if not args.run_software_replay:
        print('OFFLINE PLAN ONLY: no ROS processes started')
        print('Domain=217, localhost-only=1; fresh EKF; simulation clock')
        for name, cmd in commands(source, ROOT / 'bags/OUTPUT_PLACEHOLDER', params).items():
            print(name, cmd)
        return 0
    if not (source / 'metadata.yaml').is_file() or not params.is_file():
        raise SystemExit('Source bag or success EKF configuration missing')
    env = dict(os.environ, ROS_DOMAIN_ID='217', ROS_LOCALHOST_ONLY='1')
    print('Checking Domain 217 is unused on this host...', flush=True)
    check = subprocess.run(
        ['ros2', 'node', 'list', '--no-daemon', '--spin-time', '3'],
        env=env, capture_output=True, text=True, timeout=12)
    if check.returncode != 0 or check.stdout.strip():
        raise SystemExit(f'Isolated domain unavailable; nothing started. {check.stdout} {check.stderr}')
    result_dir = ROOT / 'bags' / (datetime.now().strftime('offline_ekf_replay_%Y%m%d_%H%M%S_%f')
                                + '_' + args.variant)
    result_dir.mkdir(parents=True, exist_ok=False)
    output = result_dir / 'replay_output'
    plan = commands(source, output, params)
    (result_dir / 'case.txt').write_text(
        f'variant={args.variant}\ninput={source}\nparams={params}\n')
    children, handles = {}, []
    def start(name):
        handle = (result_dir / f'{name}.log').open('w')
        handles.append(handle)
        process = subprocess.Popen(plan[name], env=env, stdout=handle,
                                   stderr=subprocess.STDOUT, start_new_session=True)
        children[name] = process
        return process
    def interrupt(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupt)
    try:
        print('Starting fresh EKF (no TF output, no hardware)...', flush=True)
        start('ekf')
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline:
            if children['ekf'].poll() is not None:
                raise RuntimeError('EKF exited during startup')
            try:
                ready = subprocess.run(
                    ['ros2', 'param', 'get', '/ekf_filter_node', 'use_sim_time'],
                    env=env, capture_output=True, text=True, timeout=6)
            except subprocess.TimeoutExpired:
                continue
            if ready.returncode == 0 and 'True' in ready.stdout:
                break
            time.sleep(0.3)
        else:
            raise RuntimeError('EKF parameter service readiness timeout')
        start('recorder')
        # Do not wait for /clock: the player will provide it after startup.
        deadline = time.monotonic() + 12
        while time.monotonic() < deadline:
            if children['recorder'].poll() is not None:
                raise RuntimeError('Recorder exited before playback')
            if list(output.glob('*.db3')):
                break
            time.sleep(0.2)
        else:
            raise RuntimeError('Recorder readiness timeout')
        print('Replaying sensor topics only (about 30 seconds)...', flush=True)
        player = start('player')
        if player.wait(timeout=90) != 0:
            raise RuntimeError('Player failed; inspect player.log')
        if children['ekf'].poll() is not None:
            raise RuntimeError('EKF exited during playback')
        time.sleep(1)
    except (RuntimeError, subprocess.TimeoutExpired, KeyboardInterrupt) as exc:
        print(f'REPLAY FAILED/INTERRUPTED: {exc}\nRESULT_DIR={result_dir}', flush=True)
        return 2
    finally:
        # Stop player before flushing the output bag. No motor commands exist here.
        for name in ('player', 'recorder', 'ekf'):
            if name in children:
                stop_process(children[name])
        for handle in handles:
            handle.close()
    try:
        validation = validate_replay(output)
        logs = '\n'.join((result_dir / f'{name}.log').read_text()
                         for name in ('ekf', 'recorder', 'player'))
        if 'incompatible QoS' in logs:
            raise ValueError('Incompatible QoS found in replay logs')
    except ValueError as exc:
        print(f'REPLAY COMPARISON INVALID: {exc}\nRESULT_DIR={result_dir}', flush=True)
        return 2
    print(validation, flush=True)
    (result_dir / 'validation.txt').write_text(validation+'\n')
    analysis = subprocess.run(
        [sys.executable, '-B', str(ROOT / 'analyze_lifted_bag.py'), str(output),
         '--reference-commands', str(SOURCE)],
        env=env, capture_output=True, text=True, timeout=30)
    (result_dir / 'analysis.txt').write_text(analysis.stdout + analysis.stderr)
    print(analysis.stdout, flush=True)
    if analysis.returncode:
        print(analysis.stderr, flush=True)
    print(f'RESULT_DIR={result_dir}', flush=True)
    return analysis.returncode


if __name__ == '__main__':
    raise SystemExit(main())
