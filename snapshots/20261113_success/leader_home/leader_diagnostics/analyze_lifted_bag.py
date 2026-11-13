"""Read-only SQLite bag analysis; never replay recorded motor commands."""
import argparse
import math
from pathlib import Path
import sqlite3
import statistics


def stamp_seconds(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9


def yaw(msg):
    q = msg.pose.pose.orientation
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y*q.y + q.z*q.z))


def summarize(rows, start, end):
    """Use identical header-time bounds for both odometry streams."""
    ordered = sorted(rows, key=lambda r: stamp_seconds(r[1]))
    samples = [m for _, m in ordered if start <= stamp_seconds(m) <= end]
    if len(samples) < 2:
        raise ValueError('Fewer than two messages in comparison window')
    intervals, residuals = [], []
    integral_vx = world_x = world_y = 0.0
    for a, b in zip(samples, samples[1:]):
        dt = stamp_seconds(b) - stamp_seconds(a)
        intervals.append(dt)
        if dt <= 0:
            continue
        va, vb = a.twist.twist.linear, b.twist.twist.linear
        integral_vx += (va.x + vb.x) * 0.5 * dt
        ux = ((va.x*math.cos(yaw(a)) - va.y*math.sin(yaw(a)))
              + (vb.x*math.cos(yaw(b)) - vb.y*math.sin(yaw(b)))) * 0.5 * dt
        uy = ((va.x*math.sin(yaw(a)) + va.y*math.cos(yaw(a)))
              + (vb.x*math.sin(yaw(b)) + vb.y*math.cos(yaw(b)))) * 0.5 * dt
        world_x += ux
        world_y += uy
        dx = b.pose.pose.position.x - a.pose.pose.position.x
        dy = b.pose.pose.position.y - a.pose.pose.position.y
        residuals.append((math.hypot(dx-ux, dy-uy), stamp_seconds(b), dt,
                          dx, dy, b.twist.covariance[0]))
    first, last = samples[0], samples[-1]
    dx = last.pose.pose.position.x-first.pose.pose.position.x
    dy = last.pose.pose.position.y-first.pose.pose.position.y
    return {
        'count': len(samples),
        'header_time_bounds': (stamp_seconds(first), stamp_seconds(last)),
        'rate_hz': (len(samples)-1)/(stamp_seconds(last)-stamp_seconds(first)),
        'median_dt': statistics.median(intervals), 'max_dt': max(intervals),
        'nonpositive_dt_count': sum(dt <= 0 for dt in intervals),
        'pose_delta_xy_m': (dx, dy), 'pose_net_m': math.hypot(dx, dy),
        'integral_body_vx_m': integral_vx,
        'integral_world_velocity_xy_m': (world_x, world_y),
        'pose_minus_velocity_integral_xy_m': (dx-world_x, dy-world_y),
        'max_abs_vx': max(abs(m.twist.twist.linear.x) for m in samples),
        'zero_vx_covariance_count': sum(m.twist.covariance[0] == 0 for m in samples),
        'stale_vx_covariance_count': sum(m.twist.covariance[0] >= 1000 for m in samples),
        'largest_pose_velocity_step_residuals': sorted(residuals, reverse=True)[:5],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag_directory', type=Path)
    parser.add_argument('--reference-commands', type=Path,
                        help='Read command timing from original bag without publishing commands')
    args = parser.parse_args()
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    rows = {}
    files = sorted(args.bag_directory.glob('*.db3'))
    if not files:
        raise SystemExit('No db3 files found; stop recorder before analysis')
    for file in files:
        with sqlite3.connect(file.resolve().as_uri()+'?mode=ro', uri=True) as db:
            for tid, name, typ in db.execute('select id, name, type from topics'):
                messages = db.execute(
                    'select timestamp, data from messages where topic_id=? order by timestamp',
                    (tid,)).fetchall()
                print('RECORDED', name, len(messages))
                if name not in ('/cmd_vel', '/odom', '/odometry/filtered', '/imu/data'):
                    continue
                cls = get_message(typ)
                rows.setdefault(name, []).extend(
                    (t/1e9, deserialize_message(data, cls)) for t, data in messages)
    if args.reference_commands:
        # Offline metadata comparison only. /cmd_vel is NEVER replayed.
        rows['/cmd_vel'] = []
        for file in sorted(args.reference_commands.glob('*.db3')):
            with sqlite3.connect(file.resolve().as_uri()+'?mode=ro', uri=True) as db:
                topic = db.execute(
                    "select id,type from topics where name='/cmd_vel'").fetchone()
                if topic:
                    cls = get_message(topic[1])
                    rows['/cmd_vel'].extend(
                        (t/1e9, deserialize_message(data, cls)) for t, data in db.execute(
                            'select timestamp,data from messages where topic_id=?', (topic[0],)))
        print('COMMAND_TIMING_REFERENCE_READ_ONLY', args.reference_commands)
    for name in ('/cmd_vel', '/odom', '/odometry/filtered', '/imu/data'):
        if not rows.get(name):
            raise SystemExit(f'Missing required topic: {name}')
    commands = sorted(rows['/cmd_vel'], key=lambda row: row[0])
    moving = [t for t, m in commands if abs(m.linear.x) > 1e-6 or abs(m.angular.z) > 1e-6]
    if not moving:
        raise SystemExit('No motion command recorded; recording is incomplete')
    stop = next((t for t, m in commands if t > moving[-1]
                 and abs(m.linear.x) <= 1e-6 and abs(m.angular.z) <= 1e-6), None)
    if stop is None:
        raise SystemExit('No zero command recorded after pulse')
    start = max(moving[0]-0.2, *(min(stamp_seconds(m) for _, m in rows[n])
                                for n in ('/odom', '/odometry/filtered')))
    end = min(stop+1.8, *(max(stamp_seconds(m) for _, m in rows[n])
                          for n in ('/odom', '/odometry/filtered')))
    print('COMMAND_RECEIPT first_nonzero=', moving[0], 'first_zero_after=', stop)
    print('COMMON_HEADER_TIME_WINDOW', start, end)
    for name in ('/odom', '/odometry/filtered'):
        print('\nTOPIC', name)
        for key, value in summarize(rows[name], start, end).items():
            print(key, value)
        lags = [t-stamp_seconds(m) for t, m in rows[name]]
        print('receipt_minus_header_seconds min/median/max', min(lags),
              statistics.median(lags), max(lags))
    imu = rows['/imu/data']
    print('\nIMU header_time_bounds', min(stamp_seconds(m) for _, m in imu),
          max(stamp_seconds(m) for _, m in imu))
    print('Read-only analysis finished (this analyzer never replays topics or starts nodes).')


if __name__ == '__main__':
    main()
