#!/usr/bin/env python3
"""Estimate remote-minus-local wall clock offset over one read-only SSH session."""
import argparse
import select
import shlex
import subprocess
import sys
import time


REMOTE = ('import sys,time\n'
          'for line in sys.stdin:\n'
          ' print(time.time_ns(),flush=True)\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('host', help='SSH target, e.g. sunuk@10.253.245.81')
    parser.add_argument('--interactive-auth', action='store_true',
                        help='Allow SSH password prompt on the controlling terminal; never stored')
    args = parser.parse_args()
    if args.host.startswith('-'):
        parser.error('host must not be an option')
    cmd = ['ssh', '-T', '-o', 'BatchMode=no' if args.interactive_auth else 'BatchMode=yes',
           '-o', 'NumberOfPasswordPrompts=1', '-o', 'ConnectTimeout=5',
           '-o', 'StrictHostKeyChecking=yes', '-o', 'UpdateHostKeys=no', args.host,
           'python3 -u -c ' + shlex.quote(REMOTE)]
    proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                            text=True, bufsize=1)
    samples = []
    try:
        for i in range(9):
            t0 = time.time_ns()
            mono0 = time.monotonic_ns()
            proc.stdin.write('time\n')
            proc.stdin.flush()
            auth_wait = 60 if i == 0 and args.interactive_auth else 8
            ready, _, _ = select.select([proc.stdout], [], [], auth_wait)
            if not ready:
                raise RuntimeError('SSH response timed out')
            line = proc.stdout.readline()
            t1 = time.time_ns()
            mono1 = time.monotonic_ns()
            if not line:
                raise RuntimeError('SSH closed; verify existing key authentication and host key')
            remote = int(line.strip())
            rtt = (mono1-mono0)/1e6
            if abs((t1-t0)-(mono1-mono0)) > 5_000_000:
                print(f'{i}: local clock changed during sample; rejected')
                continue
            low, high = (remote-t1)/1e6, (remote-t0)/1e6
            estimate = (low+high)/2
            label = 'startup (excluded)' if i == 0 else 'sample'
            print(f'{label} {i}: RTT={rtt:.3f}ms, remote-local estimate={estimate:+.3f}ms, '
                  f'bounds=[{low:+.3f},{high:+.3f}]ms', flush=True)
            if i:
                samples.append((rtt, estimate, low, high))
            time.sleep(.15)
        if not samples:
            raise RuntimeError('No usable samples')
        rtt, estimate, low, high = min(samples)
        print(f'\nSHORTEST RTT: {rtt:.3f}ms; leader-minus-follower estimate={estimate:+.3f}ms')
        print(f'Approximate causal bounds: [{low:+.3f},{high:+.3f}]ms')
        print('Positive means remote clock ahead. Midpoint assumes symmetric delay; not NTP calibration.')
        print('No clock/server settings, ROS nodes, motor commands or files changed.')
    except (Exception, KeyboardInterrupt) as exc:
        print(f'Clock check stopped: {exc}', file=sys.stderr)
        return 1
    finally:
        try:
            proc.stdin.close()
        except (BrokenPipeError, OSError):
            pass
        try:
            proc.wait(timeout=2)
        except subprocess.TimeoutExpired:
            proc.terminate()
            try:
                proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()
    return 0


if __name__ == '__main__':
    sys.exit(main())
