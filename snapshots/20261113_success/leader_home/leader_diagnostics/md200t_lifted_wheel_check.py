"""Short lifted-wheel check: M1 then M2 at +5 RPM for about 0.8 s each.

No ROS nodes. Default mode only displays packets. Hardware mode requires
--run-lifted-test. Stop Nav2 and all MD200T nodes first. Keep wheels lifted and
the power cutoff accessible. This does not measure ground travel or odometry.
"""
import argparse
import signal
import struct
import subprocess
import time

import md200t_zero_feedback_check as feedback


def packet(left, right):
    if (left, right) not in ((0, 0), (5, 0), (0, 5)):
        raise ValueError('Only STOP or a single motor at +5 RPM is permitted')
    body = (bytes.fromhex('b7 b8 01 cf 07 01') + struct.pack('<h', left)
            + b'\x01' + struct.pack('<h', right) + b'\x01')
    return body + bytes([(-sum(body)) & 0xff])


def exchange(ser, left, right):
    ser.reset_input_buffer()
    tx = packet(left, right)
    if ser.write(tx) != len(tx):
        raise OSError('Incomplete serial write')
    ser.flush()
    deadline = time.monotonic() + 0.08
    buffer = bytearray()
    while time.monotonic() < deadline:
        buffer.extend(ser.read(min(max(ser.in_waiting, 1), 4096)))
        frames = feedback.extract_frames(buffer)
        if frames:
            rpm = feedback.decode_rpm(frames[-1])
            print(f'CMD=({left},{right}) FEEDBACK={rpm}', flush=True)
            return rpm
    raise TimeoutError('No valid PID216 response within 80 ms')


def confirm_stop(ser):
    # Require multiple zero-speed responses; also transmit STOP after RX errors.
    consecutive = 0
    for _ in range(5):
        try:
            rpm = exchange(ser, 0, 0)
            consecutive = consecutive + 1 if max(map(abs, rpm)) <= 2 else 0
        except (OSError, TimeoutError) as exc:
            print(f'STOP feedback error: {exc}', flush=True)
            consecutive = 0
        time.sleep(0.1)
    return consecutive >= 3


def run_test(ser):
    passed = False
    try:
        if not confirm_stop(ser):
            raise RuntimeError('Initial stationary feedback not confirmed')
        for left, right in ((5, 0), (0, 5)):
            print(f'STAGE M1={left}, M2={right}; about 0.8 seconds', flush=True)
            start = time.monotonic()
            motion_seen = False
            for _ in range(8):
                if time.monotonic() - start >= 0.8:
                    break
                tick = time.monotonic()
                rpm = exchange(ser, left, right)
                motion_seen = motion_seen or abs(rpm[0 if left else 1]) >= 1
                if max(map(abs, rpm)) > 20:
                    raise RuntimeError('Unexpected RPM magnitude')
                if abs(rpm[1 if left else 0]) > 2:
                    raise RuntimeError('Idle motor reports nonzero RPM')
                time.sleep(max(0.0, tick + 0.1 - time.monotonic()))
            if not confirm_stop(ser):
                raise RuntimeError('Stationary feedback not confirmed after pulse')
            if not motion_seen:
                raise RuntimeError('No nonzero RPM observed during pulse; motion unverified')
            time.sleep(0.5)
        passed = True
    finally:
        stopped = confirm_stop(ser)
        print(f'FINAL_STOP_CONFIRMED={stopped}', flush=True)
        if not stopped:
            print('STOP UNCONFIRMED: inspect wheels and use power cutoff.', flush=True)
        if not stopped:
            passed = False
    return 0 if passed else 2


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-lifted-test', action='store_true')
    args = parser.parse_args()
    for pair in ((0, 0), (5, 0), (0, 5)):
        print(f'ALLOWED_PACKET {pair}: {packet(*pair).hex(" ")}')
    if not args.run_lifted_test:
        print('OFFLINE ONLY: no device opened or bytes sent')
        return 0
    # Check ownership before opening the device; do not contend with ROS control.
    owners = subprocess.run(['fuser', feedback.PORT], capture_output=True, text=True)
    if owners.returncode != 1 or owners.stderr.strip():
        raise RuntimeError(f'Port not confirmed free: {owners.stdout} {owners.stderr}')
    import serial
    def interrupt(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupt)
    try:
        with serial.Serial(feedback.PORT, 57600, timeout=0.005,
                           write_timeout=0.1, exclusive=True) as ser:
            return run_test(ser)
    except (KeyboardInterrupt, OSError, RuntimeError, serial.SerialException) as exc:
        print(f'TEST INTERRUPTED/FAILED: {exc}', flush=True)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
