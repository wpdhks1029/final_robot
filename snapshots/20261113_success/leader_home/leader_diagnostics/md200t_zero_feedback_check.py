"""Bounded leader MD200T zero-RPM/feedback check, without ROS.

Default is offline: no serial import or device access. Hardware mode requires
--send-zero-rpm. Only the verified leader ID1/57600 configuration is supported.
"""
import argparse
import struct
import time


PORT = '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AP2472IE-if00-port0'
# PID207: M1 enable=1, RPM=0; M2 enable=1, RPM=0; return PID216.
BODY = bytes.fromhex('b7 b8 01 cf 07 01 00 00 01 00 00 01')
ZERO_PACKET = BODY + bytes([(-sum(BODY)) & 0xff])
HEADER = bytes.fromhex('b8 b7 01 d8 0e')


def extract_frames(buffer):
    """Consume checksum-valid 14-byte PID216 responses; retain partial input."""
    frames = []
    while buffer:
        if not buffer.startswith(HEADER):
            if len(buffer) < len(HEADER) and HEADER.startswith(buffer):
                break
            del buffer[0]
            continue
        if len(buffer) < 20:
            break
        frame = bytes(buffer[:20])
        if sum(frame) & 0xff:
            del buffer[0]
            continue
        frames.append(frame)
        del buffer[:20]
    return frames


def decode_rpm(frame):
    return struct.unpack_from('<h', frame, 5)[0], struct.unpack_from('<h', frame, 12)[0]


def run_probe(ser):
    tx = rx = valid = missing = 0
    buffer = bytearray()
    result = 2
    ser.reset_input_buffer()
    try:
        for index in range(15):
            started = time.monotonic()
            written = ser.write(ZERO_PACKET)
            tx += 1
            if written != len(ZERO_PACKET):
                raise OSError(f'Incomplete write: {written}/{len(ZERO_PACKET)}')
            ser.flush()
            print(f'TX_ZERO {index + 1}/15: {ZERO_PACKET.hex(" ")}', flush=True)
            received = False
            deadline = started + 0.12
            while time.monotonic() < deadline:
                chunk = ser.read(min(max(ser.in_waiting, 1), 4096))
                if not chunk:
                    continue
                rx += len(chunk)
                print(f'RX_BYTES: {chunk.hex(" ")}', flush=True)
                buffer.extend(chunk)
                for frame in extract_frames(buffer):
                    valid += 1
                    received = True
                    left, right = decode_rpm(frame)
                    print(f'VALID_PID216 M1={left} M2={right} rpm', flush=True)
                    if abs(left) > 2 or abs(right) > 2:
                        print('ABORT: nonzero feedback; verify wheels and cut power if needed.', flush=True)
                        return 3
            missing = 0 if received else missing + 1
            if missing >= 3:
                print('ABORT: three consecutive receive windows without valid PID216.', flush=True)
                return 2
            time.sleep(max(0.0, started + 0.2 - time.monotonic()))
        result = 0 if valid else 2
        print('Zero-RPM feedback check finished; distance/direction remain unverified.', flush=True)
        return result
    finally:
        print(f'SUMMARY TX_ZERO={tx} RX_BYTES={rx} VALID_PID216={valid}', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--send-zero-rpm', action='store_true')
    args = parser.parse_args()
    assert len(ZERO_PACKET) == 13 and sum(ZERO_PACKET) & 0xff == 0
    print(f'PORT={PORT} BAUD=57600 ID=1')
    print(f'ONLY_TX_PACKET={ZERO_PACKET.hex(" ")}')
    if not args.send_zero_rpm:
        print('OFFLINE ONLY: no device opened or bytes transmitted.')
        return 0
    import serial
    try:
        with serial.Serial(PORT, 57600, timeout=0.02, write_timeout=0.5,
                           exclusive=True) as ser:
            return run_probe(ser)
    except KeyboardInterrupt:
        print('Interrupted; all attempted speed commands were zero RPM.')
        return 130
    except (OSError, serial.SerialException) as exc:
        print(f'FAILED: {exc}')
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
