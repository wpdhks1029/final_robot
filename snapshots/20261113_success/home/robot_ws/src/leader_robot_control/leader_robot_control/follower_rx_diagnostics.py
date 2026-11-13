"""MD200T PID216 offline receive diagnostics.

No serial ports are opened. No motor commands are sent.
"""

from dataclasses import dataclass

HEADER = bytes([0xB8, 0xB7])
PID216 = 0xD8
PAYLOAD_LENGTH = 14
FRAME_LENGTH = 20


@dataclass
class RxResult:
    status: str
    frame: bytes | None
    received_bytes: int


def diagnose_pid216(data: bytes, controller_id: int = 1) -> RxResult:
    data = bytes(data)
    n = len(data)

    if n == 0:
        return RxResult("NO_BYTES", None, n)

    expected_header = HEADER + bytes([
        controller_id, PID216, PAYLOAD_LENGTH
    ])

    # Search for the expected five-byte PID216 header.
    starts = [
        i for i in range(max(0, n - 4))
        if data[i:i + 5] == expected_header
    ]

    if not starts:
        # No recognizable PID216 header.
        if n < FRAME_LENGTH and data == expected_header[:n]:
            return RxResult("PARTIAL_FRAME", None, n)
        return RxResult("HEADER_MISMATCH", None, n)

    saw_incomplete = False
    saw_bad_checksum = False

    for start in starts:
        end = start + FRAME_LENGTH

        if end > n:
            saw_incomplete = True
            continue

        frame = data[start:end]

        if (sum(frame) & 0xFF) == 0:
            return RxResult("VALID_PID216", frame, n)

        saw_bad_checksum = True

    if saw_incomplete:
        return RxResult("PARTIAL_FRAME", None, n)

    if saw_bad_checksum:
        return RxResult("CHECKSUM_ERROR", None, n)

    return RxResult("HEADER_MISMATCH", None, n)
