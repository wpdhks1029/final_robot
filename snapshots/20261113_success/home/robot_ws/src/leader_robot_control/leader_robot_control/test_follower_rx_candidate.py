import ast
import time
from collections import deque
from pathlib import Path

SOURCE = Path(
    "md200t_controller_follower_rx_candidate.py"
)

tree = ast.parse(SOURCE.read_text())

method = next(
    n for n in ast.walk(tree)
    if isinstance(n, ast.FunctionDef)
    and n.name == "read_feedback_frame"
)

# Extract only the RX method.
module = ast.Module(
    body=[method],
    type_ignores=[]
)

namespace = {
    "time": time,
    "TMID": 0xB8,
    "RMID": 0xB7,
    "PID_PNT_MONITOR": 0xD8,
}

exec(
    compile(module, "<offline_rx>", "exec"),
    namespace
)

read_feedback_frame = namespace["read_feedback_frame"]


class FakeSerial:
    def __init__(self, chunks):
        self.chunks = deque(chunks)

    @property
    def in_waiting(self):
        return len(self.chunks[0]) if self.chunks else 0

    def read(self, size):
        if not self.chunks:
            return b""
        return self.chunks.popleft()


class FakeController:
    def __init__(self, chunks):
        self.ser = FakeSerial(chunks)
        self.controller_id = 1


header = bytes([0xB8, 0xB7, 1, 0xD8, 14])
payload = bytes(14)
checksum = (-sum(header + payload)) & 0xFF
frame = header + payload + bytes([checksum])

tests = [
    ("정상 프레임", [frame], True),
    ("앞쪽 잡음", [b"\x11\x22\x33", frame], True),
    ("분할 수신", [frame[:7], frame[7:15], frame[15:]], True),
    ("불완전 프레임", [frame[:10]], False),
    ("체크섬 오류", [frame[:-1] + b"\xFF"], False),
]

for name, chunks, expected in tests:
    controller = FakeController(chunks)

    result = read_feedback_frame(
        controller,
        timeout=0.03
    )

    passed = (result == frame) if expected else (result is None)

    assert passed, name
    print(f"PASS: {name}")

print("ALL FOLLOWER RX TESTS PASSED")
