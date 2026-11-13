def find_pid216_frame(buffer, controller_id=1):
    """수신 바이트에서 20바이트 PID216 프레임 검색."""
    for i in range(max(0, len(buffer) - 19)):
        frame = bytes(buffer[i:i + 20])

        if (
            frame[0] == 0xB8
            and frame[1] == 0xB7
            and frame[2] == controller_id
            and frame[3] == 0xD8
            and frame[4] == 0x0E
            and (sum(frame) & 0xFF) == 0
        ):
            return frame

    return None


# 실제 장치 응답이 아닌, 형식 검사용 합성 데이터
payload = bytes(14)
header = bytes([0xB8, 0xB7, 0x01, 0xD8, 0x0E])
checksum = (-sum(header + payload)) & 0xFF
valid_frame = header + payload + bytes([checksum])

tests = {
    "정상 프레임": (valid_frame, True),
    "앞에 잡음 3바이트": (b'\x11\x22\x33' + valid_frame, True),
    "불완전 프레임": (valid_frame[:10], False),
    "체크섬 오류": (valid_frame[:-1] + bytes([checksum ^ 0xFF]), False),
}

for name, (data, expected) in tests.items():
    found = find_pid216_frame(data)
    passed = (found is not None) == expected

    print(f"{'PASS' if passed else 'FAIL'}: {name}")
    assert passed, name

print("PID216 OFFLINE TEST PASSED")
