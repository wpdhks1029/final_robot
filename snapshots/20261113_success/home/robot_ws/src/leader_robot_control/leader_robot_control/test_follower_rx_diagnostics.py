import unittest

from follower_rx_diagnostics import diagnose_pid216


def make_frame():
    header = bytes([0xB8, 0xB7, 1, 0xD8, 14])
    payload = bytes(14)
    checksum = (-sum(header + payload)) & 0xFF
    return header + payload + bytes([checksum])


FRAME = make_frame()


class TestFollowerRxDiagnostics(unittest.TestCase):

    def check(self, data, expected):
        result = diagnose_pid216(data)
        self.assertEqual(result.status, expected)
        return result

    def test_no_bytes(self):
        self.check(b"", "NO_BYTES")

    def test_partial(self):
        self.check(FRAME[:10], "PARTIAL_FRAME")

    def test_header_mismatch(self):
        self.check(bytes([0x11]) * 20, "HEADER_MISMATCH")

    def test_checksum_error(self):
        corrupted = bytearray(FRAME)
        corrupted[6] ^= 0x01
        self.check(corrupted, "CHECKSUM_ERROR")

    def test_valid(self):
        result = self.check(FRAME, "VALID_PID216")
        self.assertEqual(result.frame, FRAME)

    def test_leading_noise(self):
        self.check(b"\x11\x22\x33" + FRAME, "VALID_PID216")

    def test_chunked_data(self):
        chunks = [FRAME[:7], FRAME[7:15], FRAME[15:]]
        buffer = b""

        for chunk in chunks:
            buffer += chunk

        self.check(buffer, "VALID_PID216")

    def test_corrupted_then_valid(self):
        corrupted = bytearray(FRAME)
        corrupted[6] ^= 0x01
        self.check(bytes(corrupted) + FRAME, "VALID_PID216")


if __name__ == "__main__":
    unittest.main(verbosity=2)
