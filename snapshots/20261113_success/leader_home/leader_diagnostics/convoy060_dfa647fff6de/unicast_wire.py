"""Bounded latest-state datagrams. Payloads retain their original ROS bytes."""
import struct
import zlib

HEADER = struct.Struct('!4sBQQ')
MAGIC = b'CVU1'
MAX_PACKET = 60000
MAX_RAW = 2_000_000


def encode(channel, sequence, sent_ns, payload):
    if len(payload) > MAX_RAW:
        raise ValueError('PAYLOAD_TOO_LARGE')
    packet = HEADER.pack(MAGIC, channel, sequence, sent_ns) + zlib.compress(payload, 1)
    if len(packet) > MAX_PACKET:
        raise ValueError('DATAGRAM_TOO_LARGE')
    return packet


def decode(packet, now_ns):
    if not HEADER.size < len(packet) <= MAX_PACKET:
        raise ValueError('INVALID_SIZE')
    magic, channel, seq, sent = HEADER.unpack_from(packet)
    if magic != MAGIC or sent <= 0:
        raise ValueError('INVALID_HEADER')
    if not -0.05 <= (now_ns - sent) / 1e9 <= 0.3:
        raise ValueError('STALE_OR_FUTURE_DATAGRAM')
    decoder = zlib.decompressobj()
    raw = decoder.decompress(packet[HEADER.size:], MAX_RAW + 1)
    if len(raw) > MAX_RAW or not decoder.eof or decoder.unused_data:
        raise ValueError('INVALID_COMPRESSED_PAYLOAD')
    return channel, seq, raw
