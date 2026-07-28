from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest

from dummy_host.protocol import (
    MessageType,
    Packet,
    ProtocolError,
    StreamDecoder,
    cobs_decode,
    cobs_encode,
    crc32c,
    decode_packet,
    encode_packet,
    pack_joint_target,
    unpack_joint_target,
)


def test_crc32c_standard_vector() -> None:
    assert crc32c(b"123456789") == 0xE3069283


@pytest.mark.parametrize("value", [b"", b"\x00", b"abc", b"a\x00b\x00", bytes(range(256))])
def test_cobs_round_trip(value: bytes) -> None:
    encoded = cobs_encode(value)
    assert b"\x00" not in encoded
    assert cobs_decode(encoded) == value


def test_packet_round_trip_and_fragmented_stream() -> None:
    packet = Packet(MessageType.HEARTBEAT, 123, 456, 789, b"\x00payload")
    wire = encode_packet(packet)
    assert decode_packet(wire) == packet
    decoder = StreamDecoder()
    assert decoder.feed(wire[:5]) == []
    assert decoder.feed(wire[5:]) == [packet]


def test_crc_error_is_rejected() -> None:
    wire = bytearray(encode_packet(Packet(MessageType.HOLD, 1, 2, 3)))
    wire[-3] ^= 0x40
    with pytest.raises(ProtocolError, match="CRC"):
        decode_packet(bytes(wire))


def test_joint_target_payload() -> None:
    action = np.arange(7, dtype=np.float32) / 10
    velocity = np.ones(6, dtype=np.float32)
    restored_action, restored_velocity, ttl, flags = unpack_joint_target(
        pack_joint_target(action, velocity, 100, 3)
    )
    np.testing.assert_array_equal(restored_action, action)
    np.testing.assert_array_equal(restored_velocity, velocity)
    assert (ttl, flags) == (100, 3)


def test_shared_wire_vectors() -> None:
    vectors = json.loads((Path(__file__).parents[1] / "protocol_vectors.json").read_text())
    hello = vectors["vectors"][0]
    config_hash = bytes.fromhex(vectors["config_hash"])
    from dummy_host.protocol import pack_hello

    packet = Packet(
        MessageType.HELLO,
        hello["session_id"],
        hello["sequence"],
        hello["sender_time_us"],
        pack_hello(config_hash, hello["capabilities"]),
    )
    assert encode_packet(packet).hex() == hello["wire_hex"]

    target = vectors["vectors"][1]
    packet = Packet(
        MessageType.SET_JOINT_TARGET,
        target["session_id"],
        target["sequence"],
        target["sender_time_us"],
        pack_joint_target(
            np.asarray(target["target"], dtype=np.float32),
            np.asarray(target["max_velocity"], dtype=np.float32),
            target["valid_for_ms"],
            target["target_flags"],
        ),
    )
    assert encode_packet(packet).hex() == target["wire_hex"]
