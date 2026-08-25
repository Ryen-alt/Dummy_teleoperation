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
    pack_target_keepalive,
    unpack_joint_target,
    unpack_target_keepalive,
    pack_state,
    unpack_state,
    PROTOCOL_VERSION,
    STATE,
    TARGET_KEEPALIVE,
)
from dummy_host.domain import ActionProgressFlags, ActionProgressRecord
from dummy_host.schema import ControlMode, RobotState


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


def test_target_keepalive_references_one_exact_action_sequence() -> None:
    assert unpack_target_keepalive(pack_target_keepalive(0xFFFFFFFF)) == 0xFFFFFFFF
    with pytest.raises(ProtocolError, match="non-zero"):
        pack_target_keepalive(0)
    with pytest.raises(ProtocolError, match="length"):
        unpack_target_keepalive(b"\x01")


def test_shared_wire_vectors() -> None:
    vectors = json.loads((Path(__file__).parents[1] / "protocol_vectors.json").read_text())
    assert vectors["protocol_version"] == PROTOCOL_VERSION
    assert vectors["decoded_sizes"]["state_payload"] == STATE.size
    assert (
        vectors["decoded_sizes"]["target_keepalive_payload"]
        == TARGET_KEEPALIVE.size
    )
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

    keepalive = vectors["vectors"][2]
    packet = Packet(
        MessageType.TARGET_KEEPALIVE,
        keepalive["session_id"],
        keepalive["sequence"],
        keepalive["sender_time_us"],
        pack_target_keepalive(keepalive["action_sequence"]),
    )
    assert encode_packet(packet).hex() == keepalive["wire_hex"]


def test_state_v4_coherent_feedback_and_exact_action_progress_round_trip(config) -> None:
    state = RobotState(
        position=np.arange(7, dtype=np.float32) / 10,
        velocity=np.arange(7, dtype=np.float32) / 100,
        monotonic_ns=123_000,
        mcu_time_us=4_000,
        mode=ControlMode.HOLD,
        fault_bits=2,
        position_valid=True,
        velocity_valid=True,
        gripper_valid=True,
        last_received_sequence=7,
        target_age_ms=12,
        config_hash=config.config_hash,
        following_error=np.arange(7, dtype=np.float32) / 1000,
        following_error_duration_ms=np.arange(7, dtype=np.uint32),
        feedback_age_ms=np.arange(7, dtype=np.uint32) + 10,
        feedback_loss_count=np.arange(7, dtype=np.uint32) + 20,
        consecutive_feedback_loss=np.arange(7, dtype=np.uint16),
        node_fault_bits=np.arange(7, dtype=np.uint16),
        node_validity=np.ones(7, dtype=np.uint8),
        hold_reason_bits=8,
        telemetry_validity=7,
        can_transport_status=0x6F,
        feedback_sample_mcu_us=np.arange(7, dtype=np.uint64) + 1000,
        feedback_sweep_id=np.full(7, 9, dtype=np.uint32),
        coherent_sweep_id=9,
        feedback_max_skew_us=20_000,
        coherent_reference_mcu_us=1_003,
        state_repeated=True,
        action_progress=(
            ActionProgressRecord(
                sequence=6,
                flags=int(ActionProgressFlags.CAN_QUEUED_EXACT)
                | int(ActionProgressFlags.POST_COMMAND_FEEDBACK),
                can_queued_mcu_us=2_000,
                post_feedback_mcu_us=3_000,
                feedback_sweep_id=9,
            ),
        ),
    )
    restored = unpack_state(pack_state(state), state.monotonic_ns)
    np.testing.assert_array_equal(restored.following_error, state.following_error)
    np.testing.assert_array_equal(restored.feedback_age_ms, state.feedback_age_ms)
    np.testing.assert_array_equal(restored.node_fault_bits, state.node_fault_bits)
    assert restored.hold_reason_bits == 8
    assert restored.telemetry_validity == 7
    assert restored.can_transport_status == 0x6F
    np.testing.assert_array_equal(
        restored.feedback_sample_mcu_us, state.feedback_sample_mcu_us
    )
    np.testing.assert_array_equal(restored.feedback_sweep_id, state.feedback_sweep_id)
    assert restored.coherent
    assert restored.feedback_max_skew_us == 20_000
    assert restored.coherent_reference_mcu_us == 1_003
    assert restored.state_repeated
    assert restored.last_can_queued_mcu_us == 2000
    assert restored.last_post_command_feedback_sequence == 6
    assert restored.last_post_command_feedback_mcu_us == 3000


def test_state_future_feedback_timestamp_is_clamped_to_state_time(config) -> None:
    state = RobotState(
        position=np.zeros(7, dtype=np.float32),
        velocity=np.zeros(7, dtype=np.float32),
        monotonic_ns=123_000,
        mcu_time_us=3_747_033_199,
        mode=ControlMode.HOLD,
        fault_bits=0,
        position_valid=True,
        velocity_valid=True,
        gripper_valid=True,
        last_received_sequence=7,
        target_age_ms=0,
        config_hash=config.config_hash,
        feedback_sample_mcu_us=np.full(
            7, np.iinfo(np.uint64).max - 10, dtype=np.uint64
        ),
        feedback_sweep_id=np.ones(7, dtype=np.uint32),
        coherent_sweep_id=1,
        coherent_reference_mcu_us=np.iinfo(np.uint64).max - 10,
    )

    restored = unpack_state(pack_state(state), state.monotonic_ns)

    np.testing.assert_array_equal(
        restored.feedback_sample_mcu_us,
        np.full(7, state.mcu_time_us, dtype=np.uint64),
    )
    assert restored.coherent_reference_mcu_us == state.mcu_time_us
