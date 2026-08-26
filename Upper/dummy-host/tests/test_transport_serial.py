from __future__ import annotations

import pytest

from dummy_host.domain import ActionStage
from dummy_host.protocol import MessageType, Packet, encode_packet
from dummy_host.robot_driver import DummyRobot
from dummy_host.transport_serial import (
    SerialTransport,
    TransportError,
    TxOutcome,
)


class _ChunkSerial:
    def __init__(self, transport: SerialTransport, chunks: list[bytes]) -> None:
        self._transport = transport
        self._chunks = iter(chunks)

    def read(self, size: int) -> bytes:
        try:
            return next(self._chunks)
        except StopIteration:
            self._transport._stop.set()
            return b""


def test_serial_reader_resynchronizes_after_one_partial_frame() -> None:
    transport = SerialTransport("unused")
    packet = Packet(MessageType.HELLO_ACK, 7, 3, 11, b"payload")
    transport._serial = _ChunkSerial(
        transport,
        [b"tail-from-an-old-frame\x00", encode_packet(packet)],
    )
    transport._read_loop()
    assert transport.receive(timeout=0) == packet
    assert transport.decoder.dropped_frames == 1


def test_serial_reader_stops_after_persistent_invalid_frames() -> None:
    transport = SerialTransport("unused", max_consecutive_invalid_frames=3)
    transport._serial = _ChunkSerial(transport, [b"\x01\x00" * 4])
    transport._read_loop()
    with pytest.raises(TransportError, match="too many consecutive invalid serial frames"):
        transport.receive(timeout=0)


def test_serial_transport_rejects_zero_invalid_frame_budget() -> None:
    with pytest.raises(ValueError, match="must be positive"):
        SerialTransport("unused", max_consecutive_invalid_frames=0)


def _opened_transport(**kwargs) -> SerialTransport:
    transport = SerialTransport("unused", **kwargs)
    transport._serial = object()
    transport._stop.clear()
    return transport


def test_tx_safety_priority_and_estop_order_preempt_motion_mailbox() -> None:
    transport = _opened_transport()
    updates = []
    transport.set_tx_observer(updates.append)
    target = Packet(MessageType.SET_JOINT_TARGET, 1, 10, 1, b"target")
    heartbeat = Packet(MessageType.HEARTBEAT, 1, 11, 2)
    hold = Packet(MessageType.HOLD, 1, 12, 3)
    estop = Packet(MessageType.ESTOP, 1, 13, 4)

    transport.send(heartbeat)
    transport.send(target)
    transport.send(hold)
    transport.send(estop)

    assert updates[-1].sequence == target.sequence
    assert updates[-1].outcome is TxOutcome.PREEMPTED_BY_SAFETY
    assert transport._next_tx().message_type is MessageType.ESTOP
    assert transport._next_tx().message_type is MessageType.HOLD
    assert transport._next_tx().message_type is MessageType.HEARTBEAT
    diagnostics = transport.diagnostics()
    assert diagnostics.target_preempted_by_safety == 1
    assert not diagnostics.target_pending


def test_latest_target_mailbox_reports_exact_superseded_sequence() -> None:
    transport = _opened_transport()
    updates = []
    transport.set_tx_observer(updates.append)
    first = Packet(MessageType.SET_JOINT_TARGET, 1, 0xFFFFFFFF, 1, b"first")
    wrapped = Packet(MessageType.SET_JOINT_TARGET, 1, 1, 2, b"wrapped")

    transport.send(first)
    transport.send(wrapped)

    assert [(update.sequence, update.outcome) for update in updates] == [
        (0xFFFFFFFF, TxOutcome.SUPERSEDED)
    ]
    queued = transport._next_tx()
    assert queued is not None and queued.sequence == 1
    assert transport.diagnostics().target_superseded == 1


def test_state_flood_never_evicts_reliable_ack_or_nack() -> None:
    transport = SerialTransport("unused", rx_queue_size=2)
    state1 = Packet(MessageType.STATE, 1, 1, 1, b"first")
    state2 = Packet(MessageType.STATE, 1, 2, 2, b"latest")
    ack = Packet(MessageType.ACK, 1, 7, 3, b"ack")
    nack = Packet(MessageType.NACK, 1, 8, 4, b"nack")

    transport._publish(state1)
    transport._publish(state2)
    transport._publish(ack)
    transport._publish(nack)

    assert transport.receive(timeout=0) is ack
    assert transport.receive(timeout=0) is nack
    assert transport.receive(timeout=0) is state2
    assert transport.diagnostics().state_overwritten == 1


def test_diagnostics_are_latest_value_without_delaying_reliable_events() -> None:
    transport = SerialTransport("unused", rx_queue_size=2)
    first = Packet(MessageType.CAN_DIAGNOSTICS, 1, 1, 1, b"first")
    latest = Packet(MessageType.CAN_DIAGNOSTICS, 1, 2, 2, b"latest")
    event = Packet(MessageType.EVENT, 1, 3, 3, b"event")

    transport._publish(first)
    transport._publish(latest)
    transport._publish(event)

    assert transport.receive(timeout=0) is event
    assert transport.receive(timeout=0) is latest
    diagnostics = transport.diagnostics()
    assert diagnostics.diagnostics_overwritten == 1
    assert not diagnostics.diagnostics_pending


def test_reliable_rx_overflow_is_fatal_without_dropping_old_packet() -> None:
    transport = SerialTransport("unused", rx_queue_size=1)
    first = Packet(MessageType.ACK, 1, 1, 1, b"ack")
    second = Packet(MessageType.EVENT, 1, 2, 2, b"event")
    transport._publish(first)
    transport._publish(second)

    with pytest.raises(TransportError, match="reliable serial receive queue overflow"):
        transport.receive(timeout=0)
    assert list(transport._rx_reliable) == [first]
    assert transport.diagnostics().reliable_rx_overflow == 1


def test_host_mailbox_supersede_immediately_injects_priority_hold(config) -> None:
    transport = _opened_transport()
    robot = DummyRobot(config, transport)
    lifecycle = []
    robot.set_action_lifecycle_listener(lifecycle.append)
    for sequence in (10, 11):
        robot._prepare_action_sequence(sequence)
        robot._emit_action_stage(sequence, ActionStage.RECEIVED, sequence)

    transport.send(Packet(MessageType.SET_JOINT_TARGET, 1, 10, 1, b"old"))
    transport.send(Packet(MessageType.SET_JOINT_TARGET, 1, 11, 2, b"new"))

    assert any(
        update.sequence == 10 and update.stage is ActionStage.SUPERSEDED
        for update in lifecycle
    )
    assert any(
        update.sequence == 11
        and update.stage is ActionStage.PREEMPTED_BY_SAFETY
        for update in lifecycle
    )
    queued = transport._next_tx()
    assert queued is not None and queued.message_type is MessageType.HOLD
    assert not transport.diagnostics().target_pending
