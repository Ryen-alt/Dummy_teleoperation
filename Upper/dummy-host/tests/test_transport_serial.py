from __future__ import annotations

from collections import deque
import json
from pathlib import Path
import threading
import time

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


class _FaultSerial:
    """Deterministic serial writer used to replay host-side USB stalls."""

    def __init__(
        self,
        *,
        write_delays_s: tuple[float, ...] = (),
        fail_write_calls: tuple[int, ...] = (),
    ) -> None:
        self._write_delays_s = deque(write_delays_s)
        self._fail_write_calls = set(fail_write_calls)
        self.write_started = threading.Event()
        self.write_calls = 0
        self.frames: list[bytes] = []

    def write(self, frame: bytes) -> int:
        call = self.write_calls
        self.write_calls += 1
        self.frames.append(bytes(frame))
        self.write_started.set()
        delay_s = self._write_delays_s.popleft() if self._write_delays_s else 0.0
        if delay_s:
            time.sleep(delay_s)
        if call in self._fail_write_calls:
            raise TimeoutError("Write timeout")
        return len(frame)


def test_serial_reader_resynchronizes_after_one_partial_frame() -> None:
    transport = SerialTransport("unused")
    packet = Packet(MessageType.HELLO_ACK, 7, 3, 11, b"payload")
    transport._serial = _ChunkSerial(
        transport,
        [b"tail-from-an-old-frame\x00", encode_packet(packet)],
    )
    transport._read_loop()
    assert transport.receive(timeout=0) == packet
    assert transport.decoder.initial_partial_frames == 1
    assert transport.decoder.dropped_frames == 0


def test_serial_reader_counts_invalid_frame_after_startup_alignment() -> None:
    transport = SerialTransport("unused")
    packet = Packet(MessageType.HELLO_ACK, 7, 3, 11, b"payload")
    transport._serial = _ChunkSerial(
        transport,
        [b"tail-from-an-old-frame\x00", b"\x01\x00", encode_packet(packet)],
    )
    transport._read_loop()
    assert transport.receive(timeout=0) == packet
    assert transport.decoder.initial_partial_frames == 1
    assert transport.decoder.dropped_frames == 1


def test_serial_reader_treats_initial_overlong_fragment_as_alignment() -> None:
    transport = SerialTransport("unused")
    packet = Packet(MessageType.HELLO_ACK, 7, 3, 11, b"payload")
    transport._serial = _ChunkSerial(
        transport,
        [b"x" * 601 + b"tail\x00", encode_packet(packet)],
    )
    transport._read_loop()
    assert transport.receive(timeout=0) == packet
    assert transport.decoder.initial_partial_frames == 1
    assert transport.decoder.dropped_frames == 0


def test_serial_reader_stops_after_persistent_invalid_frames() -> None:
    transport = SerialTransport("unused", max_consecutive_invalid_frames=3)
    transport._serial = _ChunkSerial(transport, [b"\x01\x00" * 5])
    transport._read_loop()
    with pytest.raises(TransportError, match="too many consecutive invalid serial frames"):
        transport.receive(timeout=0)


def test_serial_transport_rejects_zero_invalid_frame_budget() -> None:
    with pytest.raises(ValueError, match="must be positive"):
        SerialTransport("unused", max_consecutive_invalid_frames=0)


def test_sequence_7_fixture_reproduces_target_stuck_before_write_timeout() -> None:
    fixture_path = (
        Path(__file__).parent
        / "fixtures"
        / "sequence_7_write_timeout_lifecycle.json"
    )
    lifecycle = json.loads(fixture_path.read_text(encoding="utf-8"))

    assert lifecycle["action_sequence"] == 7
    assert lifecycle["serial_send_started_host_ns"] is not None
    assert lifecycle["serial_send_finished_host_ns"] is None
    assert lifecycle["acknowledged_host_ns"] is None
    assert lifecycle["can_tx_complete_exact_host_ns"] is None
    assert lifecycle["action_credit_miss_host_ns"] > lifecycle["serial_send_started_host_ns"]
    assert (
        lifecycle["action_credit_miss_host_ns"]
        - lifecycle["serial_send_started_host_ns"]
    ) == 46_537_006
    assert lifecycle["terminal_stage"] == "failed"
    assert lifecycle["detail"] == "Write timeout"


@pytest.mark.parametrize("blocking_ms", [20, 50, 100])
def test_blocking_usb_write_measures_motion_target_delay(blocking_ms: int) -> None:
    transport = _opened_transport()
    serial = _FaultSerial(write_delays_s=(blocking_ms / 1000.0, 0.0))
    transport._serial = serial
    updates = []
    target_finished = threading.Event()

    def observe(update) -> None:
        updates.append(update)
        if update.sequence == 99:
            target_finished.set()

    transport.set_tx_observer(observe)
    transport.send(Packet(MessageType.GET_CAN_DIAGNOSTICS, 1, 1, 1))
    writer = threading.Thread(target=transport._write_loop, daemon=True)
    writer.start()
    assert serial.write_started.wait(0.2)
    transport.send(Packet(MessageType.SET_JOINT_TARGET, 1, 99, 2, b"target"))
    assert target_finished.wait(0.5)
    transport._stop.set()
    with transport._tx_condition:
        transport._tx_condition.notify_all()
    writer.join(timeout=0.2)

    target_update = next(update for update in updates if update.sequence == 99)
    target_wait_ms = (target_update.started_ns - target_update.enqueued_ns) / 1e6
    assert target_wait_ms >= blocking_ms * 0.8


def test_write_timeout_is_reported_without_fabricating_send_completion() -> None:
    transport = _opened_transport()
    transport._serial = _FaultSerial(fail_write_calls=(0,))
    updates = []
    transport.set_tx_observer(updates.append)
    transport.send(Packet(MessageType.SET_JOINT_TARGET, 1, 7, 1, b"target"))

    writer = threading.Thread(target=transport._write_loop, daemon=True)
    writer.start()
    writer.join(timeout=0.2)

    with pytest.raises(TransportError, match="Write timeout"):
        transport.receive(timeout=0)
    assert len(updates) == 1
    assert updates[0].sequence == 7
    assert updates[0].outcome is TxOutcome.FAILED
    assert updates[0].started_ns > 0
    assert updates[0].finished_ns >= updates[0].started_ns
    diagnostics = transport.diagnostics()
    assert diagnostics.writes_enqueued == 1
    assert diagnostics.writes_started == 1
    assert diagnostics.writes_completed == 0
    assert diagnostics.writes_failed == 1
    assert diagnostics.last_write_message_type == "SET_JOINT_TARGET"
    assert diagnostics.last_write_outcome == "failed"


def test_serial_transport_uses_independent_write_timeout() -> None:
    transport = SerialTransport(
        "unused",
        read_timeout_s=0.05,
        write_timeout_s=0.02,
    )
    assert transport.read_timeout_s == 0.05
    assert transport.write_timeout_s == 0.02


def test_tx_trace_includes_type_length_and_enqueue_queue_depth() -> None:
    transport = _opened_transport()
    transport._serial = _FaultSerial()
    updates = []
    finished = threading.Event()

    def observe(update) -> None:
        updates.append(update)
        finished.set()

    transport.set_tx_observer(observe)
    packet = Packet(MessageType.SET_JOINT_TARGET, 1, 23, 1, b"target")
    transport.send(packet)
    writer = threading.Thread(target=transport._write_loop, daemon=True)
    writer.start()
    assert finished.wait(0.2)
    transport._stop.set()
    with transport._tx_condition:
        transport._tx_condition.notify_all()
    writer.join(timeout=0.2)

    assert updates[0].message_type is MessageType.SET_JOINT_TARGET
    assert updates[0].frame_length == len(encode_packet(packet))
    assert updates[0].queue_depth_at_enqueue >= 1


def test_continuous_fake_serial_pressure_has_no_loss_or_queue_deadlock() -> None:
    packet_count = 200
    transport = _opened_transport(tx_queue_size=256)
    serial = _FaultSerial()
    transport._serial = serial
    updates = []
    all_finished = threading.Event()

    def observe(update) -> None:
        updates.append(update)
        if len(updates) == packet_count:
            all_finished.set()

    transport.set_tx_observer(observe)
    writer = threading.Thread(target=transport._write_loop, daemon=True)
    writer.start()
    for sequence in range(1, packet_count + 1):
        transport.send(Packet(MessageType.HEARTBEAT, 1, sequence, sequence))
    assert all_finished.wait(1.0)
    transport._stop.set()
    with transport._tx_condition:
        transport._tx_condition.notify_all()
    writer.join(timeout=0.2)

    assert not writer.is_alive()
    assert len(serial.frames) == packet_count
    assert [update.sequence for update in updates] == list(
        range(1, packet_count + 1)
    )
    diagnostics = transport.diagnostics()
    assert diagnostics.writes_enqueued == packet_count
    assert diagnostics.writes_started == packet_count
    assert diagnostics.writes_completed == packet_count
    assert diagnostics.writes_failed == 0
    assert diagnostics.reliable_tx_depth == 0


def test_multi_usb_packet_diagnostics_does_not_hide_reliable_event() -> None:
    transport = SerialTransport("unused")
    diagnostics = encode_packet(
        Packet(MessageType.CAN_DIAGNOSTICS, 1, 40, 1, b"d" * 380)
    )
    event = Packet(MessageType.EVENT, 1, 41, 2, b"event")
    chunks = [diagnostics[index : index + 64] for index in range(0, len(diagnostics), 64)]
    chunks.append(encode_packet(event))
    transport._serial = _ChunkSerial(transport, chunks)

    transport._read_loop()

    assert transport.receive(timeout=0) == event
    decoded_diagnostics = transport.receive(timeout=0)
    assert decoded_diagnostics is not None
    assert decoded_diagnostics.message_type is MessageType.CAN_DIAGNOSTICS
    assert decoded_diagnostics.payload == b"d" * 380


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


def test_motion_target_precedes_thirty_two_pending_diagnostics_requests() -> None:
    transport = _opened_transport(tx_queue_size=32)
    for sequence in range(1, 33):
        transport.send(
            Packet(MessageType.GET_CAN_DIAGNOSTICS, 1, sequence, sequence)
        )
    target = Packet(MessageType.SET_JOINT_TARGET, 1, 99, 99, b"target")
    transport.send(target)

    queued = transport._next_tx()
    assert queued is not None
    assert queued.message_type is MessageType.SET_JOINT_TARGET


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
