from __future__ import annotations

import logging
import threading
import time
from collections import deque
from collections.abc import Callable
from dataclasses import dataclass
from enum import Enum
from typing import Protocol

from .protocol import MessageType, Packet, ProtocolError, StreamDecoder, encode_packet

LOG = logging.getLogger(__name__)


class TransportError(RuntimeError):
    pass


class TransportClosed(TransportError):
    pass


class TxOutcome(str, Enum):
    SENT = "sent"
    SUPERSEDED = "superseded"
    PREEMPTED_BY_SAFETY = "preempted_by_safety"
    FAILED = "failed"


@dataclass(frozen=True)
class TransportTxUpdate:
    sequence: int
    outcome: TxOutcome
    enqueued_ns: int
    started_ns: int = 0
    finished_ns: int = 0
    detail: str | None = None


@dataclass(frozen=True)
class TransportDiagnostics:
    estop_depth: int
    safety_depth: int
    reliable_tx_depth: int
    target_pending: bool
    reliable_rx_depth: int
    state_pending: bool
    diagnostics_pending: bool
    estop_high_watermark: int
    safety_high_watermark: int
    reliable_tx_high_watermark: int
    reliable_rx_high_watermark: int
    target_superseded: int
    target_preempted_by_safety: int
    state_overwritten: int
    diagnostics_overwritten: int
    reliable_rx_overflow: int
    max_safety_wait_ns: int


@dataclass(frozen=True)
class _TxItem:
    sequence: int | None
    message_type: MessageType | None
    frame: bytes
    enqueued_ns: int


class PacketTransport(Protocol):
    is_simulated: bool

    def open(self) -> None: ...
    def close(self) -> None: ...
    def send(self, packet: Packet) -> None: ...
    def receive(self, timeout: float | None = None) -> Packet | None: ...


_SAFETY_TYPES = {
    MessageType.ESTOP,
    MessageType.HOLD,
    MessageType.RELEASE_CONTROL,
    MessageType.SET_MODE,
}
_MOTION_FLUSH_TYPES = {MessageType.ESTOP, MessageType.HOLD}


class SerialTransport:
    """Priority-aware threaded USB CDC transport.

    Safety/control packets are reliable. Motion targets use one latest-value
    mailbox so an old target can never delay HOLD or ESTOP. STATE telemetry is
    also latest-value while ACK/NACK/EVENT packets are never silently dropped.
    """

    is_simulated = False

    def __init__(
        self,
        port: str,
        baudrate: int = 115_200,
        *,
        read_timeout_s: float = 0.05,
        rx_queue_size: int = 128,
        tx_queue_size: int = 32,
        max_consecutive_invalid_frames: int = 3,
    ) -> None:
        if min(rx_queue_size, tx_queue_size, max_consecutive_invalid_frames) <= 0:
            raise ValueError("transport queue sizes and invalid-frame limit must be positive")
        self.port = port
        self.baudrate = baudrate
        self.read_timeout_s = read_timeout_s
        self.max_consecutive_invalid_frames = max_consecutive_invalid_frames
        self._rx_capacity = rx_queue_size
        self._tx_capacity = tx_queue_size
        self._rx_reliable: deque[Packet] = deque()
        self._latest_state: Packet | None = None
        self._latest_diagnostics: Packet | None = None
        self._rx_error: BaseException | None = None
        self._rx_condition = threading.Condition()
        self._estop_tx: deque[_TxItem] = deque()
        self._safety_tx: deque[_TxItem] = deque()
        self._reliable_tx: deque[_TxItem] = deque()
        self._target_tx: _TxItem | None = None
        self._tx_condition = threading.Condition()
        self._tx_observer: Callable[[TransportTxUpdate], None] | None = None
        self._stop = threading.Event()
        self._threads: list[threading.Thread] = []
        self._serial = None
        self.decoder = StreamDecoder()
        self._estop_high_watermark = 0
        self._safety_high_watermark = 0
        self._reliable_tx_high_watermark = 0
        self._reliable_rx_high_watermark = 0
        self._target_superseded = 0
        self._target_preempted = 0
        self._state_overwritten = 0
        self._diagnostics_overwritten = 0
        self._reliable_rx_overflow = 0
        self._max_safety_wait_ns = 0

    def open(self) -> None:
        if self._threads:
            return
        try:
            import serial
        except ImportError as exc:
            raise TransportError("pyserial is required for USB CDC") from exc
        try:
            self._serial = serial.Serial(
                self.port,
                self.baudrate,
                timeout=self.read_timeout_s,
                write_timeout=self.read_timeout_s,
            )
            self._serial.reset_input_buffer()
            self._serial.reset_output_buffer()
        except serial.SerialException as exc:
            if self._serial is not None:
                self._serial.close()
                self._serial = None
            raise TransportError(f"cannot open serial port {self.port}: {exc}") from exc
        self.decoder = StreamDecoder()
        with self._rx_condition:
            self._rx_reliable.clear()
            self._latest_state = None
            self._latest_diagnostics = None
            self._rx_error = None
        with self._tx_condition:
            self._estop_tx.clear()
            self._safety_tx.clear()
            self._reliable_tx.clear()
            self._target_tx = None
        self._stop.clear()
        self._threads = [
            threading.Thread(target=self._read_loop, name="dummy-serial-rx", daemon=True),
            threading.Thread(target=self._write_loop, name="dummy-serial-tx", daemon=True),
        ]
        for thread in self._threads:
            thread.start()

    def close(self) -> None:
        self._stop.set()
        with self._tx_condition:
            self._tx_condition.notify_all()
        with self._rx_condition:
            self._rx_condition.notify_all()
        for thread in self._threads:
            thread.join(timeout=1.0)
        self._threads.clear()
        if self._serial is not None:
            self._serial.close()
            self._serial = None

    def send(self, packet: Packet) -> None:
        if self._serial is None or self._stop.is_set():
            raise TransportClosed("serial transport is not open")
        item = _TxItem(
            packet.sequence,
            packet.message_type,
            encode_packet(packet),
            time.monotonic_ns(),
        )
        displaced: tuple[_TxItem, TxOutcome] | None = None
        with self._tx_condition:
            if packet.message_type == MessageType.SET_JOINT_TARGET:
                if self._target_tx is not None:
                    displaced = (self._target_tx, TxOutcome.SUPERSEDED)
                    self._target_superseded += 1
                self._target_tx = item
            elif packet.message_type in _SAFETY_TYPES:
                target_queue = (
                    self._estop_tx
                    if packet.message_type == MessageType.ESTOP
                    else self._safety_tx
                )
                if len(target_queue) >= self._tx_capacity:
                    raise TransportError("serial safety queue is full")
                if (
                    packet.message_type in _MOTION_FLUSH_TYPES
                    and self._target_tx is not None
                ):
                    displaced = (self._target_tx, TxOutcome.PREEMPTED_BY_SAFETY)
                    self._target_tx = None
                    self._target_preempted += 1
                if packet.message_type == MessageType.ESTOP:
                    self._estop_tx.append(item)
                    self._estop_high_watermark = max(
                        self._estop_high_watermark, len(self._estop_tx)
                    )
                else:
                    self._safety_tx.append(item)
                self._safety_high_watermark = max(
                    self._safety_high_watermark, len(self._safety_tx)
                )
            else:
                if len(self._reliable_tx) >= self._tx_capacity:
                    raise TransportError("serial reliable control queue is full")
                self._reliable_tx.append(item)
                self._reliable_tx_high_watermark = max(
                    self._reliable_tx_high_watermark, len(self._reliable_tx)
                )
            if displaced is not None:
                # Notify while the re-entrant TX lock is still held.  The
                # robot's SUPERSEDED callback re-enters send() to enqueue HOLD;
                # keeping this atomic prevents the writer from taking the new
                # target in the gap between replacement and safety preemption.
                old, outcome = displaced
                self._notify_tx(
                    TransportTxUpdate(
                        old.sequence or 0,
                        outcome,
                        old.enqueued_ns,
                        finished_ns=time.monotonic_ns(),
                    )
                )
            self._tx_condition.notify()

    def send_raw_frame_for_fault_injection(self, frame: bytes) -> None:
        if self._serial is None or self._stop.is_set():
            raise TransportClosed("serial transport is not open")
        if not frame or not frame.endswith(b"\x00") or len(frame) > 600:
            raise ValueError("fault-injection frame must be bounded and zero-delimited")
        item = _TxItem(None, None, bytes(frame), time.monotonic_ns())
        with self._tx_condition:
            if len(self._reliable_tx) >= self._tx_capacity:
                raise TransportError("serial reliable control queue is full")
            self._reliable_tx.append(item)
            self._tx_condition.notify()

    def receive(self, timeout: float | None = None) -> Packet | None:
        deadline = None if timeout is None else time.monotonic() + timeout
        with self._rx_condition:
            while True:
                if self._rx_error is not None:
                    error = self._rx_error
                    self._rx_error = None
                    raise TransportError(str(error)) from error
                if self._rx_reliable:
                    return self._rx_reliable.popleft()
                if self._latest_diagnostics is not None:
                    diagnostics = self._latest_diagnostics
                    self._latest_diagnostics = None
                    return diagnostics
                if self._latest_state is not None:
                    state = self._latest_state
                    self._latest_state = None
                    return state
                if self._stop.is_set():
                    return None
                remaining = None if deadline is None else deadline - time.monotonic()
                if remaining is not None and remaining <= 0:
                    return None
                self._rx_condition.wait(remaining)

    def set_tx_observer(
        self, observer: Callable[[TransportTxUpdate], None] | None
    ) -> None:
        self._tx_observer = observer

    def diagnostics(self) -> TransportDiagnostics:
        with self._tx_condition, self._rx_condition:
            return TransportDiagnostics(
                estop_depth=len(self._estop_tx),
                safety_depth=len(self._safety_tx),
                reliable_tx_depth=len(self._reliable_tx),
                target_pending=self._target_tx is not None,
                reliable_rx_depth=len(self._rx_reliable),
                state_pending=self._latest_state is not None,
                diagnostics_pending=self._latest_diagnostics is not None,
                estop_high_watermark=self._estop_high_watermark,
                safety_high_watermark=self._safety_high_watermark,
                reliable_tx_high_watermark=self._reliable_tx_high_watermark,
                reliable_rx_high_watermark=self._reliable_rx_high_watermark,
                target_superseded=self._target_superseded,
                target_preempted_by_safety=self._target_preempted,
                state_overwritten=self._state_overwritten,
                diagnostics_overwritten=self._diagnostics_overwritten,
                reliable_rx_overflow=self._reliable_rx_overflow,
                max_safety_wait_ns=self._max_safety_wait_ns,
            )

    def _notify_tx(self, update: TransportTxUpdate) -> None:
        observer = self._tx_observer
        if observer is not None and update.sequence != 0:
            observer(update)

    def _publish(self, item: Packet | BaseException) -> None:
        with self._rx_condition:
            if isinstance(item, BaseException):
                self._rx_error = item
            elif item.message_type == MessageType.STATE:
                if self._latest_state is not None:
                    self._state_overwritten += 1
                self._latest_state = item
            elif item.message_type == MessageType.CAN_DIAGNOSTICS:
                if self._latest_diagnostics is not None:
                    self._diagnostics_overwritten += 1
                self._latest_diagnostics = item
            elif len(self._rx_reliable) >= self._rx_capacity:
                self._reliable_rx_overflow += 1
                self._rx_error = TransportError(
                    "reliable serial receive queue overflow; no ACK/NACK/EVENT was dropped"
                )
                self._stop.set()
            else:
                self._rx_reliable.append(item)
                self._reliable_rx_high_watermark = max(
                    self._reliable_rx_high_watermark, len(self._rx_reliable)
                )
            self._rx_condition.notify_all()

    def _read_loop(self) -> None:
        assert self._serial is not None
        consecutive_invalid_frames = 0
        try:
            while not self._stop.is_set():
                data = self._serial.read(256)
                if not data:
                    continue
                dropped_before = self.decoder.dropped_frames
                packets = self.decoder.feed(data)
                dropped = self.decoder.dropped_frames - dropped_before
                for packet in packets:
                    self._publish(packet)
                if packets:
                    consecutive_invalid_frames = 0
                elif dropped:
                    consecutive_invalid_frames += dropped
                    LOG.warning(
                        "discarded %d invalid serial frame(s) while resynchronizing "
                        "(%d consecutive, %d total)",
                        dropped,
                        consecutive_invalid_frames,
                        self.decoder.dropped_frames,
                    )
                    if consecutive_invalid_frames > self.max_consecutive_invalid_frames:
                        raise ProtocolError(
                            "too many consecutive invalid serial frames "
                            f"({consecutive_invalid_frames}); stopping the host link"
                        )
        except BaseException as exc:
            if not self._stop.is_set():
                self._publish(exc)

    def _next_tx(self) -> _TxItem | None:
        with self._tx_condition:
            while not self._stop.is_set():
                if self._estop_tx:
                    return self._estop_tx.popleft()
                if self._safety_tx:
                    return self._safety_tx.popleft()
                if self._reliable_tx:
                    return self._reliable_tx.popleft()
                if self._target_tx is not None:
                    item = self._target_tx
                    self._target_tx = None
                    return item
                self._tx_condition.wait(0.1)
        return None

    def _write_loop(self) -> None:
        assert self._serial is not None
        try:
            while not self._stop.is_set():
                item = self._next_tx()
                if item is None:
                    continue
                started_ns = time.monotonic_ns()
                if item.message_type in _SAFETY_TYPES:
                    self._max_safety_wait_ns = max(
                        self._max_safety_wait_ns, started_ns - item.enqueued_ns
                    )
                error: str | None = None
                try:
                    written = self._serial.write(item.frame)
                    if written != len(item.frame):
                        raise TransportError(
                            "partial serial write: "
                            f"{written}/{len(item.frame)} byte(s) accepted"
                        )
                except BaseException as exc:
                    error = str(exc)
                    raise
                finally:
                    if item.sequence is not None:
                        self._notify_tx(
                            TransportTxUpdate(
                                item.sequence,
                                TxOutcome.SENT if error is None else TxOutcome.FAILED,
                                item.enqueued_ns,
                                started_ns,
                                time.monotonic_ns(),
                                error,
                            )
                        )
        except BaseException as exc:
            if not self._stop.is_set():
                self._publish(exc)
