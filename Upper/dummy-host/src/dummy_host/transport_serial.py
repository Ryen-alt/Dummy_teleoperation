from __future__ import annotations

import logging
import queue
import threading
from typing import Protocol

from .protocol import Packet, ProtocolError, StreamDecoder, encode_packet

LOG = logging.getLogger(__name__)


class TransportError(RuntimeError):
    pass


class TransportClosed(TransportError):
    pass


class PacketTransport(Protocol):
    is_simulated: bool

    def open(self) -> None: ...
    def close(self) -> None: ...
    def send(self, packet: Packet) -> None: ...
    def receive(self, timeout: float | None = None) -> Packet | None: ...


class SerialTransport:
    """Bounded, threaded USB CDC transport.

    The serial reader and writer never execute in the control loop. Queue overflow is
    surfaced as an error instead of silently accumulating stale targets.
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
        if max_consecutive_invalid_frames <= 0:
            raise ValueError("max_consecutive_invalid_frames must be positive")
        self.port = port
        self.baudrate = baudrate
        self.read_timeout_s = read_timeout_s
        self.max_consecutive_invalid_frames = max_consecutive_invalid_frames
        self._rx: queue.Queue[Packet | BaseException] = queue.Queue(rx_queue_size)
        self._tx: queue.Queue[bytes | None] = queue.Queue(tx_queue_size)
        self._stop = threading.Event()
        self._threads: list[threading.Thread] = []
        self._serial = None
        self.decoder = StreamDecoder()

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
            # USB CDC can retain bytes from the previous host session. The
            # firmware also streams STATE continuously, so opening the port
            # may otherwise begin halfway through a COBS frame.
            self._serial.reset_input_buffer()
            self._serial.reset_output_buffer()
        except serial.SerialException as exc:
            if self._serial is not None:
                self._serial.close()
                self._serial = None
            raise TransportError(f"cannot open serial port {self.port}: {exc}") from exc
        self.decoder = StreamDecoder()
        for pending in (self._rx, self._tx):
            while True:
                try:
                    pending.get_nowait()
                except queue.Empty:
                    break
        self._stop.clear()
        self._threads = [
            threading.Thread(target=self._read_loop, name="dummy-serial-rx", daemon=True),
            threading.Thread(target=self._write_loop, name="dummy-serial-tx", daemon=True),
        ]
        for thread in self._threads:
            thread.start()

    def close(self) -> None:
        self._stop.set()
        try:
            self._tx.put_nowait(None)
        except queue.Full:
            pass
        for thread in self._threads:
            thread.join(timeout=1.0)
        self._threads.clear()
        if self._serial is not None:
            self._serial.close()
            self._serial = None

    def send(self, packet: Packet) -> None:
        if self._serial is None or self._stop.is_set():
            raise TransportClosed("serial transport is not open")
        try:
            self._tx.put_nowait(encode_packet(packet))
        except queue.Full as exc:
            raise TransportError("serial write queue is full; refusing stale command buildup") from exc

    def send_raw_frame_for_fault_injection(self, frame: bytes) -> None:
        """Queue one pre-encoded frame for the explicitly gated fault tool only."""
        if self._serial is None or self._stop.is_set():
            raise TransportClosed("serial transport is not open")
        if not frame or not frame.endswith(b"\x00") or len(frame) > 600:
            raise ValueError("fault-injection frame must be bounded and zero-delimited")
        try:
            self._tx.put_nowait(bytes(frame))
        except queue.Full as exc:
            raise TransportError("serial write queue is full") from exc

    def receive(self, timeout: float | None = None) -> Packet | None:
        try:
            item = self._rx.get(timeout=timeout)
        except queue.Empty:
            return None
        if isinstance(item, BaseException):
            raise TransportError(str(item)) from item
        return item

    def _publish(self, item: Packet | BaseException) -> None:
        try:
            self._rx.put_nowait(item)
        except queue.Full:
            # State is periodic. Drop the oldest item to keep diagnostics current.
            try:
                self._rx.get_nowait()
                self._rx.put_nowait(item)
            except queue.Empty:
                pass

    def _read_loop(self) -> None:
        assert self._serial is not None
        consecutive_invalid_frames = 0
        try:
            while not self._stop.is_set():
                data = self._serial.read(256)
                if data:
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

    def _write_loop(self) -> None:
        assert self._serial is not None
        try:
            while not self._stop.is_set():
                try:
                    frame = self._tx.get(timeout=0.1)
                except queue.Empty:
                    continue
                if frame is None:
                    return
                self._serial.write(frame)
        except BaseException as exc:
            if not self._stop.is_set():
                self._publish(exc)
