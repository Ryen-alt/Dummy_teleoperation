from __future__ import annotations

import pytest

from dummy_host.protocol import MessageType, Packet, encode_packet
from dummy_host.transport_serial import SerialTransport, TransportError


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
