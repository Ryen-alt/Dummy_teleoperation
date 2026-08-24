from __future__ import annotations

import argparse
import json
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

from dummy_host.domain.models import HoldReasonBits
from dummy_host.fake_mcu import FakeMcuTransport
from dummy_host.protocol import (
    ACQUIRE_CONTROL,
    SET_MODE,
    MessageType,
    Packet,
    ResultCode,
    encode_packet,
    pack_hello,
    pack_joint_target,
    unpack_ack,
    unpack_state,
)
from dummy_host.schema import ControlMode, RobotConfig, RobotState, load_robot_config
from dummy_host.transport_serial import PacketTransport, SerialTransport


REAL_ACKNOWLEDGEMENT = "I_ACCEPT_REAL_ROBOT_HOLD"
SCENARIOS = (
    "pause-target",
    "pause-heartbeat",
    "duplicate",
    "out-of-order",
    "corrupt",
)


class FaultInjectionError(RuntimeError):
    pass


@dataclass
class MutableClock:
    now_ns: int = 1_000_000_000

    def __call__(self) -> int:
        return self.now_ns

    def advance_ms(self, milliseconds: int) -> None:
        self.now_ns += milliseconds * 1_000_000


class ProtocolHarness:
    def __init__(
        self,
        config: RobotConfig,
        transport: PacketTransport,
        *,
        clock_ns: Callable[[], int],
        advance_ms: Callable[[int], None],
        timeout_s: float = 0.75,
    ) -> None:
        self.config = config
        self.transport = transport
        self.clock_ns = clock_ns
        self.advance_ms = advance_ms
        self.timeout_s = timeout_s
        self.session_id = 0x53544550
        self.sequence = 0
        self.latest_state: RobotState | None = None

    def next_sequence(self) -> int:
        self.sequence += 1
        return self.sequence

    def packet(self, message_type: MessageType, payload: bytes = b"", *, sequence: int | None = None) -> Packet:
        return Packet(
            message_type,
            self.session_id,
            self.next_sequence() if sequence is None else sequence,
            self.clock_ns() // 1_000,
            payload,
        )

    def exchange(self, packet: Packet) -> Packet:
        self.transport.send(packet)
        deadline = time.monotonic() + self.timeout_s
        while time.monotonic() < deadline:
            response = self.transport.receive(timeout=min(0.02, deadline - time.monotonic()))
            if response is None:
                continue
            if response.message_type == MessageType.STATE:
                self.latest_state = unpack_state(response.payload, self.clock_ns())
                continue
            if response.sequence == packet.sequence:
                return response
        raise FaultInjectionError(f"timeout waiting for {packet.message_type.name}")

    def wait_state(self, predicate: Callable[[RobotState], bool]) -> RobotState:
        deadline = time.monotonic() + self.timeout_s
        while time.monotonic() < deadline:
            packet = self.transport.receive(timeout=min(0.02, deadline - time.monotonic()))
            if packet is None or packet.message_type != MessageType.STATE:
                continue
            state = unpack_state(packet.payload, self.clock_ns())
            self.latest_state = state
            if predicate(state):
                return state
        if self.latest_state is not None and predicate(self.latest_state):
            return self.latest_state
        raise FaultInjectionError("timeout waiting for expected STATE transition")

    @staticmethod
    def expect_ok(response: Packet) -> None:
        if response.message_type != MessageType.ACK or unpack_ack(response.payload).result != ResultCode.OK:
            raise FaultInjectionError("request was not acknowledged")

    def connect(self) -> None:
        self.transport.open()
        hello = self.packet(MessageType.HELLO, pack_hello(self.config.config_hash_bytes))
        response = self.exchange(hello)
        if response.message_type != MessageType.HELLO_ACK:
            raise FaultInjectionError("HELLO did not return HELLO_ACK")
        self.advance_ms(max(1, int(1000 / self.config.control_rate_hz) + 1))
        self.wait_state(lambda state: state.config_hash == self.config.config_hash)

    def acquire(self) -> None:
        self.expect_ok(
            self.exchange(
                self.packet(
                    MessageType.ACQUIRE_CONTROL,
                    ACQUIRE_CONTROL.pack(self.config.lease_timeout_ms),
                )
            )
        )
        self.expect_ok(
            self.exchange(
                self.packet(MessageType.SET_MODE, SET_MODE.pack(int(ControlMode.TELEOP)))
            )
        )

    def target_packet(self, *, sequence: int | None = None) -> Packet:
        if self.latest_state is None:
            raise FaultInjectionError("STATE is required before constructing a safe target")
        return self.packet(
            MessageType.SET_JOINT_TARGET,
            pack_joint_target(
                self.latest_state.position.copy(),
                self.config.joint_velocity_limit_rad_s,
                self.config.target_ttl_ms,
            ),
            sequence=sequence,
        )

    def safe_cleanup(self) -> None:
        try:
            self.exchange(self.packet(MessageType.HOLD))
        except BaseException:
            pass
        try:
            self.exchange(self.packet(MessageType.RELEASE_CONTROL))
        except BaseException:
            pass
        self.transport.close()


def _run_scenario(harness: ProtocolHarness, scenario: str) -> dict[str, object]:
    if scenario == "corrupt":
        packet = harness.packet(MessageType.HEARTBEAT)
        wire = bytearray(encode_packet(packet))
        wire[-3] ^= 0x40
        if isinstance(harness.transport, SerialTransport):
            harness.transport.send_raw_frame_for_fault_injection(bytes(wire))
            deadline = time.monotonic() + 0.2
            acknowledged = False
            state_retained = False
            while time.monotonic() < deadline:
                response = harness.transport.receive(timeout=0.02)
                if response is None:
                    continue
                if response.message_type == MessageType.STATE:
                    state_retained = True
                elif response.sequence == packet.sequence:
                    acknowledged = True
            return {
                "passed": not acknowledged,
                "corrupt_frame_acknowledged": acknowledged,
                "state_stream_retained": state_retained,
            }
        if isinstance(harness.transport, FakeMcuTransport):
            dropped = harness.transport.send_raw_frame_for_fault_injection(bytes(wire))
            response = harness.transport.receive(timeout=0)
            return {
                "passed": dropped == 1 and response is None,
                "dropped_frames": dropped,
                "unexpected_response": response is not None,
            }
        raise FaultInjectionError("transport does not support raw fault injection")

    harness.acquire()
    if scenario == "pause-target":
        harness.expect_ok(harness.exchange(harness.target_packet()))
        harness.advance_ms(harness.config.target_ttl_ms + 1)
        state = harness.wait_state(
            lambda value: bool(value.hold_reason_bits & int(HoldReasonBits.TARGET_TIMEOUT))
        )
        return {"passed": state.mode == ControlMode.HOLD, "hold_reason_bits": state.hold_reason_bits}
    if scenario == "pause-heartbeat":
        harness.advance_ms(harness.config.lease_timeout_ms + 1)
        state = harness.wait_state(
            lambda value: bool(value.hold_reason_bits & int(HoldReasonBits.LEASE_TIMEOUT))
        )
        return {"passed": state.mode == ControlMode.HOLD, "hold_reason_bits": state.hold_reason_bits}
    if scenario == "duplicate":
        packet = harness.target_packet()
        harness.expect_ok(harness.exchange(packet))
        response = harness.exchange(packet)
        ack = unpack_ack(response.payload)
        return {"passed": response.message_type == MessageType.NACK and ack.result == ResultCode.BAD_SEQUENCE,
                "result": ack.result.name}
    if scenario == "out-of-order":
        newer = harness.target_packet(sequence=harness.next_sequence() + 1)
        harness.sequence = newer.sequence
        harness.expect_ok(harness.exchange(newer))
        older = harness.target_packet(sequence=newer.sequence - 1)
        response = harness.exchange(older)
        ack = unpack_ack(response.payload)
        return {"passed": response.message_type == MessageType.NACK and ack.result == ResultCode.BAD_SEQUENCE,
                "result": ack.result.name}
    raise FaultInjectionError(f"unknown scenario {scenario}")


def run_fault_injection(
    config: RobotConfig,
    scenarios: tuple[str, ...],
    *,
    real_port: str | None = None,
) -> dict[str, object]:
    if not scenarios:
        raise FaultInjectionError("at least one fault-injection scenario is required")
    source = "fake" if real_port is None else "real"
    results: dict[str, object] = {}
    for scenario in scenarios:
        if real_port is None:
            clock = MutableClock()
            transport: PacketTransport = FakeMcuTransport(config, clock_ns=clock)
            advance = clock.advance_ms
        else:
            transport = SerialTransport(real_port)
            advance = lambda milliseconds: time.sleep(milliseconds / 1000.0)
        harness = ProtocolHarness(
            config,
            transport,
            clock_ns=clock if real_port is None else time.monotonic_ns,
            advance_ms=advance,
        )
        try:
            harness.connect()
            result = _run_scenario(harness, scenario)
            results[scenario] = result
            if not bool(result.get("passed")):
                raise FaultInjectionError(f"scenario {scenario} did not reach its expected result")
        finally:
            harness.safe_cleanup()
    return {"source": source, "scenarios": results, "passed": True}


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Deterministic protocol fault injection; Fake MCU is the default"
    )
    parser.add_argument("--config", required=True)
    parser.add_argument("--scenario", action="append", choices=SCENARIOS)
    parser.add_argument("--json-output")
    parser.add_argument("--execute-real", action="store_true")
    parser.add_argument("--port")
    parser.add_argument(
        "--acknowledge-real-risk",
        help=f"required literal for real mode: {REAL_ACKNOWLEDGEMENT}",
    )
    args = parser.parse_args()
    if args.execute_real:
        if not args.port:
            parser.error("--execute-real requires --port")
        if args.acknowledge_real_risk != REAL_ACKNOWLEDGEMENT:
            parser.error(
                "real mode requires --acknowledge-real-risk " + REAL_ACKNOWLEDGEMENT
            )
    elif args.port or args.acknowledge_real_risk:
        parser.error("--port/--acknowledge-real-risk are only valid with --execute-real")

    config = load_robot_config(args.config)
    scenarios = tuple(args.scenario or SCENARIOS)
    report = run_fault_injection(
        config,
        scenarios,
        real_port=args.port if args.execute_real else None,
    )
    rendered = json.dumps(report, indent=2, sort_keys=True)
    print(rendered)
    if args.json_output:
        Path(args.json_output).write_text(rendered + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
