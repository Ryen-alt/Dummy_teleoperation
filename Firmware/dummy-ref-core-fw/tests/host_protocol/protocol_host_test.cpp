#include "binary_control_session.hpp"
#include "binary_protocol.hpp"
#include "binary_state_bridge.hpp"
#include "external_target_executor.hpp"
#include "joint_space_mapping.hpp"
#include "monotonic_micros.hpp"
#include "measured_state_estimator.hpp"
#include "can_feedback_monitor.hpp"
#include "feedback_safety_supervisor.hpp"
#include "feedback_poll_scheduler.hpp"
#include "published_double_buffer.hpp"
#include "robot_config_generated.hpp"
#include "spsc_ring.hpp"

#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

using namespace dummy::protocol;

namespace
{
void TestSpscRingUsesAllSlotsAndWrapsWithoutOverwrite()
{
    SpscRing<uint32_t, 32U> ring;
    assert(ring.capacity() == 32U);
    for (uint32_t value = 0U; value < 32U; ++value)
        assert(ring.Push(value));
    assert(ring.Size() == 32U);
    assert(!ring.Push(32U));

    uint32_t value = 0U;
    for (uint32_t expected = 0U; expected < 16U; ++expected)
    {
        assert(ring.Pop(value));
        assert(value == expected);
    }
    for (uint32_t next = 32U; next < 48U; ++next)
        assert(ring.Push(next));
    assert(ring.Size() == 32U);
    for (uint32_t expected = 16U; expected < 48U; ++expected)
    {
        assert(ring.Pop(value));
        assert(value == expected);
    }
    assert(!ring.Pop(value));
}

void TestPublishedDoubleBufferReturnsWholeSnapshots()
{
    struct Snapshot
    {
        uint32_t generation = 0U;
        std::array<uint32_t, 7U> values{};
    };
    PublishedDoubleBuffer<Snapshot> snapshots;
    Snapshot first{};
    first.generation = 1U;
    first.values.fill(0x11111111U);
    assert(snapshots.TryPublish(first));
    const Snapshot first_read = snapshots.Read();
    assert(first_read.generation == 1U);
    assert(first_read.values == first.values);

    Snapshot second{};
    second.generation = 2U;
    second.values.fill(0x22222222U);
    assert(snapshots.TryPublish(second));
    const Snapshot second_read = snapshots.Read();
    assert(second_read.generation == 2U);
    assert(second_read.values == second.values);
}

std::vector<uint8_t> FromHex(const std::string& hex)
{
    assert(hex.size() % 2 == 0);
    std::vector<uint8_t> output;
    output.reserve(hex.size() / 2);
    for (size_t index = 0; index < hex.size(); index += 2)
        output.push_back(static_cast<uint8_t>(std::stoul(hex.substr(index, 2), nullptr, 16)));
    return output;
}

Packet MakePacket(MessageType type, uint32_t session, uint32_t sequence)
{
    Packet packet{};
    packet.header.magic = kMagic;
    packet.header.version = kProtocolVersion;
    packet.header.message_type = static_cast<uint8_t>(type);
    packet.header.session_id = session;
    packet.header.sequence = sequence;
    return packet;
}

template<typename T>
void SetPayload(Packet& packet, const T& payload)
{
    static_assert(sizeof(T) <= kMaxPayload, "test payload too large");
    packet.header.payload_length = sizeof(T);
    std::memcpy(packet.payload.data(), &payload, sizeof(T));
}

ResultCode ResponseCode(const ProcessResult& result)
{
    AckPayload payload{};
    assert(result.response.header.payload_length == sizeof(payload));
    std::memcpy(&payload, result.response.payload.data(), sizeof(payload));
    return static_cast<ResultCode>(payload.result);
}

uint16_t ResponseDetail(const ProcessResult& result)
{
    AckPayload payload{};
    assert(result.response.header.payload_length == sizeof(payload));
    std::memcpy(&payload, result.response.payload.data(), sizeof(payload));
    return payload.detail;
}

SessionConfig MakeConfig(bool verified)
{
    SessionConfig config{};
    config.config_sha256 = dummy::generated_config::kConfigSha256;
    config.joint_min_rad = dummy::generated_config::kJointMinRad;
    config.joint_max_rad = dummy::generated_config::kJointMaxRad;
    config.max_velocity_rad_s = dummy::generated_config::kMaxVelocityRadS;
    config.hardware_parameters_verified = verified;
    config.max_lease_ms = 1000;
    config.max_target_ttl_ms = 250;
    return config;
}

Packet DecodeHelloVector()
{
    const auto wire = FromHex(
        "06594405012401013944332211887766550807060504030201"
        "cefed4f980ea7592d9108bfbc5575d1d0aebc5cf319cf41d"
        "40421a73f5043d4a5a5aa5a5150cea8a00");
    Packet packet{};
    const DecodeStatus status = DecodePacket(wire.data(), wire.size(), packet);
    if (status != DecodeStatus::Ok)
        std::cerr << "HELLO vector decode failed with status " << static_cast<int>(status) << '\n';
    assert(status == DecodeStatus::Ok);
    assert(packet.header.message_type == static_cast<uint8_t>(MessageType::Hello));
    assert(packet.header.session_id == 0x11223344U);
    assert(packet.header.sequence == 0x55667788U);
    return packet;
}

Packet MakeConfiguredHello();

void TestCodecVectors()
{
    static constexpr uint8_t kCheck[] = {'1','2','3','4','5','6','7','8','9'};
    assert(Crc32c(kCheck, sizeof(kCheck)) == 0xE3069283U);
    Packet hello = DecodeHelloVector();
    std::array<uint8_t, 600> output{};
    const size_t length = EncodePacket(hello, output.data(), output.size());
    const auto expected = FromHex(
        "06594405012401013944332211887766550807060504030201"
        "cefed4f980ea7592d9108bfbc5575d1d0aebc5cf319cf41d"
        "40421a73f5043d4a5a5aa5a5150cea8a00");
    assert(length == expected.size());
    assert(std::equal(expected.begin(), expected.end(), output.begin()));

    output[length - 3] ^= 0x40;
    Packet invalid{};
    assert(DecodePacket(output.data(), length, invalid) == DecodeStatus::BadCrc);

    const auto target_wire = FromHex(
        "06594405063c01012544332211897766551007060504030201"
        "cdcccc3dcdcc4c3e9a9999bf9a99993ecdccccbe0101023f01"
        "0f403f3333b33e3333b33e3333b33e0101023f0101073f33"
        "33333fc8020309403020101f478fc700");
    Packet target_packet{};
    assert(DecodePacket(target_wire.data(), target_wire.size(), target_packet) == DecodeStatus::Ok);
    assert(target_packet.header.message_type == static_cast<uint8_t>(MessageType::SetJointTarget));
    assert(target_packet.header.sequence == 0x55667789U);
    JointTargetPayload target{};
    std::memcpy(&target, target_packet.payload.data(), sizeof(target));
    assert(std::fabs(target.target[0] - 0.1F) < 1e-6F);
    assert(std::fabs(target.target[1] - 0.2F) < 1e-6F);
    assert(std::fabs(target.target[2] + 1.2F) < 1e-6F);
    assert(std::fabs(target.target[6] - 0.75F) < 1e-6F);
    assert(target.valid_for_ms == 200);
    assert(target.target_flags == 3);
    assert(target.control_tick_id == 0x10203040U);
    const size_t target_length = EncodePacket(target_packet, output.data(), output.size());
    assert(target_length == target_wire.size());
    assert(std::equal(target_wire.begin(), target_wire.end(), output.begin()));

    const auto keepalive_wire = FromHex(
        "065944050a0801011d443322118a7766551807060504030201"
        "8977665541302010e9e44b0a00");
    Packet keepalive_packet{};
    assert(DecodePacket(keepalive_wire.data(), keepalive_wire.size(),
                        keepalive_packet) == DecodeStatus::Ok);
    assert(keepalive_packet.header.message_type ==
           static_cast<uint8_t>(MessageType::TargetKeepalive));
    TargetKeepalivePayload keepalive{};
    std::memcpy(&keepalive, keepalive_packet.payload.data(), sizeof(keepalive));
    assert(keepalive.action_sequence == 0x55667789U);
    assert(keepalive.control_tick_id == 0x10203041U);
    const size_t keepalive_length = EncodePacket(
        keepalive_packet, output.data(), output.size());
    assert(keepalive_length == keepalive_wire.size());
    assert(std::equal(keepalive_wire.begin(), keepalive_wire.end(), output.begin()));
}

void TestMonotonicMicrosIgnoresSmallRegressionAndExtendsWrap()
{
    MonotonicMicros32 clock;
    assert(clock.Extend(1000U) == 1000U);
    assert(clock.Extend(1100U) == 1100U);
    assert(clock.Extend(1090U) == 1100U);
    assert(clock.Extend(1101U) == 1101U);

    MonotonicMicros32 wrapping_clock;
    assert(wrapping_clock.Extend(0xFFFFFFF0U) == 0xFFFFFFF0ULL);
    assert(wrapping_clock.Extend(0x00000020U) ==
           (uint64_t{1} << 32U) + 0x20U);

    // CAN RX can advance a sample timestamp after STATE captured now_us. The
    // reconstruction must clamp that race instead of underflowing near 2^64.
    assert(ExtendRecentMicros32(3747033199ULL, 3747033204U) ==
           3747033199ULL);
    assert(ExtendRecentMicros32(5000ULL, 4500U) == 4500ULL);
    assert(ExtendRecentMicros32(
               (uint64_t{1} << 32U) + 500U,
               std::numeric_limits<uint32_t>::max() - 499U) ==
           (uint64_t{1} << 32U) - 500U);
}

void TestMeasuredVelocityUsesOnlyValidMonotonicIntervals()
{
    MeasuredStateEstimator fast_estimator;
    std::array<float, 7> fast_position{};
    std::array<uint32_t, 7> fast_time{};
    fast_time.fill(1000U);
    assert(!fast_estimator.Update(fast_position, fast_time, 1U, true).valid);
    fast_position[0] = 1.0F;
    fast_time.fill(1500U);
    assert(!fast_estimator.Update(fast_position, fast_time, 2U, true).valid);

    MeasuredStateEstimator estimator(100000U);
    std::array<float, 7> position{};
    std::array<uint32_t, 7> sample_time{};
    sample_time.fill(1000U);
    assert(!estimator.Update(position, sample_time, 1U, true).valid);

    position[0] = 0.01F;
    position[6] = 0.1F;
    sample_time.fill(51000U);
    sample_time[6] = 101000U;
    const VelocityEstimate moving = estimator.Update(
        position, sample_time, 2U, true);
    assert(moving.valid);
    assert(std::fabs(moving.velocity[0] - 0.2F) < 1e-6F);
    assert(std::fabs(moving.velocity[6] - 1.0F) < 1e-6F);

    // Re-sending the same coherent sweep reuses the prior derivative and is
    // explicitly labelled repeated instead of producing a zero/spike pair.
    position[0] = 99.0F;
    const VelocityEstimate repeated = estimator.Update(
        position, sample_time, 2U, true);
    assert(repeated.valid);
    assert(repeated.repeated);
    assert(std::fabs(repeated.velocity[0] - 0.2F) < 1e-6F);

    // A long per-node feedback gap invalidates exactly one interval and
    // rebases the next estimate instead of emitting a spike.
    position[0] = 0.02F;
    sample_time.fill(200000U);
    assert(!estimator.Update(position, sample_time, 3U, true).valid);
    sample_time.fill(250000U);
    assert(!estimator.Update(position, sample_time, 4U, false).valid);
    sample_time.fill(300000U);
    assert(!estimator.Update(position, sample_time, 5U, true).valid);
    position[0] = 0.03F;
    sample_time.fill(350000U);
    assert(estimator.Update(position, sample_time, 6U, true).valid);
}

void TestCanFeedbackMonitorTracksAgeAndLossWithoutInventingFaultSources()
{
    CanFeedbackMonitor monitor;
    monitor.OnPositionRequest(0, 1000U);
    for (uint8_t node = 1; node <= 7; ++node)
    {
        if (node != 3)
            monitor.OnPositionResponse(node, 2000U);
    }
    monitor.OnPositionRequest(0, 6000U);
    auto status = monitor.Snapshot(7000U);
    assert(status[0].position_seen);
    assert(status[0].position_age_ms == 5U);
    assert(!status[2].position_seen);
    assert(status[2].position_age_ms == kFeedbackAgeUnknown);
    assert(status[2].total_position_losses == 1U);
    assert(status[2].consecutive_position_losses == 1U);

    monitor.OnPositionResponse(3, 7500U);
    monitor.OnTemperatureRequest(0, 8000U);
    monitor.OnTemperatureResponse(3, 9000U, 42.5F);
    status = monitor.Snapshot(10000U);
    assert(status[2].position_age_ms == 2U);
    assert(status[2].consecutive_position_losses == 0U);
    assert(status[2].temperature_seen);
    assert(std::fabs(status[2].temperature_c - 42.5F) < 1e-6F);
}

void TestCanFeedbackMonitorClampsConcurrentFutureTimestampAndPreservesWrap()
{
    CanFeedbackMonitor monitor;

    // Reproduces the runtime race where CAN RX updates the response timestamp
    // a few microseconds after the control task captured its snapshot time.
    monitor.OnPositionRequest(1U, 900U);
    monitor.OnTemperatureRequest(1U, 900U);
    monitor.OnPositionResponse(1, 1001U);
    monitor.OnTemperatureResponse(1, 1002U, 35.0F);
    auto status = monitor.Snapshot(1000U);
    assert(status[0].position_age_ms == 0U);
    assert(status[0].temperature_age_ms == 0U);

    // A real 32-bit micros() wrap remains a small positive elapsed interval.
    monitor.Reset();
    monitor.OnPositionRequest(
        1U, std::numeric_limits<uint32_t>::max() - 999U);
    monitor.OnPositionResponse(
        1, std::numeric_limits<uint32_t>::max() - 499U);
    status = monitor.Snapshot(500U);
    assert(status[0].position_age_ms == 1U);
}

void TestCanFeedbackMonitorPublishesOnlyCompleteLowSkewSweep()
{
    // This vector intentionally spans 42 ms, so make the acceptance threshold
    // explicit and verify equality at the configured boundary.
    CanFeedbackMonitor monitor(42000U);
    uint32_t now_us = 1000U;
    for (uint8_t node = 1U; node <= 7U; ++node)
    {
        monitor.OnPositionRequest(node, now_us);
        monitor.OnPositionResponse(node, now_us + 100U);
        now_us += 4000U;
    }
    auto coherent = monitor.CoherentSnapshot();
    assert(coherent.valid);
    assert(coherent.sweep_id == 1U);
    assert(coherent.max_skew_us == 24000U);
    for (uint8_t node = 1U; node <= 7U; ++node)
        assert(coherent.position_sweep_id[node - 1U] == coherent.sweep_id);

    // A partial next sweep cannot replace the last complete snapshot.
    monitor.OnPositionRequest(1U, now_us);
    monitor.OnPositionResponse(1U, now_us + 100U);
    const auto partial = monitor.CoherentSnapshot();
    assert(partial.valid);
    assert(partial.sweep_id == coherent.sweep_id);

    CanFeedbackMonitor excessive_skew;
    now_us = 1000U;
    for (uint8_t node = 1U; node <= 7U; ++node)
    {
        excessive_skew.OnPositionRequest(node, now_us);
        excessive_skew.OnPositionResponse(node, now_us + 100U);
        now_us += 6000U;
    }
    assert(!excessive_skew.CoherentSnapshot().valid);
}

void TestStateValidityBitsComeFromOneFeedbackSnapshot()
{
    assert(PositionFeedbackValidityBits(false, false) == 0U);
    assert(PositionFeedbackValidityBits(true, false) == kStatePositionValid);
    assert(PositionFeedbackValidityBits(false, true) == kStateGripperValid);
    assert(PositionFeedbackValidityBits(true, true) ==
        (kStatePositionValid | kStateGripperValid));
}

void TestCanTransportStatusSurvivesStatePayloadCopy()
{
    StatePayload source{};
    source.can_transport_status = 0xA5U;
    std::array<uint8_t, sizeof(StatePayload)> bytes{};
    std::memcpy(bytes.data(), &source, sizeof(source));
    StatePayload restored{};
    std::memcpy(&restored, bytes.data(), sizeof(restored));
    assert(restored.can_transport_status == 0xA5U);
}

FeedbackSafetyConfig MakeSafetyConfig()
{
    FeedbackSafetyConfig config{};
    config.following_error_limit.fill(0.1F);
    config.following_error_hold_ms = 50U;
    config.feedback_hold_ms = 100U;
    config.feedback_fault_ms = 300U;
    config.temperature_max_age_ms = 1000U;
    config.temperature_fault_c = 70.0F;
    config.temperature_fault_ms = 100U;
    return config;
}

FeedbackSafetyInput MakeFreshSafetyInput(uint64_t now_us)
{
    FeedbackSafetyInput input{};
    input.now_us = now_us;
    input.control_active = true;
    input.following_active = true;
    for (auto& node : input.feedback)
    {
        node.position_seen = true;
        node.position_age_ms = 0U;
        node.temperature_seen = true;
        node.temperature_age_ms = 0U;
        node.temperature_c = 35.0F;
    }
    return input;
}

void TestFeedbackSafetyPersistenceSeparatesHoldFromLatchedFault()
{
    FeedbackSafetySupervisor following_supervisor(MakeSafetyConfig());
    auto input = MakeFreshSafetyInput(1000U);
    input.commanded_position[1] = 0.2F;
    auto output = following_supervisor.Update(input);
    assert(output.hold_reason_bits == 0U);
    input.now_us += 50000U;
    output = following_supervisor.Update(input);
    assert((output.hold_reason_bits & kHoldReasonFollowingError) != 0U);
    assert(output.fault_bits == 0U);
    assert(output.following_error_duration_ms[1] == 50U);

    FeedbackSafetySupervisor feedback_supervisor(MakeSafetyConfig());
    input = MakeFreshSafetyInput(1000U);
    input.feedback[4].position_age_ms = 120U;
    output = feedback_supervisor.Update(input);
    assert((output.hold_reason_bits & kHoldReasonFeedbackStale) != 0U);
    assert(output.fault_bits == 0U);
    input.now_us = 401000U;
    input.feedback[4].position_age_ms = 400U;
    output = feedback_supervisor.Update(input);
    assert((output.fault_bits & kFaultFeedbackLost) != 0U);

    FeedbackSafetySupervisor temperature_supervisor(MakeSafetyConfig());
    input = MakeFreshSafetyInput(1000U);
    input.feedback[2].temperature_c = 80.0F;
    assert(temperature_supervisor.Update(input).fault_bits == 0U);
    input.now_us = 101000U;
    input.control_active = false;
    output = temperature_supervisor.Update(input);
    assert((output.fault_bits & kFaultOverTemperature) != 0U);
    // No encoder/stall/current source is advertised by the current motor CAN
    // response, so those validity and fault bits stay clear.
    assert((output.node_validity[2] & kNodeEncoderFaultSourceValid) == 0U);
    assert((output.node_fault_bits[2] &
            (kNodeFaultEncoder | kNodeFaultStall | kNodeFaultOverCurrent)) == 0U);
}

void TestUnverifiedConfigurationCannotAcquire()
{
    ControlSession session(MakeConfig(false), "test-fw");
    const Packet hello = MakeConfiguredHello();
    assert(session.Process(hello, 1000).response.header.message_type ==
           static_cast<uint8_t>(MessageType::HelloAck));
    Packet acquire = MakePacket(MessageType::AcquireControl, hello.header.session_id,
                                hello.header.sequence + 1);
    SetPayload(acquire, AcquireControlPayload{500});
    const ProcessResult result = session.Process(acquire, 2000);
    assert(result.response.header.message_type == static_cast<uint8_t>(MessageType::Nack));
    assert(ResponseCode(result) == ResultCode::BadConfig);
}

void TestCanDiagnosticsAreReadOnlyAfterConfiguredHello()
{
    ControlSession session(MakeConfig(false), "test-fw");
    Packet diagnostics = MakePacket(MessageType::GetCanDiagnostics,
                                    0x11223344U, 2U);
    ProcessResult result = session.Process(diagnostics, 500U);
    assert(ResponseCode(result) == ResultCode::BadSession);
    assert(!result.can_diagnostics_requested);

    const Packet hello = MakeConfiguredHello();
    assert(session.Process(hello, 1000U).response.header.message_type ==
           static_cast<uint8_t>(MessageType::HelloAck));
    diagnostics.header.session_id = hello.header.session_id;
    diagnostics.header.sequence = hello.header.sequence + 1U;
    result = session.Process(diagnostics, 2000U);
    assert(ResponseCode(result) == ResultCode::Ok);
    assert(result.can_diagnostics_requested);
    assert(!session.lease_active());
    assert(session.mode() == ControlMode::Hold);

    Packet malformed = diagnostics;
    malformed.header.payload_length = 1U;
    result = session.Process(malformed, 3000U);
    assert(ResponseCode(result) == ResultCode::BadLength);
    assert(!result.can_diagnostics_requested);

    Packet wrong_session = diagnostics;
    wrong_session.header.session_id ^= 0x01010101U;
    result = session.Process(wrong_session, 4000U);
    assert(ResponseCode(result) == ResultCode::BadSession);
    assert(!result.can_diagnostics_requested);

    Packet heartbeat = MakePacket(MessageType::Heartbeat,
                                  hello.header.session_id,
                                  hello.header.sequence + 2U);
    assert(ResponseCode(session.Process(heartbeat, 5000U)) ==
           ResultCode::NoLease);
}

void TestSessionTargetAndTimeout()
{
    ControlSession session(MakeConfig(true), "test-fw");
    session.SetControlReady(true);
    const Packet hello = MakeConfiguredHello();
    const ProcessResult hello_result = session.Process(hello, 1000);
    HelloAckPayload hello_ack{};
    std::memcpy(&hello_ack, hello_result.response.payload.data(), sizeof(hello_ack));
    assert((hello_ack.capabilities & kCapabilityMultiChannelSequence) != 0U);
    assert((hello_ack.capabilities & kCapabilityTargetKeepalive) != 0U);
    assert((hello_ack.capabilities & kCapabilityCanTxCompleteExact) != 0U);
    assert((hello_ack.capabilities & kCapabilityControlFreshnessToken) != 0U);
    assert((hello_ack.capabilities & kCapabilityTimeSync) != 0U);
    assert((hello_ack.capabilities & kCapabilityCanDiagnostics) != 0U);
    assert((hello_ack.capabilities & kCapabilityCanDiagnosticsV2) != 0U);

    Packet acquire = MakePacket(MessageType::AcquireControl, hello.header.session_id,
                                hello.header.sequence + 1);
    SetPayload(acquire, AcquireControlPayload{500});
    assert(ResponseCode(session.Process(acquire, 2000)) == ResultCode::Ok);
    assert(session.lease_active());
    assert(session.mode() == ControlMode::Hold);

    Packet mode = MakePacket(MessageType::SetMode, hello.header.session_id,
                             hello.header.sequence + 2);
    SetPayload(mode, SetModePayload{static_cast<uint8_t>(ControlMode::Teleop)});
    assert(ResponseCode(session.Process(mode, 3000)) == ResultCode::Ok);
    assert(session.mode() == ControlMode::Teleop);

    JointTargetPayload target{};
    const float positions[7] = {0.1F, 0.2F, -1.2F, 0.3F, -0.4F, 0.5F, 0.75F};
    const float velocities[6] = {0.35F, 0.35F, 0.35F, 0.5F, 0.5F, 0.7F};
    std::copy(std::begin(positions), std::end(positions), target.target);
    std::copy(std::begin(velocities), std::end(velocities), target.max_velocity);
    target.valid_for_ms = 100;
    target.target_flags = 3;
    target.control_tick_id = 1U;
    Packet command = MakePacket(MessageType::SetJointTarget, hello.header.session_id,
                                hello.header.sequence + 3);
    SetPayload(command, target);
    ProcessResult result = session.Process(command, 4000);
    assert(ResponseCode(result) == ResultCode::Ok);
    assert(result.target_updated);
    assert(session.active_target().valid);
    assert(std::fabs(session.active_target().target[2] + 1.2F) < 1e-6F);
    assert(!session.Tick(103999));
    assert(session.Tick(104000));
    assert(session.mode() == ControlMode::Hold);
    assert(!session.active_target().valid);
}

void TestTargetKeepaliveIsExactAndControlBound()
{
    ControlSession session(MakeConfig(true), "test-fw");
    session.SetControlReady(true);
    const Packet hello = MakeConfiguredHello();
    session.Process(hello, 1000U);

    Packet acquire = MakePacket(MessageType::AcquireControl,
                                hello.header.session_id,
                                hello.header.sequence + 1U);
    SetPayload(acquire, AcquireControlPayload{500U});
    assert(ResponseCode(session.Process(acquire, 2000U)) == ResultCode::Ok);
    Packet mode = MakePacket(MessageType::SetMode, hello.header.session_id,
                             hello.header.sequence + 2U);
    SetPayload(mode, SetModePayload{static_cast<uint8_t>(ControlMode::Teleop)});
    assert(ResponseCode(session.Process(mode, 3000U)) == ResultCode::Ok);

    JointTargetPayload target{};
    std::copy(dummy::generated_config::kInitialPoseRad.begin(),
              dummy::generated_config::kInitialPoseRad.end(), target.target);
    target.target[6] = 0.5F;
    target.max_velocity[0] = 0.1F;
    target.max_velocity[1] = 0.1F;
    target.max_velocity[2] = 0.1F;
    target.max_velocity[3] = 0.1F;
    target.max_velocity[4] = 0.1F;
    target.max_velocity[5] = 0.1F;
    target.valid_for_ms = 100U;
    target.control_tick_id = 1U;
    Packet command = MakePacket(MessageType::SetJointTarget,
                                hello.header.session_id,
                                hello.header.sequence + 3U);
    SetPayload(command, target);
    assert(ResponseCode(session.Process(command, 4000U)) == ResultCode::Ok);

    // A generic lease heartbeat does not extend the motion deadline.
    Packet heartbeat = MakePacket(MessageType::Heartbeat,
                                  hello.header.session_id,
                                  hello.header.sequence + 4U);
    assert(ResponseCode(session.Process(heartbeat, 50000U)) == ResultCode::Ok);
    assert(session.active_target().deadline_us == 104000U);

    // A fresh control tick must name the exact active action sequence.
    Packet wrong = MakePacket(MessageType::TargetKeepalive,
                              hello.header.session_id,
                              hello.header.sequence + 5U);
    SetPayload(wrong, TargetKeepalivePayload{
        command.header.sequence + 1U, 2U});
    assert(ResponseCode(session.Process(wrong, 60000U)) == ResultCode::BadSequence);
    Packet refresh = MakePacket(MessageType::TargetKeepalive,
                                hello.header.session_id,
                                hello.header.sequence + 6U);
    SetPayload(refresh, TargetKeepalivePayload{
        command.header.sequence, 2U});
    assert(ResponseCode(session.Process(refresh, 90000U)) == ResultCode::Ok);
    assert(session.active_target().last_refresh_time_us == 90000U);
    assert(session.active_target().deadline_us == 190000U);
    assert(!session.Tick(189999U));
    assert(session.Tick(190000U));
    assert(session.mode() == ControlMode::Hold);
}

void TestTelemetryMovesToLatestHelloAfterRelease()
{
    ControlSession session(MakeConfig(true), "test-fw");
    session.SetControlReady(true);
    const Packet first_hello = MakeConfiguredHello();
    session.Process(first_hello, 1000);
    assert(session.telemetry_session_id() == first_hello.header.session_id);

    Packet acquire = MakePacket(MessageType::AcquireControl, first_hello.header.session_id,
                                first_hello.header.sequence + 1);
    SetPayload(acquire, AcquireControlPayload{500});
    assert(ResponseCode(session.Process(acquire, 2000)) == ResultCode::Ok);
    assert(session.telemetry_session_id() == first_hello.header.session_id);

    Packet release = MakePacket(MessageType::ReleaseControl, first_hello.header.session_id,
                                first_hello.header.sequence + 2);
    assert(ResponseCode(session.Process(release, 3000)) == ResultCode::Ok);
    assert(!session.lease_active());

    Packet next_hello = MakeConfiguredHello();
    next_hello.header.session_id = 0xA5A5A5A5U;
    next_hello.header.sequence = 1;
    assert(session.Process(next_hello, 4000).response.header.message_type ==
           static_cast<uint8_t>(MessageType::HelloAck));
    assert(session.telemetry_session_id() == next_hello.header.session_id);
}

void TestBadSequenceAndTargetAreRejected()
{
    ControlSession session(MakeConfig(true), "test-fw");
    session.SetControlReady(true);
    const Packet hello = MakeConfiguredHello();
    session.Process(hello, 1);
    Packet acquire = MakePacket(MessageType::AcquireControl, hello.header.session_id,
                                hello.header.sequence + 1);
    SetPayload(acquire, AcquireControlPayload{500});
    session.Process(acquire, 2);
    Packet mode = MakePacket(MessageType::SetMode, hello.header.session_id,
                             hello.header.sequence + 2);
    SetPayload(mode, SetModePayload{static_cast<uint8_t>(ControlMode::Teleop)});
    session.Process(mode, 3);

    JointTargetPayload target{};
    for (size_t index = 0; index < 6; ++index)
        target.max_velocity[index] = 0.1F;
    target.target[2] = -1.0F;
    target.target[6] = 0.5F;
    target.valid_for_ms = 100;
    target.control_tick_id = 1U;
    Packet bad = MakePacket(MessageType::SetJointTarget, hello.header.session_id,
                            hello.header.sequence + 3);
    target.target[0] = 100.0F;
    SetPayload(bad, target);
    assert(ResponseCode(session.Process(bad, 4)) == ResultCode::OutOfRange);

    Packet repeated = MakePacket(MessageType::Heartbeat, hello.header.session_id,
                                 hello.header.sequence + 3);
    assert(ResponseCode(session.Process(repeated, 5)) == ResultCode::BadSequence);
}

void TestSequenceWrapUsesUint32SerialArithmetic()
{
    ControlSession session(MakeConfig(true), "test-fw");
    session.SetControlReady(true);
    Packet hello = MakeConfiguredHello();
    hello.header.sequence = 0xFFFFFFFDU;
    session.Process(hello, 1U);

    Packet acquire = MakePacket(
        MessageType::AcquireControl, hello.header.session_id, 0xFFFFFFFEU);
    SetPayload(acquire, AcquireControlPayload{500U});
    assert(ResponseCode(session.Process(acquire, 2U)) == ResultCode::Ok);

    Packet mode = MakePacket(
        MessageType::SetMode, hello.header.session_id, 0xFFFFFFFFU);
    SetPayload(mode, SetModePayload{static_cast<uint8_t>(ControlMode::Teleop)});
    assert(ResponseCode(session.Process(mode, 3U)) == ResultCode::Ok);

    Packet heartbeat = MakePacket(
        MessageType::Heartbeat, hello.header.session_id, 1U);
    assert(ResponseCode(session.Process(heartbeat, 4U)) == ResultCode::Ok);
    assert(ResponseCode(session.Process(heartbeat, 5U)) == ResultCode::BadSequence);
}

void TestReliableControlMayOvertakeAnOlderMotionTarget()
{
    ControlSession session(MakeConfig(true), "test-fw");
    session.SetControlReady(true);
    const Packet hello = MakeConfiguredHello();
    session.Process(hello, 1U);

    Packet acquire = MakePacket(MessageType::AcquireControl,
                                hello.header.session_id,
                                hello.header.sequence + 1U);
    SetPayload(acquire, AcquireControlPayload{500U});
    assert(ResponseCode(session.Process(acquire, 2U)) == ResultCode::Ok);

    Packet mode = MakePacket(MessageType::SetMode, hello.header.session_id,
                             hello.header.sequence + 2U);
    SetPayload(mode, SetModePayload{static_cast<uint8_t>(ControlMode::Teleop)});
    assert(ResponseCode(session.Process(mode, 3U)) == ResultCode::Ok);

    JointTargetPayload target{};
    std::copy(dummy::generated_config::kInitialPoseRad.begin(),
              dummy::generated_config::kInitialPoseRad.end(), target.target);
    target.target[2] = -1.0F;
    target.target[6] = 0.5F;
    for (size_t index = 0; index < 6; ++index)
        target.max_velocity[index] = 0.1F;
    target.valid_for_ms = 100U;
    target.control_tick_id = 1U;
    Packet delayed_target = MakePacket(MessageType::SetJointTarget,
                                       hello.header.session_id,
                                       hello.header.sequence + 3U);
    SetPayload(delayed_target, target);

    // The serial writer intentionally prioritizes reliable control over the
    // latest-value motion mailbox.  That must not turn the delayed target into
    // a false replay rejection.
    Packet overtaking_heartbeat = MakePacket(MessageType::Heartbeat,
                                             hello.header.session_id,
                                             hello.header.sequence + 4U);
    assert(ResponseCode(session.Process(overtaking_heartbeat, 4U)) == ResultCode::Ok);
    assert(ResponseCode(session.Process(delayed_target, 5U)) == ResultCode::Ok);
    assert(session.last_received_sequence() == delayed_target.header.sequence);

    // The target channel still rejects an exact replay independently.
    assert(ResponseCode(session.Process(delayed_target, 6U)) == ResultCode::BadSequence);
}

Packet MakeConfiguredHello()
{
    Packet packet = DecodeHelloVector();
    HelloPayload payload{};
    std::memcpy(&payload, packet.payload.data(), sizeof(payload));
    std::copy(dummy::generated_config::kConfigSha256.begin(),
              dummy::generated_config::kConfigSha256.end(), payload.config_sha256);
    SetPayload(packet, payload);
    return packet;
}

void TestUrdfJointSpaceMapping()
{
    const std::array<float, 6> expected_reduction = {50.0F, 50.0F, 50.0F,
                                                      50.0F, 50.0F, 50.0F};
    assert(dummy::generated_config::kJointReduction == expected_reduction);

    const std::array<float, 6> firmware_rest_degrees = {0.0F, -73.0F, 180.0F,
                                                        0.0F, 0.0F, 0.0F};
    for (size_t index = 0; index < firmware_rest_degrees.size(); ++index)
    {
        const float urdf = LegacyFirmwareDegreesToUrdfRadians(
            firmware_rest_degrees[index], index);
        assert(std::fabs(urdf) < 1e-5F);
        const float legacy = UrdfRadiansToLegacyFirmwareRadians(urdf, index);
        assert(std::fabs(legacy - dummy::generated_config::kJointZeroOffsetRad[index]) <
               1e-5F);
    }

    constexpr float kProbe = 0.25F;
    assert(std::fabs(UrdfRadiansToLegacyFirmwareRadians(kProbe, 1) -
                     (dummy::generated_config::kJointZeroOffsetRad[1] + kProbe)) < 1e-6F);
    assert(std::fabs(UrdfRadiansToLegacyFirmwareRadians(kProbe, 3) -
                     (dummy::generated_config::kJointZeroOffsetRad[3] - kProbe)) < 1e-6F);
    assert(std::fabs(UrdfRadiansToLegacyFirmwareRadians(kProbe, 5) -
                     (dummy::generated_config::kJointZeroOffsetRad[5] - kProbe)) < 1e-6F);
}

void TestControlAcquisitionRequiresFeedbackBootstrap()
{
    ControlSession session(MakeConfig(true), "test-fw");
    const Packet hello = MakeConfiguredHello();
    session.Process(hello, 1000U);
    Packet acquire = MakePacket(MessageType::AcquireControl,
                                hello.header.session_id,
                                hello.header.sequence + 1U);
    SetPayload(acquire, AcquireControlPayload{500U});
    const ProcessResult rejected = session.Process(acquire, 2000U);
    assert(ResponseCode(rejected) == ResultCode::BadMode);
    assert(ResponseDetail(rejected) == kAckDetailFeedbackNotReady);
    assert(!session.lease_active());

    session.SetControlReady(true);
    assert(ResponseCode(session.Process(acquire, 3000U)) == ResultCode::Ok);
    assert(session.lease_active());
}

void TestCanDispatcherTransitionsAndFrequencyPlan()
{
    CanDispatchScheduler scheduler;
    scheduler.SetMode(CanDispatchMode::Stream);
    uint32_t now_us = 0U;
    std::vector<uint8_t> transition_nodes;
    std::vector<uint8_t> diagnostic_nodes;
    FeedbackResponseEvents transition_responses{};
    bool configured = false;
    bool enabled = false;
    for (size_t tick = 0; tick < 50U && !enabled; ++tick)
    {
        const CanDispatchStep step = scheduler.Next(
            now_us, transition_responses);
        transition_responses = {};
        if (step.action != CanDispatchAction::None)
        {
            assert(step.transition);
            scheduler.OnQueued(step, now_us);
            if (step.action == CanDispatchAction::ActuatorTarget)
                transition_nodes.push_back(step.node_id);
            else if (step.action ==
                     CanDispatchAction::MotorDiagnosticsRequest)
            {
                diagnostic_nodes.push_back(step.node_id);
                transition_responses.temperature_mask = static_cast<uint8_t>(
                    1U << (step.node_id - 1U));
            }
            else if (step.action ==
                     CanDispatchAction::ConfigureGripperVelocity)
            {
                configured = true;
            }
            else if (step.action == CanDispatchAction::EnableBroadcast)
                enabled = true;
        }
        now_us += 1429U;
    }
    assert((transition_nodes == std::vector<uint8_t>{1U, 2U, 3U, 4U, 5U, 6U, 7U}));
    assert((diagnostic_nodes == std::vector<uint8_t>{1U, 2U, 3U, 4U, 5U, 6U, 7U}));
    assert(configured);
    assert(enabled);

    const CanDispatchDiagnostics before = scheduler.diagnostics();
    FeedbackResponseEvents responses{};
    std::array<uint32_t, kActuatorNodeCount> last_node_tx{};
    std::array<bool, kActuatorNodeCount> node_seen{};
    const uint32_t end_us = now_us + 10000000U;
    while (now_us < end_us)
    {
        const CanDispatchStep step = scheduler.Next(now_us, responses);
        responses = {};
        if (step.action != CanDispatchAction::None)
        {
            if (step.node_id >= 1U && step.node_id <= kActuatorNodeCount)
            {
                const size_t index = step.node_id - 1U;
                if (node_seen[index])
                    assert(now_us - last_node_tx[index] >= 5000U);
                node_seen[index] = true;
                last_node_tx[index] = now_us;
            }
            scheduler.OnQueued(step, now_us);
            if (step.action == CanDispatchAction::PositionRequest)
                responses.position_mask = static_cast<uint8_t>(
                    1U << (step.node_id - 1U));
            else if (step.action == CanDispatchAction::TemperatureRequest)
                responses.temperature_mask = static_cast<uint8_t>(
                    1U << (step.node_id - 1U));
            // Model a TX-complete/RX-response wake much sooner than the 1 ms
            // watchdog. No parallel mailbox is used.
            now_us += 100U;
        }
        else
        {
            now_us += 1000U;
        }
    }

    const CanDispatchDiagnostics after = scheduler.diagnostics();
    uint32_t total_temperatures = 0U;
    for (size_t index = 0; index < kActuatorNodeCount; ++index)
    {
        const uint32_t targets = after.target_queued[index] -
            before.target_queued[index];
        const uint32_t positions = after.position_requested[index] -
            before.position_requested[index];
        const uint32_t temperatures = after.temperature_requested[index] -
            before.temperature_requested[index];
        total_temperatures += temperatures;
        assert(targets >= 490U && targets <= 500U);
        assert(positions >= 390U && positions <= 400U);
        assert(temperatures >= 9U && temperatures <= 10U);
        assert(after.position_responded[index] >= positions - 1U);
    }
    assert(total_temperatures >= 69U && total_temperatures <= 70U);
}

void TestCanDispatcherWaitsForResponseAndTimesOut()
{
    CanDispatchScheduler scheduler;
    uint32_t now_us = 0U;
    CanDispatchStep request{};
    for (size_t tick = 0; tick < 20U; ++tick)
    {
        request = scheduler.Next(now_us);
        if (request.action == CanDispatchAction::PositionRequest)
            break;
        now_us += 1429U;
    }
    assert(request.action == CanDispatchAction::PositionRequest);
    scheduler.OnQueued(request, now_us);

    CanDispatchStep waiting = scheduler.Next(now_us + 1000U);
    assert(waiting.action == CanDispatchAction::None);
    assert(waiting.timed_out_action == CanDispatchAction::None);

    CanDispatchStep timeout = scheduler.Next(now_us + 4000U);
    assert(timeout.timed_out_action == CanDispatchAction::PositionRequest);
    assert(timeout.timed_out_node_id == request.node_id);
    assert(!timeout.timed_out_final);
    assert(timeout.action == CanDispatchAction::PositionRequest);
    assert(timeout.node_id != request.node_id);
    assert(scheduler.diagnostics().position_timed_out[request.node_id - 1U] == 1U);
}

void TestCanDispatcherContinuesSweepAndRetriesOnlyMissingNode()
{
    CanDispatchConfig config{};
    config.node_quiet_us = 0U;
    config.temperature_hz_per_node = 0U;
    CanDispatchScheduler scheduler(config);
    assert(scheduler.Next(0U).action == CanDispatchAction::None);
    uint32_t now_us = 25000U;

    CanDispatchStep step = scheduler.Next(now_us);
    assert(step.action == CanDispatchAction::PositionRequest);
    const uint8_t missing_node = step.node_id;
    const uint32_t sweep_id = step.feedback_sweep_id;
    scheduler.OnQueued(step, now_us);

    // The first timeout continues with the next node while preserving the
    // missing node's sweep identity for one tail retry.
    now_us += config.response_timeout_us;
    step = scheduler.Next(now_us);
    assert(step.timed_out_node_id == missing_node);
    assert(!step.timed_out_final);
    assert(step.action == CanDispatchAction::PositionRequest);
    assert(step.node_id != missing_node);
    assert(step.feedback_sweep_id == sweep_id);

    for (size_t completed = 1U; completed < kActuatorNodeCount; ++completed)
    {
        scheduler.OnQueued(step, now_us);
        FeedbackResponseEvents response{};
        response.position_mask = static_cast<uint8_t>(
            1U << (step.node_id - 1U));
        now_us += 100U;
        step = scheduler.Next(now_us, response);
    }

    assert(step.action == CanDispatchAction::PositionRequest);
    assert(step.node_id == missing_node);
    assert(step.feedback_sweep_id == sweep_id);
    scheduler.OnQueued(step, now_us);
    FeedbackResponseEvents retry_response{};
    retry_response.position_mask = static_cast<uint8_t>(
        1U << (missing_node - 1U));
    scheduler.Next(now_us + 100U, retry_response);

    const auto diagnostics = scheduler.diagnostics();
    assert(diagnostics.position_requested[missing_node - 1U] == 2U);
    assert(diagnostics.position_responded[missing_node - 1U] == 1U);
    assert(diagnostics.position_timed_out[missing_node - 1U] == 1U);
    for (uint8_t node_id = 1U; node_id <= kActuatorNodeCount; ++node_id)
    {
        if (node_id != missing_node)
            assert(diagnostics.position_requested[node_id - 1U] == 1U);
    }
}

void TestCanDispatcherAcceptsLateSweepResponseAndReportsRetryExhaustion()
{
    CanDispatchConfig config{};
    config.node_quiet_us = 0U;
    config.temperature_hz_per_node = 0U;

    CanDispatchScheduler late_scheduler(config);
    late_scheduler.Next(0U);
    uint32_t now_us = 25000U;
    CanDispatchStep step = late_scheduler.Next(now_us);
    const uint8_t late_node = step.node_id;
    late_scheduler.OnQueued(step, now_us);
    now_us += config.response_timeout_us;
    step = late_scheduler.Next(now_us);
    assert(!step.timed_out_final);

    for (size_t completed = 1U; completed < kActuatorNodeCount; ++completed)
    {
        late_scheduler.OnQueued(step, now_us);
        FeedbackResponseEvents response{};
        response.position_mask = static_cast<uint8_t>(
            1U << (step.node_id - 1U));
        if (completed == 1U)
            response.position_mask = static_cast<uint8_t>(
                response.position_mask | (1U << (late_node - 1U)));
        now_us += 100U;
        step = late_scheduler.Next(now_us, response);
    }
    assert(step.action == CanDispatchAction::None);
    assert(late_scheduler.diagnostics().position_requested[late_node - 1U] == 1U);

    CanDispatchScheduler exhausted_scheduler(config);
    exhausted_scheduler.Next(0U);
    now_us = 25000U;
    step = exhausted_scheduler.Next(now_us);
    const uint8_t missing_node = step.node_id;
    exhausted_scheduler.OnQueued(step, now_us);
    now_us += config.response_timeout_us;
    step = exhausted_scheduler.Next(now_us);
    for (size_t completed = 1U; completed < kActuatorNodeCount; ++completed)
    {
        exhausted_scheduler.OnQueued(step, now_us);
        FeedbackResponseEvents response{};
        response.position_mask = static_cast<uint8_t>(
            1U << (step.node_id - 1U));
        now_us += 100U;
        step = exhausted_scheduler.Next(now_us, response);
    }
    assert(step.node_id == missing_node);
    exhausted_scheduler.OnQueued(step, now_us);
    const CanDispatchStep exhausted = exhausted_scheduler.Next(
        now_us + config.response_timeout_us);
    assert(exhausted.timed_out_action == CanDispatchAction::PositionRequest);
    assert(exhausted.timed_out_node_id == missing_node);
    assert(exhausted.timed_out_final);
    assert(exhausted_scheduler.diagnostics().position_timed_out[
        missing_node - 1U] == 2U);
}

uint32_t CompleteStreamTransition(CanDispatchScheduler& scheduler)
{
    scheduler.SetMode(CanDispatchMode::Stream);
    uint32_t now_us = 0U;
    FeedbackResponseEvents responses{};
    uint8_t hold_count = 0U;
    uint8_t diagnostics_count = 0U;
    bool configured = false;
    bool enabled = false;
    for (size_t attempt = 0U; attempt < 64U && !enabled; ++attempt)
    {
        const CanDispatchStep step = scheduler.Next(now_us, responses);
        responses = {};
        if (step.action != CanDispatchAction::None)
        {
            assert(step.transition);
            scheduler.OnQueued(step, now_us);
            if (step.action == CanDispatchAction::ActuatorTarget)
            {
                assert(step.node_id == hold_count + 1U);
                ++hold_count;
            }
            else if (step.action ==
                     CanDispatchAction::MotorDiagnosticsRequest)
            {
                assert(step.node_id == diagnostics_count + 1U);
                ++diagnostics_count;
                responses.temperature_mask = static_cast<uint8_t>(
                    1U << (step.node_id - 1U));
            }
            else if (step.action ==
                     CanDispatchAction::ConfigureGripperVelocity)
            {
                configured = true;
            }
            else if (step.action == CanDispatchAction::EnableBroadcast)
            {
                enabled = true;
            }
        }
        ++now_us;
    }
    assert(hold_count == kActuatorNodeCount);
    assert(diagnostics_count == kActuatorNodeCount);
    assert(configured);
    assert(enabled);
    assert(scheduler.Next(now_us).action == CanDispatchAction::None);
    return now_us;
}

void TestCanDispatcherPreflightTimeoutStopsEnable()
{
    CanDispatchConfig config{};
    config.node_quiet_us = 0U;
    config.position_hz_per_node = 0U;
    config.temperature_hz_per_node = 0U;
    CanDispatchScheduler scheduler(config);
    scheduler.SetMode(CanDispatchMode::Stream);
    uint32_t now_us = 0U;
    for (uint8_t node_id = 1U; node_id <= kActuatorNodeCount; ++node_id)
    {
        const CanDispatchStep hold = scheduler.Next(now_us++);
        assert(hold.action == CanDispatchAction::ActuatorTarget);
        assert(hold.node_id == node_id);
        scheduler.OnQueued(hold, now_us);
    }

    const CanDispatchStep request = scheduler.Next(now_us++);
    assert(request.action == CanDispatchAction::MotorDiagnosticsRequest);
    assert(request.node_id == 1U);
    scheduler.OnQueued(request, now_us);
    now_us += config.response_timeout_us;
    const CanDispatchStep timeout = scheduler.Next(now_us);
    assert(timeout.timed_out_final);
    assert(timeout.timed_out_action ==
           CanDispatchAction::MotorDiagnosticsRequest);
    assert(timeout.timed_out_node_id == 1U);
    assert(timeout.action != CanDispatchAction::ConfigureGripperVelocity);
    assert(timeout.action != CanDispatchAction::EnableBroadcast);
    const CanDispatchDiagnostics diagnostics = scheduler.diagnostics();
    assert(diagnostics.temperature_timed_out[0] == 1U);
    assert(!diagnostics.query_pending);
}

void TestCanDispatcherDoesNotBurstAfterDeferredDeadline()
{
    CanDispatchConfig config{};
    config.node_quiet_us = 0U;
    config.position_hz_per_node = 0U;
    config.temperature_hz_per_node = 0U;
    CanDispatchScheduler scheduler(config);
    uint32_t now_us = CompleteStreamTransition(scheduler);

    CanDispatchStep due{};
    for (size_t tick = 0; tick < 50U; ++tick)
    {
        due = scheduler.Next(now_us);
        now_us += 1429U;
        if (due.action == CanDispatchAction::ActuatorTarget)
            scheduler.OnDeferred();
    }
    assert(due.action == CanDispatchAction::ActuatorTarget);
    scheduler.OnQueued(due, now_us);

    // One overdue cycle becomes exactly one atomic seven-node fan-out. Nodes
    // 2..7 follow TX-complete wakes immediately, but no historical cycle is
    // replayed after that fan-out finishes.
    for (uint8_t node_id = 2U; node_id <= kActuatorNodeCount; ++node_id)
    {
        const CanDispatchStep next = scheduler.Next(now_us + node_id * 100U);
        assert(next.action == CanDispatchAction::ActuatorTarget);
        assert(next.node_id == node_id);
        scheduler.OnQueued(next, now_us + node_id * 100U);
    }
    const CanDispatchStep after = scheduler.Next(now_us + 1000U);
    assert(after.action == CanDispatchAction::None);
}

void TestCanDispatcherRejectsInvalidRatePlanWithoutFallback()
{
    CanDispatchConfig config{};
    config.scheduler_watchdog_hz = 99U;
    CanDispatchScheduler scheduler(config);
    scheduler.SetMode(CanDispatchMode::Stream);
    for (uint32_t tick = 0; tick < 1000U; ++tick)
        assert(scheduler.Next(tick * 10000U).action == CanDispatchAction::None);
    const auto diagnostics = scheduler.diagnostics();
    assert(!diagnostics.config_valid);
    assert(diagnostics.idle_slot_count == 1000U);
}

void TestCanDispatcherBootstrapsEveryNodeAndFaultPreemptsQuery()
{
    CanDispatchScheduler scheduler;
    uint32_t now_us = 0U;
    FeedbackResponseEvents responses{};
    for (size_t tick = 0; tick < 100U; ++tick)
    {
        const CanDispatchStep step = scheduler.Next(now_us, responses);
        responses = {};
        assert(step.action != CanDispatchAction::ActuatorTarget);
        if (step.action != CanDispatchAction::None)
        {
            scheduler.OnQueued(step, now_us);
            if (step.action == CanDispatchAction::PositionRequest)
                responses.position_mask = static_cast<uint8_t>(
                    1U << (step.node_id - 1U));
            else if (step.action == CanDispatchAction::TemperatureRequest)
                responses.temperature_mask = static_cast<uint8_t>(
                    1U << (step.node_id - 1U));
        }
        now_us += 1429U;
    }
    const auto bootstrap = scheduler.diagnostics();
    for (size_t index = 0; index < kActuatorNodeCount; ++index)
    {
        assert(bootstrap.position_requested[index] >= 2U);
        assert(bootstrap.position_responded[index] >= 2U);
    }

    // Leave one feedback query outstanding, then verify a FAULT produces an
    // immediate disable decision instead of waiting for the 4 ms timeout.
    CanDispatchStep pending{};
    for (size_t tick = 0; tick < 20U; ++tick)
    {
        pending = scheduler.Next(now_us);
        if (pending.action == CanDispatchAction::PositionRequest)
            break;
        now_us += 1429U;
    }
    assert(pending.action == CanDispatchAction::PositionRequest);
    scheduler.OnQueued(pending, now_us);
    scheduler.SetMode(CanDispatchMode::Fault);
    const CanDispatchStep fault = scheduler.Next(now_us + 1U);
    assert(fault.action == CanDispatchAction::DisableBroadcast);
    assert(fault.transition);

    CanDispatchScheduler hold_scheduler;
    CanDispatchStep hold_pending{};
    uint32_t hold_now_us = 0U;
    for (size_t tick = 0; tick < 20U; ++tick)
    {
        hold_pending = hold_scheduler.Next(hold_now_us);
        if (hold_pending.action == CanDispatchAction::PositionRequest)
            break;
        hold_now_us += 1429U;
    }
    assert(hold_pending.action == CanDispatchAction::PositionRequest);
    hold_scheduler.OnQueued(hold_pending, hold_now_us);
    hold_scheduler.SetMode(CanDispatchMode::Hold);
    CanDispatchStep hold{};
    for (uint32_t wait_us = 1U; wait_us <= 5001U; wait_us += 1000U)
    {
        hold = hold_scheduler.Next(hold_now_us + wait_us);
        assert(hold.action == CanDispatchAction::None ||
               hold.action == CanDispatchAction::ActuatorTarget);
        if (hold.action == CanDispatchAction::ActuatorTarget)
            break;
    }
    assert(hold.action == CanDispatchAction::ActuatorTarget);
    assert(hold.transition);
}

void TestSafetyModesPreemptPartialTargetFanout()
{
    CanDispatchConfig config{};
    config.node_quiet_us = 0U;
    config.position_hz_per_node = 0U;
    config.temperature_hz_per_node = 0U;

    CanDispatchScheduler hold_scheduler(config);
    uint32_t now_us = CompleteStreamTransition(hold_scheduler) + 20000U;
    const CanDispatchStep normal_target = hold_scheduler.Next(now_us);
    assert(normal_target.action == CanDispatchAction::ActuatorTarget);
    assert(normal_target.node_id == 1U);
    assert(!normal_target.transition);
    hold_scheduler.OnQueued(normal_target, now_us);

    hold_scheduler.SetMode(CanDispatchMode::Hold);
    const CanDispatchStep hold = hold_scheduler.Next(now_us + 1U);
    assert(hold.action == CanDispatchAction::ActuatorTarget);
    assert(hold.node_id == 1U);
    assert(hold.transition);

    CanDispatchScheduler fault_scheduler(config);
    now_us = CompleteStreamTransition(fault_scheduler) + 20000U;
    const CanDispatchStep fault_target = fault_scheduler.Next(now_us);
    assert(fault_target.action == CanDispatchAction::ActuatorTarget);
    assert(fault_target.node_id == 1U);
    assert(!fault_target.transition);
    fault_scheduler.OnQueued(fault_target, now_us);

    fault_scheduler.SetMode(CanDispatchMode::Fault);
    const CanDispatchStep fault = fault_scheduler.Next(now_us + 1U);
    assert(fault.action == CanDispatchAction::DisableBroadcast);
    assert(fault.transition);
}

void TestTargetCompletionRetriesOnlyTheExactFailedNode()
{
    TargetCompletionTracker tracker(15000U);
    const TargetFanoutKey key{7U, 19U, 3U};
    assert(tracker.Begin(key, 100U));
    assert(tracker.RecordCompletion(key, 1U, true, 200U) ==
           TargetCompletionResult::Awaiting);
    assert(tracker.RecordCompletion(key, 2U, false, 300U) ==
           TargetCompletionResult::RetryRequired);

    const TargetRetryRequest retry = tracker.retry_request();
    assert(retry.valid);
    assert(retry.key.session_epoch == key.session_epoch);
    assert(retry.key.action_sequence == key.action_sequence);
    assert(retry.key.fanout_generation == key.fanout_generation);
    assert(retry.node_id == 2U);
    TargetRetryRequest wrong = retry;
    ++wrong.key.session_epoch;
    assert(!tracker.MarkRetryQueued(wrong));
    assert(tracker.retry_request().valid);
    assert(tracker.MarkRetryQueued(retry));
    assert(!tracker.retry_request().valid);
    assert(tracker.RecordCompletion(key, 2U, true, 400U) ==
           TargetCompletionResult::Awaiting);
    for (uint8_t node_id = 3U; node_id < kActuatorNodeCount; ++node_id)
    {
        assert(tracker.RecordCompletion(
                   key, node_id, true,
                   static_cast<uint32_t>(400U + node_id * 100U)) ==
               TargetCompletionResult::Awaiting);
    }
    assert(tracker.RecordCompletion(key, kActuatorNodeCount, true, 1200U) ==
           TargetCompletionResult::CompleteExact);
    assert(!tracker.active());
    const TargetCompletionDiagnostics diagnostics = tracker.diagnostics();
    assert(diagnostics.retry_count == 1U);
    assert(diagnostics.retry_exhausted_count == 0U);
    assert(diagnostics.deadline_failure_count == 0U);
    assert(diagnostics.max_fanout_us == 1100U);
    tracker.ResetDiagnostics();
    const TargetCompletionDiagnostics reset = tracker.diagnostics();
    assert(reset.retry_count == 0U);
    assert(reset.retry_exhausted_count == 0U);
    assert(reset.deadline_failure_count == 0U);
    assert(reset.max_fanout_us == 0U);
}

void TestTargetCompletionFailsOnSecondErrorOrFanoutDeadline()
{
    const TargetFanoutKey key{9U, 22U, 5U};
    TargetCompletionTracker retry_exhausted(15000U);
    assert(retry_exhausted.Begin(key, 1000U));
    assert(retry_exhausted.RecordCompletion(key, 4U, false, 1100U) ==
           TargetCompletionResult::RetryRequired);
    const TargetRetryRequest retry = retry_exhausted.retry_request();
    assert(retry_exhausted.MarkRetryQueued(retry));
    assert(retry_exhausted.RecordCompletion(key, 4U, false, 1200U) ==
           TargetCompletionResult::Failed);
    assert(!retry_exhausted.active());
    assert(retry_exhausted.diagnostics().retry_count == 1U);
    assert(retry_exhausted.diagnostics().retry_exhausted_count == 1U);

    TargetCompletionTracker deadline(15000U);
    assert(deadline.Begin(key, 0xFFFFFF00U));
    assert(deadline.CheckDeadline(0x00003997U) ==
           TargetCompletionResult::Awaiting);
    assert(deadline.CheckDeadline(0x00003998U) ==
           TargetCompletionResult::Failed);
    assert(deadline.diagnostics().deadline_failure_count == 1U);
    assert(deadline.diagnostics().max_fanout_us == 15000U);
}

void TestTargetCompletionSafetyCancelRejectsStaleRetry()
{
    TargetCompletionTracker tracker;
    const TargetFanoutKey key{11U, 28U, 8U};
    assert(tracker.Begin(key, 10U));
    assert(tracker.RecordCompletion(key, 6U, false, 20U) ==
           TargetCompletionResult::RetryRequired);
    const TargetRetryRequest retry = tracker.retry_request();
    tracker.Cancel();
    assert(!tracker.active());
    assert(!tracker.MarkRetryQueued(retry));
    assert(tracker.RecordCompletion(key, 6U, true, 30U) ==
           TargetCompletionResult::Ignored);
}

void TestActuatorApplicationTrackerRequiresEverySuccessfulNode()
{
    ActuatorApplicationTracker tracker;
    for (uint8_t node_id = 1U; node_id <= 6U; ++node_id)
        assert(!tracker.RecordTransmission(10U, node_id, true));
    assert(!tracker.RecordTransmission(10U, 7U, false));
    assert(tracker.RecordTransmission(10U, 7U, true));
    assert(!tracker.RecordTransmission(10U, 7U, true));

    assert(!tracker.RecordTransmission(11U, 7U, true));
    for (uint8_t node_id = 1U; node_id <= 5U; ++node_id)
        assert(!tracker.RecordTransmission(11U, node_id, true));
    assert(tracker.RecordTransmission(11U, 6U, true));

    tracker.Reset();
    assert(!tracker.RecordTransmission(11U, 1U, true));

    // Even a sequence superseded before its first successful node write is
    // reported exactly, rather than disappearing from the lifecycle.
    assert(!tracker.RecordTransmission(12U, 1U, false));
    assert(!tracker.RecordTransmission(13U, 1U, true));
    assert(tracker.TakeSupersededSequence() == 12U);
}

void TestActuatorApplicationTrackerRejectsAbortAtEveryNode()
{
    for (uint8_t aborted_node = 1U;
         aborted_node <= kActuatorNodeCount; ++aborted_node)
    {
        ActuatorApplicationTracker tracker;
        const uint32_t sequence = 100U + aborted_node;
        for (uint8_t node_id = 1U; node_id <= kActuatorNodeCount; ++node_id)
        {
            const bool transmitted = node_id != aborted_node;
            assert(!tracker.RecordTransmission(sequence, node_id, transmitted));
        }
        // Repeated notifications for successful nodes cannot fill the missing
        // bit or manufacture an exact seven-node completion.
        for (uint8_t node_id = 1U; node_id <= kActuatorNodeCount; ++node_id)
        {
            if (node_id != aborted_node)
                assert(!tracker.RecordTransmission(sequence, node_id, true));
        }
    }
}

void TestLatestTargetExecutorIsBoundedAndHolds()
{
    ExecutorConfig config{};
    config.max_acceleration_rad_s2.fill(1.0F);
    config.gripper_max_velocity_per_s = 0.2F;
    config.gripper_max_acceleration_per_s2 = 0.8F;
    config.loop_rate_hz = 200;
    ExternalTargetExecutor executor(config);

    std::array<float, 7> measured{};
    ExecutorTarget target{};
    target.position[0] = 1.0F;
    target.position[6] = 0.75F;
    target.max_velocity_rad_s.fill(0.2F);
    target.sequence = 10;
    target.valid = true;

    const ExecutorStep first = executor.Step(target, true, measured);
    assert(first.command_valid);
    assert(first.sequence == 10);
    assert(first.position[0] > 0.0F);
    assert(first.position[0] <= 0.0000251F);
    assert(first.position[6] > 0.0F);
    assert(first.position[6] <= 0.0000201F);

    // A newer target replaces the old target immediately, while acceleration
    // limiting prevents an instantaneous velocity reversal.
    const float velocity_before = executor.commanded_velocity()[0];
    target.position[0] = -1.0F;
    target.sequence = 11;
    const ExecutorStep latest = executor.Step(target, true, measured);
    assert(latest.sequence == 11);
    assert(executor.commanded_velocity()[0] >= velocity_before - 0.005001F);

    ExecutorTarget none{};
    const ExecutorStep hold = executor.Step(none, false, measured);
    assert(!hold.command_valid);
    assert(hold.entered_hold);
    assert(!executor.active());
    assert(!executor.Step(none, false, measured).entered_hold);
}

void TestExecutorRejectsInvalidRuntimeLimits()
{
    ExecutorConfig config{};
    config.max_acceleration_rad_s2.fill(1.0F);
    config.gripper_max_velocity_per_s = 0.2F;
    config.gripper_max_acceleration_per_s2 = 0.8F;
    ExternalTargetExecutor executor(config);
    std::array<float, 7> measured{};
    ExecutorTarget target{};
    target.valid = true;
    target.sequence = 1;
    target.max_velocity_rad_s.fill(0.1F);
    target.max_velocity_rad_s[2] = 0.0F;
    const ExecutorStep result = executor.Step(target, true, measured);
    assert(!result.command_valid);
    assert(result.entered_hold);
}

void TestLatestTargetWinsBeforeApplication()
{
    ControlSession session(MakeConfig(true), "test-fw");
    session.SetControlReady(true);
    const Packet hello = MakeConfiguredHello();
    session.Process(hello, 1000);
    Packet acquire = MakePacket(MessageType::AcquireControl, hello.header.session_id,
                                hello.header.sequence + 1);
    SetPayload(acquire, AcquireControlPayload{500});
    session.Process(acquire, 2000);
    Packet mode = MakePacket(MessageType::SetMode, hello.header.session_id,
                             hello.header.sequence + 2);
    SetPayload(mode, SetModePayload{static_cast<uint8_t>(ControlMode::Teleop)});
    session.Process(mode, 3000);

    JointTargetPayload target{};
    std::copy(dummy::generated_config::kInitialPoseRad.begin(),
              dummy::generated_config::kInitialPoseRad.end(), target.target);
    target.target[2] = -1.0F;
    target.target[6] = 0.5F;
    for (size_t index = 0; index < 6; ++index)
        target.max_velocity[index] = 0.1F;
    target.valid_for_ms = 100;
    target.control_tick_id = 1U;

    Packet first = MakePacket(MessageType::SetJointTarget, hello.header.session_id,
                              hello.header.sequence + 3);
    target.target[0] = 0.1F;
    SetPayload(first, target);
    assert(ResponseCode(session.Process(first, 4000)) == ResultCode::Ok);

    Packet latest = MakePacket(MessageType::SetJointTarget, hello.header.session_id,
                               hello.header.sequence + 4);
    target.target[0] = 0.2F;
    target.control_tick_id = 2U;
    SetPayload(latest, target);
    assert(ResponseCode(session.Process(latest, 5000)) == ResultCode::Ok);
    assert(session.active_target().sequence == latest.header.sequence);
    assert(std::fabs(session.active_target().target[0] - 0.2F) < 1e-6F);
    assert(session.last_received_sequence() == latest.header.sequence);
}

void TestLeaseTimeoutAndSessionIndependentEstop()
{
    ControlSession session(MakeConfig(true), "test-fw");
    session.SetControlReady(true);
    const Packet hello = MakeConfiguredHello();
    session.Process(hello, 1000);
    Packet acquire = MakePacket(MessageType::AcquireControl, hello.header.session_id,
                                hello.header.sequence + 1);
    SetPayload(acquire, AcquireControlPayload{500});
    session.Process(acquire, 2000);
    Packet mode = MakePacket(MessageType::SetMode, hello.header.session_id,
                             hello.header.sequence + 2);
    SetPayload(mode, SetModePayload{static_cast<uint8_t>(ControlMode::Teleop)});
    session.Process(mode, 3000);
    // SET_MODE and targets never extend the lease; only ACQUIRE establishes it
    // and HEARTBEAT refreshes it.
    assert(!session.Tick(501999));
    assert(session.Tick(502000));
    assert(!session.lease_active());
    assert(session.mode() == ControlMode::Hold);

    Packet estop = MakePacket(MessageType::EmergencyStop, 0xDEADBEEFU, 1);
    const ProcessResult stopped = session.Process(estop, 504000);
    assert(ResponseCode(stopped) == ResultCode::Ok);
    assert(stopped.emergency_stop_requested);
    assert(session.mode() == ControlMode::Fault);
}
} // namespace

int main()
{
    TestSpscRingUsesAllSlotsAndWrapsWithoutOverwrite();
    TestPublishedDoubleBufferReturnsWholeSnapshots();
    TestCodecVectors();
    TestMonotonicMicrosIgnoresSmallRegressionAndExtendsWrap();
    TestMeasuredVelocityUsesOnlyValidMonotonicIntervals();
    TestCanFeedbackMonitorTracksAgeAndLossWithoutInventingFaultSources();
    TestCanFeedbackMonitorClampsConcurrentFutureTimestampAndPreservesWrap();
    TestCanFeedbackMonitorPublishesOnlyCompleteLowSkewSweep();
    TestStateValidityBitsComeFromOneFeedbackSnapshot();
    TestCanTransportStatusSurvivesStatePayloadCopy();
    TestFeedbackSafetyPersistenceSeparatesHoldFromLatchedFault();
    TestControlAcquisitionRequiresFeedbackBootstrap();
    TestCanDispatcherTransitionsAndFrequencyPlan();
    TestCanDispatcherWaitsForResponseAndTimesOut();
    TestCanDispatcherContinuesSweepAndRetriesOnlyMissingNode();
    TestCanDispatcherAcceptsLateSweepResponseAndReportsRetryExhaustion();
    TestCanDispatcherPreflightTimeoutStopsEnable();
    TestCanDispatcherDoesNotBurstAfterDeferredDeadline();
    TestCanDispatcherRejectsInvalidRatePlanWithoutFallback();
    TestCanDispatcherBootstrapsEveryNodeAndFaultPreemptsQuery();
    TestSafetyModesPreemptPartialTargetFanout();
    TestTargetCompletionRetriesOnlyTheExactFailedNode();
    TestTargetCompletionFailsOnSecondErrorOrFanoutDeadline();
    TestTargetCompletionSafetyCancelRejectsStaleRetry();
    TestActuatorApplicationTrackerRequiresEverySuccessfulNode();
    TestActuatorApplicationTrackerRejectsAbortAtEveryNode();
    TestUrdfJointSpaceMapping();
    TestUnverifiedConfigurationCannotAcquire();
    TestCanDiagnosticsAreReadOnlyAfterConfiguredHello();
    TestSessionTargetAndTimeout();
    TestTargetKeepaliveIsExactAndControlBound();
    TestTelemetryMovesToLatestHelloAfterRelease();
    TestBadSequenceAndTargetAreRejected();
    TestSequenceWrapUsesUint32SerialArithmetic();
    TestReliableControlMayOvertakeAnOlderMotionTarget();
    TestLatestTargetExecutorIsBoundedAndHolds();
    TestExecutorRejectsInvalidRuntimeLimits();
    TestLatestTargetWinsBeforeApplication();
    TestLeaseTimeoutAndSessionIndependentEstop();
    std::cout << "dummy protocol host tests passed\n";
    return 0;
}
