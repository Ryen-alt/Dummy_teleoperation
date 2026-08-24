#include "binary_control_session.hpp"
#include "binary_protocol.hpp"
#include "external_target_executor.hpp"
#include "joint_space_mapping.hpp"
#include "monotonic_micros.hpp"
#include "measured_state_estimator.hpp"
#include "can_feedback_monitor.hpp"
#include "feedback_safety_supervisor.hpp"
#include "feedback_poll_scheduler.hpp"
#include "robot_config_generated.hpp"

#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

using namespace dummy::protocol;

namespace
{
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
        "06594402012401013944332211887766550807060504030201"
        "cefed4f980ea7592d9108bfbc5575d1d0aebc5cf319cf41d"
        "40421a73f5043d4a5a5aa5a5ca8f978e00");
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
        "06594402012401013944332211887766550807060504030201"
        "cefed4f980ea7592d9108bfbc5575d1d0aebc5cf319cf41d"
        "40421a73f5043d4a5a5aa5a5ca8f978e00");
    assert(length == expected.size());
    assert(std::equal(expected.begin(), expected.end(), output.begin()));

    output[length - 3] ^= 0x40;
    Packet invalid{};
    assert(DecodePacket(output.data(), length, invalid) == DecodeStatus::BadCrc);

    const auto target_wire = FromHex(
        "06594402063801012544332211897766551007060504030201"
        "cdcccc3dcdcc4c3e9a9999bf9a99993ecdccccbe0101023f01"
        "0f403f3333b33e3333b33e3333b33e0101023f0101073f33"
        "33333f64020305a0621ac500");
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
    assert(target.valid_for_ms == 100);
    assert(target.target_flags == 3);
    const size_t target_length = EncodePacket(target_packet, output.data(), output.size());
    assert(target_length == target_wire.size());
    assert(std::equal(target_wire.begin(), target_wire.end(), output.begin()));
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
}

void TestMeasuredVelocityUsesOnlyValidMonotonicIntervals()
{
    MeasuredStateEstimator fast_estimator;
    std::array<float, 7> fast_position{};
    assert(!fast_estimator.Update(fast_position, 1000U, true).valid);
    fast_position[0] = 1.0F;
    assert(!fast_estimator.Update(fast_position, 1500U, true).valid);

    MeasuredStateEstimator estimator(100000U);
    std::array<float, 7> position{};
    assert(!estimator.Update(position, 1000U, true).valid);

    position[0] = 0.01F;
    position[6] = 0.1F;
    const VelocityEstimate moving = estimator.Update(position, 51000U, true);
    assert(moving.valid);
    assert(std::fabs(moving.velocity[0] - 0.2F) < 1e-6F);
    assert(std::fabs(moving.velocity[6] - 2.0F) < 1e-6F);

    // A repeated timestamp and a long feedback gap both invalidate exactly one
    // interval and rebase the next estimate instead of emitting a spike.
    assert(!estimator.Update(position, 51000U, true).valid);
    position[0] = 0.02F;
    assert(!estimator.Update(position, 200000U, true).valid);
    assert(!estimator.Update(position, 250000U, false).valid);
    assert(!estimator.Update(position, 300000U, true).valid);
    position[0] = 0.03F;
    assert(estimator.Update(position, 350000U, true).valid);
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

void TestSessionTargetAndTimeout()
{
    ControlSession session(MakeConfig(true), "test-fw");
    const Packet hello = MakeConfiguredHello();
    session.Process(hello, 1000);

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
    Packet command = MakePacket(MessageType::SetJointTarget, hello.header.session_id,
                                hello.header.sequence + 3);
    SetPayload(command, target);
    ProcessResult result = session.Process(command, 4000);
    assert(ResponseCode(result) == ResultCode::Ok);
    assert(result.target_updated);
    assert(session.active_target().valid);
    assert(std::fabs(session.active_target().target[2] + 1.2F) < 1e-6F);
    session.MarkTargetApplied(command.header.sequence);
    assert(session.last_applied_sequence() == command.header.sequence);

    assert(!session.Tick(103999));
    assert(session.Tick(104000));
    assert(session.mode() == ControlMode::Hold);
    assert(!session.active_target().valid);
}

void TestTelemetryMovesToLatestHelloAfterRelease()
{
    ControlSession session(MakeConfig(true), "test-fw");
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
    Packet bad = MakePacket(MessageType::SetJointTarget, hello.header.session_id,
                            hello.header.sequence + 3);
    target.target[0] = 100.0F;
    SetPayload(bad, target);
    assert(ResponseCode(session.Process(bad, 4)) == ResultCode::OutOfRange);

    Packet repeated = MakePacket(MessageType::Heartbeat, hello.header.session_id,
                                 hello.header.sequence + 3);
    assert(ResponseCode(session.Process(repeated, 5)) == ResultCode::BadSequence);
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

void TestFeedbackPollSchedulerUsesOneRequestPerSlot()
{
    constexpr uint32_t kTemperatureSlotInterval = 100;
    FeedbackPollScheduler scheduler(kTemperatureSlotInterval);
    std::array<uint32_t, kActuatorNodeCount> position_counts{};
    std::array<uint32_t, kActuatorNodeCount> temperature_counts{};

    for (uint32_t slot = 0; slot < 700; ++slot)
    {
        const FeedbackPollRequest request = scheduler.Next();
        assert(request.node_id >= 1 && request.node_id <= kActuatorNodeCount);
        if (request.kind == FeedbackPollKind::Position)
            ++position_counts[request.node_id - 1U];
        else
            ++temperature_counts[request.node_id - 1U];
    }

    for (size_t index = 0; index < kActuatorNodeCount; ++index)
    {
        assert(position_counts[index] == 99U);
        assert(temperature_counts[index] == 1U);
    }

    scheduler.Reset();
    for (uint8_t expected_node = 1; expected_node <= 7; ++expected_node)
    {
        const FeedbackPollRequest request = scheduler.Next();
        assert(request.kind == FeedbackPollKind::Position);
        assert(request.node_id == expected_node);
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

    Packet first = MakePacket(MessageType::SetJointTarget, hello.header.session_id,
                              hello.header.sequence + 3);
    target.target[0] = 0.1F;
    SetPayload(first, target);
    assert(ResponseCode(session.Process(first, 4000)) == ResultCode::Ok);

    Packet latest = MakePacket(MessageType::SetJointTarget, hello.header.session_id,
                               hello.header.sequence + 4);
    target.target[0] = 0.2F;
    SetPayload(latest, target);
    assert(ResponseCode(session.Process(latest, 5000)) == ResultCode::Ok);
    assert(session.active_target().sequence == latest.header.sequence);
    assert(std::fabs(session.active_target().target[0] - 0.2F) < 1e-6F);
    assert(session.last_received_sequence() == latest.header.sequence);
    assert(session.last_applied_sequence() == 0);

    session.MarkTargetApplied(first.header.sequence);
    assert(session.last_applied_sequence() == 0);
    session.MarkTargetApplied(latest.header.sequence);
    assert(session.last_applied_sequence() == latest.header.sequence);
}

void TestLeaseTimeoutAndSessionIndependentEstop()
{
    ControlSession session(MakeConfig(true), "test-fw");
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
    assert(!session.Tick(502999));
    assert(session.Tick(503000));
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
    TestCodecVectors();
    TestMonotonicMicrosIgnoresSmallRegressionAndExtendsWrap();
    TestMeasuredVelocityUsesOnlyValidMonotonicIntervals();
    TestCanFeedbackMonitorTracksAgeAndLossWithoutInventingFaultSources();
    TestFeedbackSafetyPersistenceSeparatesHoldFromLatchedFault();
    TestFeedbackPollSchedulerUsesOneRequestPerSlot();
    TestUrdfJointSpaceMapping();
    TestUnverifiedConfigurationCannotAcquire();
    TestSessionTargetAndTimeout();
    TestTelemetryMovesToLatestHelloAfterRelease();
    TestBadSequenceAndTargetAreRejected();
    TestLatestTargetExecutorIsBoundedAndHolds();
    TestExecutorRejectsInvalidRuntimeLimits();
    TestLatestTargetWinsBeforeApplication();
    TestLeaseTimeoutAndSessionIndependentEstop();
    std::cout << "dummy protocol host tests passed\n";
    return 0;
}
