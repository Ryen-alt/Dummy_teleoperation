#include "binary_control_session.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace dummy::protocol
{
namespace
{
template<typename T>
bool ReadPayload(const Packet& packet, T& output)
{
    if (packet.header.payload_length != sizeof(T))
        return false;
    std::memcpy(&output, packet.payload.data(), sizeof(T));
    return true;
}

template<typename T>
void WritePayload(Packet& packet, const T& payload)
{
    static_assert(sizeof(T) <= kMaxPayload, "payload too large");
    packet.header.payload_length = sizeof(T);
    std::memcpy(packet.payload.data(), &payload, sizeof(T));
}

bool HashMatches(const uint8_t* lhs, const std::array<uint8_t, 32>& rhs)
{
    uint8_t difference = 0;
    for (size_t index = 0; index < rhs.size(); ++index)
        difference |= static_cast<uint8_t>(lhs[index] ^ rhs[index]);
    return difference == 0;
}

bool IsNewerSequence(uint32_t candidate, uint32_t previous)
{
    const uint32_t delta = candidate - previous;
    return delta != 0U && delta < 0x80000000U;
}
} // namespace

ControlSession::ControlSession(const SessionConfig& config, const char* firmware_version)
    : config_(config)
{
    if (firmware_version != nullptr)
        std::strncpy(firmware_version_.data(), firmware_version, firmware_version_.size() - 1);
}

ProcessResult ControlSession::Process(const Packet& request, uint64_t now_us)
{
    const auto message_type = static_cast<MessageType>(request.header.message_type);
    if (message_type == MessageType::Hello)
        return Hello(request);
    if (message_type == MessageType::TimeSync)
    {
        TimeSyncPayload payload{};
        if (!hello_valid_ || request.header.session_id != hello_session_id_)
            return Ack(request, ResultCode::BadSession);
        if (!ReadPayload(request, payload))
            return Ack(request, ResultCode::BadLength);
        ProcessResult result{};
        result.response.header.message_type = static_cast<uint8_t>(
            MessageType::TimeSyncAck);
        result.response.header.session_id = request.header.session_id;
        result.response.header.sequence = request.header.sequence;
        WritePayload(result.response, TimeSyncAckPayload{
            payload.host_t0_ns, now_us, now_us});
        return result;
    }
    if (message_type == MessageType::EmergencyStop)
    {
        SetFault(kFaultEmergencyStop);
        ProcessResult result = Ack(request);
        result.emergency_stop_requested = true;
        result.entered_hold = true;
        return result;
    }
    if (message_type == MessageType::Hold)
    {
        EnterHold(kHoldReasonOperator);
        ProcessResult result = Ack(request);
        result.entered_hold = true;
        return result;
    }

    if (message_type == MessageType::AcquireControl)
    {
        AcquireControlPayload payload{};
        if (!hello_valid_ || request.header.session_id != hello_session_id_)
            return Ack(request, ResultCode::BadConfig);
        if (!ReadPayload(request, payload))
            return Ack(request, ResultCode::BadLength);
        if (!config_.hardware_parameters_verified)
            return Ack(request, ResultCode::BadConfig);
        if (fault_bits_ != 0)
            return Ack(request, ResultCode::FaultActive);
        if (!control_ready_)
            return Ack(request, ResultCode::BadMode,
                       kAckDetailFeedbackNotReady);
        if (lease_active_ && request.header.session_id != session_id_)
            return Ack(request, ResultCode::LeaseConflict);
        if (payload.lease_ms == 0 || payload.lease_ms > config_.max_lease_ms)
            return Ack(request, ResultCode::OutOfRange);
        const bool same_active_epoch = lease_active_ &&
            request.header.session_id == session_id_;
        const uint32_t acquire_watermark = same_active_epoch
            ? last_command_sequence_ : hello_sequence_;
        if (!IsNewerSequence(request.header.sequence, acquire_watermark))
            return Ack(request, ResultCode::BadSequence);

        session_id_ = request.header.session_id;
        last_command_sequence_ = request.header.sequence;
        if (!same_active_epoch)
        {
            last_target_sequence_ = request.header.sequence;
            last_received_sequence_ = 0;
            last_control_tick_id_ = 0;
        }
        lease_duration_ms_ = payload.lease_ms;
        lease_active_ = true;
        mode_ = ControlMode::Hold;
        hold_reason_bits_ = 0;
        active_target_ = {};
        ExtendLease(now_us);
        return Ack(request);
    }

    ProcessResult validation{};
    if (!ValidateSession(request, validation))
        return validation;
    if (message_type == MessageType::SetJointTarget)
    {
        if (!IsNewerSequence(request.header.sequence, last_target_sequence_))
            return Ack(request, ResultCode::BadSequence);
        last_target_sequence_ = request.header.sequence;
        // Keep the reliable-control watermark monotonic when target and
        // control packets happen to arrive in enqueue order.  Never move it
        // backwards when a reliable packet legitimately overtook a target.
        if (IsNewerSequence(request.header.sequence, last_command_sequence_))
            last_command_sequence_ = request.header.sequence;
    }
    else
    {
        if (!IsNewerSequence(request.header.sequence, last_command_sequence_))
            return Ack(request, ResultCode::BadSequence);
        last_command_sequence_ = request.header.sequence;
    }

    switch (message_type)
    {
        case MessageType::ReleaseControl:
        {
            if (request.header.payload_length != 0)
                return Ack(request, ResultCode::BadLength);
            EnterHold(kHoldReasonOperator);
            lease_active_ = false;
            ProcessResult result = Ack(request);
            result.entered_hold = true;
            return result;
        }
        case MessageType::SetMode:
        {
            SetModePayload payload{};
            if (!ReadPayload(request, payload))
                return Ack(request, ResultCode::BadLength);
            const auto requested_mode = static_cast<ControlMode>(payload.mode);
            if (requested_mode != ControlMode::Hold && requested_mode != ControlMode::Teleop &&
                requested_mode != ControlMode::Policy)
                return Ack(request, ResultCode::BadMode);
            if (fault_bits_ != 0)
                return Ack(request, ResultCode::FaultActive);
            active_target_ = {};
            mode_ = requested_mode;
            hold_reason_bits_ = requested_mode == ControlMode::Hold
                ? kHoldReasonOperator : 0;
            ProcessResult result = Ack(request);
            result.entered_hold = requested_mode == ControlMode::Hold;
            return result;
        }
        case MessageType::Heartbeat:
            if (request.header.payload_length != 0)
                return Ack(request, ResultCode::BadLength);
            ExtendLease(now_us);
            return Ack(request);
        case MessageType::TargetKeepalive:
        {
            TargetKeepalivePayload payload{};
            if (!ReadPayload(request, payload))
                return Ack(request, ResultCode::BadLength);
            if (mode_ != ControlMode::Teleop && mode_ != ControlMode::Policy)
                return Ack(request, ResultCode::BadMode);
            if (!active_target_.valid ||
                payload.action_sequence != active_target_.sequence)
                return Ack(request, ResultCode::BadSequence);
            if (now_us >= active_target_.deadline_us)
            {
                EnterHold(kHoldReasonTargetTimeout);
                return Ack(request, ResultCode::Expired);
            }
            if (!IsNewerSequence(payload.control_tick_id,
                                 last_control_tick_id_))
                return Ack(request, ResultCode::BadSequence);
            active_target_.last_refresh_time_us = now_us;
            active_target_.deadline_us = now_us +
                static_cast<uint64_t>(active_target_.valid_for_ms) * 1000U;
            active_target_.control_tick_id = payload.control_tick_id;
            last_control_tick_id_ = payload.control_tick_id;
            return Ack(request);
        }
        case MessageType::SetJointTarget:
        {
            JointTargetPayload payload{};
            if (!ReadPayload(request, payload))
                return Ack(request, ResultCode::BadLength);
            if (mode_ != ControlMode::Teleop && mode_ != ControlMode::Policy)
                return Ack(request, ResultCode::BadMode);
            const ResultCode target_result = ValidateTarget(payload);
            if (target_result != ResultCode::Ok)
                return Ack(request, target_result);
            if (!IsNewerSequence(payload.control_tick_id,
                                 last_control_tick_id_))
                return Ack(request, ResultCode::BadSequence);

            for (size_t index = 0; index < active_target_.target.size(); ++index)
                active_target_.target[index] = payload.target[index];
            for (size_t index = 0; index < active_target_.max_velocity.size(); ++index)
                active_target_.max_velocity[index] = payload.max_velocity[index];
            active_target_.sequence = request.header.sequence;
            active_target_.received_time_us = now_us;
            active_target_.last_refresh_time_us = now_us;
            active_target_.deadline_us = now_us + static_cast<uint64_t>(payload.valid_for_ms) * 1000U;
            active_target_.valid_for_ms = payload.valid_for_ms;
            active_target_.flags = payload.target_flags;
            active_target_.control_tick_id = payload.control_tick_id;
            active_target_.valid = true;
            last_received_sequence_ = request.header.sequence;
            last_control_tick_id_ = payload.control_tick_id;
            ProcessResult result = Ack(request);
            result.target_updated = true;
            return result;
        }
        case MessageType::ClearFault:
            return Ack(request, ResultCode::Unsupported);
        case MessageType::GetCanDiagnostics:
        {
            if (request.header.payload_length != 0U)
                return Ack(request, ResultCode::BadLength);
            ProcessResult result = Ack(request);
            result.can_diagnostics_requested = true;
            return result;
        }
        default:
            return Ack(request, ResultCode::Unsupported);
    }
}

bool ControlSession::Tick(uint64_t now_us)
{
    if (lease_active_ && now_us >= lease_deadline_us_)
    {
        EnterHold(kHoldReasonLeaseTimeout);
        lease_active_ = false;
        return true;
    }
    if (active_target_.valid && now_us >= active_target_.deadline_us)
    {
        EnterHold(kHoldReasonTargetTimeout);
        return true;
    }
    return false;
}

void ControlSession::SetFault(uint16_t fault_bits)
{
    fault_bits_ |= fault_bits;
    mode_ = ControlMode::Fault;
    lease_active_ = false;
    active_target_ = {};
}

void ControlSession::RequestSafetyHold(uint16_t hold_reason_bits)
{
    if (hold_reason_bits == 0 || fault_bits_ != 0)
        return;
    EnterHold(hold_reason_bits);
}

bool ControlSession::ClearFault(bool hardware_safe)
{
    if (!hardware_safe)
        return false;
    fault_bits_ = 0;
    mode_ = ControlMode::Hold;
    hold_reason_bits_ = 0;
    active_target_ = {};
    return true;
}

StatePayload ControlSession::MakeState(const std::array<float, 7>& position,
                                       const std::array<float, 7>& velocity,
                                       uint8_t validity, uint64_t now_us,
                                       const FeedbackSafetyOutput& safety) const
{
    StatePayload state{};
    state.mcu_time_us = now_us;
    std::copy(position.begin(), position.end(), state.position);
    std::copy(velocity.begin(), velocity.end(), state.velocity);
    state.last_received_sequence = last_received_sequence_;
    state.mode = static_cast<uint8_t>(mode_);
    state.validity = validity;
    state.fault_bits = fault_bits_;
    if (active_target_.valid && now_us >= active_target_.last_refresh_time_us)
        state.target_age_ms = static_cast<uint32_t>(
            (now_us - active_target_.last_refresh_time_us) / 1000U);
    std::copy(config_.config_sha256.begin(), config_.config_sha256.end(), state.config_sha256);
    std::copy(safety.following_error.begin(), safety.following_error.end(),
              state.following_error);
    std::copy(safety.following_error_duration_ms.begin(),
              safety.following_error_duration_ms.end(),
              state.following_error_duration_ms);
    std::copy(safety.feedback_age_ms.begin(), safety.feedback_age_ms.end(),
              state.feedback_age_ms);
    std::copy(safety.feedback_loss_count.begin(), safety.feedback_loss_count.end(),
              state.feedback_loss_count);
    std::copy(safety.consecutive_feedback_loss.begin(),
              safety.consecutive_feedback_loss.end(),
              state.consecutive_feedback_loss);
    std::copy(safety.node_fault_bits.begin(), safety.node_fault_bits.end(),
              state.node_fault_bits);
    std::copy(safety.node_validity.begin(), safety.node_validity.end(),
              state.node_validity);
    state.hold_reason_bits = hold_reason_bits_;
    state.telemetry_validity = safety.telemetry_validity;
    return state;
}

ProcessResult ControlSession::Ack(const Packet& request, ResultCode result, uint16_t detail) const
{
    ProcessResult output{};
    output.response.header.message_type = static_cast<uint8_t>(
        result == ResultCode::Ok ? MessageType::Ack : MessageType::Nack);
    output.response.header.flags = 0;
    output.response.header.session_id = request.header.session_id;
    output.response.header.sequence = request.header.sequence;
    output.response.header.sender_time_us = 0;
    AckPayload payload{
        request.header.message_type,
        static_cast<uint8_t>(result),
        detail,
    };
    WritePayload(output.response, payload);
    return output;
}

ProcessResult ControlSession::Hello(const Packet& request)
{
    HelloPayload payload{};
    if (request.header.session_id == 0U)
        return Ack(request, ResultCode::BadSession);
    if (!ReadPayload(request, payload))
        return Ack(request, ResultCode::BadLength);
    if (!HashMatches(payload.config_sha256, config_.config_sha256))
    {
        hello_valid_ = false;
        return Ack(request, ResultCode::BadConfig);
    }

    const bool new_epoch = !hello_valid_ ||
        request.header.session_id != hello_session_id_;
    if (new_epoch)
    {
        lease_active_ = false;
        active_target_ = {};
        mode_ = fault_bits_ == 0U ? ControlMode::Hold : ControlMode::Fault;
    }
    hello_valid_ = true;
    hello_session_id_ = request.header.session_id;
    hello_sequence_ = request.header.sequence;
    ProcessResult output{};
    output.response.header.message_type = static_cast<uint8_t>(MessageType::HelloAck);
    output.response.header.flags = 0;
    output.response.header.session_id = request.header.session_id;
    output.response.header.sequence = request.header.sequence;
    output.response.header.sender_time_us = 0;
    HelloAckPayload response{};
    std::copy(config_.config_sha256.begin(), config_.config_sha256.end(), response.config_sha256);
    response.capabilities =
        kCapabilityMultiChannelSequence | kCapabilityTargetKeepalive |
        kCapabilityCanTxCompleteExact |
        kCapabilityControlFreshnessToken | kCapabilityTimeSync |
        kCapabilityCanDiagnostics | kCapabilityCanDiagnosticsV2;
    std::copy(firmware_version_.begin(), firmware_version_.end(), response.firmware_version);
    WritePayload(output.response, response);
    return output;
}

bool ControlSession::ValidateSession(const Packet& request, ProcessResult& result) const
{
    if (!lease_active_)
    {
        result = Ack(request, ResultCode::NoLease);
        return false;
    }
    if (request.header.session_id != session_id_)
    {
        result = Ack(request, ResultCode::BadSession);
        return false;
    }
    return true;
}

ResultCode ControlSession::ValidateTarget(const JointTargetPayload& target) const
{
    if (target.valid_for_ms == 0 || target.valid_for_ms > config_.max_target_ttl_ms)
        return ResultCode::Expired;
    for (size_t index = 0; index < 7; ++index)
    {
        if (!std::isfinite(target.target[index]))
            return ResultCode::NonFinite;
    }
    for (size_t index = 0; index < 6; ++index)
    {
        if (!std::isfinite(target.max_velocity[index]))
            return ResultCode::NonFinite;
        if (target.target[index] < config_.joint_min_rad[index] ||
            target.target[index] > config_.joint_max_rad[index] ||
            target.max_velocity[index] <= 0 ||
            target.max_velocity[index] > config_.max_velocity_rad_s[index])
            return ResultCode::OutOfRange;
    }
    if (target.target[6] < 0.0F || target.target[6] > 1.0F)
        return ResultCode::OutOfRange;
    return ResultCode::Ok;
}

void ControlSession::EnterHold(uint16_t hold_reason_bits)
{
    mode_ = fault_bits_ == 0 ? ControlMode::Hold : ControlMode::Fault;
    hold_reason_bits_ |= hold_reason_bits;
    active_target_ = {};
}

void ControlSession::ExtendLease(uint64_t now_us)
{
    lease_deadline_us_ = now_us + static_cast<uint64_t>(lease_duration_ms_) * 1000U;
}

} // namespace dummy::protocol
