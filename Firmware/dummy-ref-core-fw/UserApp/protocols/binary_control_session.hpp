#ifndef DUMMY_BINARY_CONTROL_SESSION_HPP
#define DUMMY_BINARY_CONTROL_SESSION_HPP

#include "binary_protocol.hpp"
#include "feedback_safety_supervisor.hpp"

#include <array>
#include <cstdint>

namespace dummy::protocol
{

struct SessionConfig
{
    std::array<uint8_t, 32> config_sha256{};
    std::array<float, 6> joint_min_rad{};
    std::array<float, 6> joint_max_rad{};
    std::array<float, 6> max_velocity_rad_s{};
    bool hardware_parameters_verified = false;
    uint32_t max_lease_ms = 1000;
    uint16_t max_target_ttl_ms = 250;
};

struct ActiveTarget
{
    std::array<float, 7> target{};
    std::array<float, 6> max_velocity{};
    uint32_t sequence = 0;
    uint64_t received_time_us = 0;
    uint64_t last_refresh_time_us = 0;
    uint64_t deadline_us = 0;
    uint16_t valid_for_ms = 0;
    uint16_t flags = 0;
    uint32_t control_tick_id = 0;
    bool valid = false;
};

struct ProcessResult
{
    Packet response{};
    bool target_updated = false;
    bool entered_hold = false;
    bool emergency_stop_requested = false;
    bool can_diagnostics_requested = false;
};

class ControlSession
{
public:
    ControlSession(const SessionConfig& config, const char* firmware_version);

    ProcessResult Process(const Packet& request, uint64_t now_us);
    bool Tick(uint64_t now_us);

    ControlMode mode() const { return mode_; }
    bool hello_valid() const { return hello_valid_; }
    bool lease_active() const { return lease_active_; }
    uint32_t session_id() const { return session_id_; }
    uint32_t telemetry_session_id() const
    { return lease_active_ ? session_id_ : hello_session_id_; }
    uint32_t last_received_sequence() const { return last_received_sequence_; }
    const ActiveTarget& active_target() const { return active_target_; }
    void SetControlReady(bool ready) { control_ready_ = ready; }
    bool control_ready() const { return control_ready_; }
    void SetFault(uint16_t fault_bits);
    void RequestSafetyHold(uint16_t hold_reason_bits);
    bool ClearFault(bool hardware_safe);
    uint16_t hold_reason_bits() const { return hold_reason_bits_; }

    StatePayload MakeState(const std::array<float, 7>& position,
                           const std::array<float, 7>& velocity,
                           uint8_t validity, uint64_t now_us,
                           const FeedbackSafetyOutput& safety) const;

private:
    ProcessResult Ack(const Packet& request, ResultCode result = ResultCode::Ok,
                      uint16_t detail = 0) const;
    ProcessResult Hello(const Packet& request);
    bool ValidateSession(const Packet& request, ProcessResult& result) const;
    ResultCode ValidateTarget(const JointTargetPayload& target) const;
    void EnterHold(uint16_t hold_reason_bits);
    void ExtendLease(uint64_t now_us);

    SessionConfig config_;
    std::array<char, 32> firmware_version_{};
    ControlMode mode_ = ControlMode::Disabled;
    ActiveTarget active_target_{};
    bool hello_valid_ = false;
    bool lease_active_ = false;
    bool control_ready_ = false;
    uint32_t hello_session_id_ = 0;
    uint32_t hello_sequence_ = 0;
    uint32_t session_id_ = 0;
    uint32_t last_command_sequence_ = 0;
    // Motion targets and reliable control commands are transported on
    // different priority channels.  A newer heartbeat may therefore reach
    // the MCU before an older target without making that target a replay.
    uint32_t last_target_sequence_ = 0;
    uint32_t last_received_sequence_ = 0;
    uint32_t last_control_tick_id_ = 0;
    uint32_t lease_duration_ms_ = 0;
    uint64_t lease_deadline_us_ = 0;
    uint16_t fault_bits_ = 0;
    uint16_t hold_reason_bits_ = 0;
};

} // namespace dummy::protocol

#endif // DUMMY_BINARY_CONTROL_SESSION_HPP
