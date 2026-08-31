#ifndef DUMMY_BINARY_PROTOCOL_HPP
#define DUMMY_BINARY_PROTOCOL_HPP

#include <array>
#include <cstddef>
#include <cstdint>

namespace dummy::protocol
{

constexpr uint16_t kMagic = 0x4459;
constexpr uint8_t kProtocolVersion = 5;
constexpr size_t kMaxDecodedFrame = 576;
constexpr size_t kCrcSize = 4;
constexpr size_t kActionProgressReplayCapacity = 6;
constexpr uint32_t kCapabilityMultiChannelSequence = 1U << 0U;
constexpr uint32_t kCapabilityTargetKeepalive = 1U << 1U;
constexpr uint32_t kCapabilityCanTxCompleteExact = 1U << 2U;
constexpr uint32_t kCapabilityControlFreshnessToken = 1U << 3U;
constexpr uint32_t kCapabilityTimeSync = 1U << 4U;
constexpr uint32_t kCapabilityCanDiagnostics = 1U << 5U;
constexpr uint32_t kCapabilityCanDiagnosticsV2 = 1U << 6U;
constexpr uint32_t kCapabilityCanTimingProfile = 1U << 7U;
constexpr uint16_t kCanDiagnosticsFormatVersion = 2U;
constexpr uint16_t kCanDiagnosticsPayloadSize = 380U;
constexpr uint8_t kCanDiagnosticsWindowActive = 1U << 0U;
constexpr uint8_t kCanDiagnosticsEpochStable = 1U << 1U;
constexpr uint8_t kCanDiagnosticsMotorCountersMonotonic = 1U << 2U;
constexpr uint8_t kCanDiagnosticsMarkersComplete = 1U << 3U;
constexpr uint8_t kCanDiagnosticsWindowValid =
    kCanDiagnosticsWindowActive | kCanDiagnosticsEpochStable |
    kCanDiagnosticsMotorCountersMonotonic |
    kCanDiagnosticsMarkersComplete;
constexpr uint16_t kCanTimingProfileFormatVersion = 1U;
constexpr uint16_t kCanTimingProfilePayloadSize = 520U;
constexpr uint8_t kCanTimingProfileWindowActive = 1U << 0U;
constexpr uint8_t kCanTimingProfileEpochStable = 1U << 1U;
constexpr uint8_t kCanTimingProfileMotorPagesComplete = 1U << 2U;
constexpr uint8_t kCanTimingProfileLatencySamplesValid = 1U << 3U;
constexpr uint8_t kCanTimingProfileWindowValid =
    kCanTimingProfileWindowActive | kCanTimingProfileEpochStable |
    kCanTimingProfileMotorPagesComplete |
    kCanTimingProfileLatencySamplesValid;

enum class MessageType : uint8_t
{
    Hello = 0x01,
    AcquireControl = 0x02,
    ReleaseControl = 0x03,
    SetMode = 0x04,
    Heartbeat = 0x05,
    SetJointTarget = 0x06,
    Hold = 0x07,
    EmergencyStop = 0x08,
    ClearFault = 0x09,
    TargetKeepalive = 0x0A,
    TimeSync = 0x0B,
    GetCanDiagnostics = 0x0C,
    GetCanTimingProfile = 0x0D,
    HelloAck = 0x81,
    State = 0x82,
    Ack = 0x83,
    Nack = 0x84,
    Fault = 0x85,
    Event = 0x86,
    TimeSyncAck = 0x87,
    CanDiagnostics = 0x88,
    CanTimingProfile = 0x89,
};

enum class ControlMode : uint8_t
{
    Disabled = 1,
    Hold = 2,
    Teleop = 3,
    Policy = 4,
    Gravity = 5,
    Fault = 6,
};

enum class ResultCode : uint8_t
{
    Ok = 0,
    BadVersion = 1,
    BadLength = 2,
    BadConfig = 3,
    BadSession = 4,
    BadSequence = 5,
    BadMode = 6,
    NoLease = 7,
    Expired = 8,
    NonFinite = 9,
    OutOfRange = 10,
    FaultActive = 11,
    LeaseConflict = 12,
    Unsupported = 13,
};

constexpr uint16_t kAckDetailFeedbackNotReady = 1U;

enum class DecodeStatus : uint8_t
{
    Ok,
    Empty,
    Overflow,
    MalformedCobs,
    TooShort,
    BadCrc,
    BadMagic,
    BadVersion,
    BadLength,
    UnknownMessage,
};

#pragma pack(push, 1)
struct PacketHeader
{
    uint16_t magic;
    uint8_t version;
    uint8_t message_type;
    uint16_t payload_length;
    uint16_t flags;
    uint32_t session_id;
    uint32_t sequence;
    uint64_t sender_time_us;
};

struct HelloPayload
{
    uint8_t config_sha256[32];
    uint32_t capabilities;
};

struct HelloAckPayload
{
    uint8_t config_sha256[32];
    uint32_t capabilities;
    char firmware_version[32];
};

struct AcquireControlPayload
{
    uint32_t lease_ms;
};

struct SetModePayload
{
    uint8_t mode;
};

struct JointTargetPayload
{
    float target[7];
    float max_velocity[6];
    uint16_t valid_for_ms;
    uint16_t target_flags;
    uint32_t control_tick_id;
};

struct TargetKeepalivePayload
{
    uint32_t action_sequence;
    uint32_t control_tick_id;
};

struct TimeSyncPayload
{
    uint64_t host_t0_ns;
};

struct TimeSyncAckPayload
{
    uint64_t host_t0_ns;
    uint64_t mcu_rx_us;
    uint64_t mcu_tx_us;
};

struct AckPayload
{
    uint8_t request_type;
    uint8_t result;
    uint16_t detail;
};

enum class ActionProgressStage : uint8_t
{
    CanQueuedExact = 1,
    CanTxCompleteExact = 2,
    PostCommandFeedback = 3,
    Superseded = 4,
    PreemptedBySafety = 5,
    Failed = 6,
};

constexpr uint8_t kActionProgressCanQueuedExact = 1U << 0;
constexpr uint8_t kActionProgressCanTxCompleteExact = 1U << 1;
constexpr uint8_t kActionProgressPostCommandFeedback = 1U << 2;
constexpr uint8_t kActionProgressSuperseded = 1U << 3;
constexpr uint8_t kActionProgressPreemptedBySafety = 1U << 4;
constexpr uint8_t kActionProgressFailed = 1U << 5;
constexpr uint8_t kStateRepeated = 1U << 0;

struct ActionProgressPayload
{
    uint32_t action_sequence;
    uint8_t stage;
    uint8_t reserved[3];
    uint64_t stage_time_us;
    uint32_t feedback_sweep_id;
};

struct ActionProgressRecord
{
    uint32_t action_sequence;
    uint8_t flags;
    uint8_t reserved[3];
    uint32_t can_queued_time_low_us;
    uint32_t can_tx_complete_time_low_us;
    uint32_t post_feedback_time_low_us;
    uint32_t feedback_sweep_id;
};

// Stable codes stored in the diagnostics-v2 reserved tail. This preserves the
// 380-byte payload and protocol-v5 compatibility while making a failed
// pre-window Stream transition diagnosable after the firmware returns HOLD.
enum class CanTransitionFailureCode : uint32_t
{
    None = 0U,
    EpochChanged = 1U,
    AuthorizationLost = 2U,
    TransportOverflow = 3U,
    SafetyTxCompletion = 4U,
    ConfigurationTxCompletion = 5U,
    EnableValidation = 6U,
    MotorDiagnosticsTimeout = 7U,
    MotorMarkersIncomplete = 8U,
    ConfigurationQueue = 9U,
    EnableQueue = 10U,
};

struct CanDiagnosticsPayload
{
    uint16_t format_version;
    uint16_t payload_size;
    uint32_t session_epoch;
    uint8_t motor_marker_mask;
    uint8_t window_flags;
    uint16_t reserved0;
    uint32_t window_reset_count;
    uint64_t window_start_us;
    uint64_t window_duration_us;
    uint32_t target_tx_complete[7];
    uint32_t position_request[7];
    uint32_t position_response[7];
    uint32_t position_timeout[7];
    uint32_t temperature_request[7];
    uint32_t temperature_response[7];
    uint32_t temperature_timeout[7];
    uint8_t motor_tx_drop[7];
    uint8_t motor_rx_error[7];
    uint8_t motor_busoff[7];
    uint8_t reserved_motor[3];
    uint32_t main_can_busoff[2];
    uint32_t main_can_rx_overflow[2];
    uint32_t main_can_rx_high_water[2];
    uint32_t unexpected_response_count;
    uint32_t maintenance_response_count;
    uint32_t query_target_overlap_count;
    uint32_t target_retry_count;
    uint32_t target_retry_exhausted_count;
    uint32_t target_deadline_failure_count;
    uint32_t main_can_tx_abort[2];
    uint32_t main_can_tx_error[2];
    uint32_t main_can_tx_recovery[2];
    uint32_t main_can_completion_overflow[2];
    uint32_t safety_preemption_count;
    uint32_t max_safety_wait_us;
    uint32_t max_fanout_us;
    uint32_t max_rx_dispatch_latency_us;
    uint32_t main_can_rx_frame[2];
    uint32_t main_can_tx_busy[2];
    uint32_t transition_failure_count;
    uint32_t last_transition_failure_code;
    uint32_t last_transition_failure_node_id;
    uint32_t last_transition_failure_detail;
};

// Independent A9 timing evidence. Keeping this separate from the fixed
// diagnostics-v2 payload preserves binary protocol v5 and Raw Session v6.
// Motor timing values use 0.1 us units; main-controller latency values use us.
struct CanTimingProfilePayload
{
    uint16_t format_version;
    uint16_t payload_size;
    uint32_t session_epoch;
    uint32_t window_reset_count;
    uint64_t window_start_us;
    uint64_t window_duration_us;
    uint8_t motor_page_valid_mask[4];
    uint8_t window_flags;
    uint8_t reserved0[3];
    uint32_t position_samples[7];
    uint32_t position_p50_us[7];
    uint32_t position_p99_us[7];
    uint32_t position_p999_us[7];
    uint32_t position_max_us[7];
    uint32_t temperature_samples[7];
    uint32_t temperature_p50_us[7];
    uint32_t temperature_p99_us[7];
    uint32_t temperature_p999_us[7];
    uint32_t temperature_max_us[7];
    uint8_t motor_flags[7];
    uint8_t reserved_motor;
    uint16_t motor_can_samples[7];
    uint16_t motor_can_p999_x10_us[7];
    uint16_t motor_can_max_x10_us[7];
    uint16_t motor_jitter_p999_x10_us[7];
    uint16_t motor_jitter_max_x10_us[7];
    uint16_t motor_control_p999_x10_us[7];
    uint16_t motor_control_max_x10_us[7];
    uint16_t motor_missed_ticks[7];
    uint32_t timing_request[7];
    uint32_t timing_response[7];
    uint32_t timing_timeout[7];
};

struct StatePayload
{
    uint64_t mcu_time_us;
    float position[7];
    float velocity[7];
    uint32_t last_received_sequence;
    uint8_t mode;
    uint8_t validity;
    uint16_t fault_bits;
    uint32_t target_age_ms;
    uint8_t config_sha256[32];
    float following_error[7];
    uint32_t following_error_duration_ms[7];
    uint32_t feedback_age_ms[7];
    uint32_t feedback_loss_count[7];
    uint16_t consecutive_feedback_loss[7];
    uint16_t node_fault_bits[7];
    uint8_t node_validity[7];
    uint8_t can_transport_status;
    uint16_t hold_reason_bits;
    uint16_t telemetry_validity;
    uint64_t feedback_sample_mcu_us[7];
    uint32_t feedback_sweep_id[7];
    uint32_t coherent_sweep_id;
    uint32_t feedback_max_skew_us;
    uint64_t coherent_reference_mcu_us;
    uint8_t state_flags;
    uint8_t action_progress_count;
    uint8_t action_progress_head;
    uint8_t progress_reserved;
    ActionProgressRecord action_progress[kActionProgressReplayCapacity];
};
#pragma pack(pop)

static_assert(sizeof(float) == 4, "protocol requires IEEE-754 binary32 floats");
static_assert(sizeof(PacketHeader) == 24, "PacketHeader layout changed");
static_assert(sizeof(JointTargetPayload) == 60, "JointTargetPayload layout changed");
static_assert(sizeof(TargetKeepalivePayload) == 8, "TargetKeepalivePayload layout changed");
static_assert(sizeof(TimeSyncPayload) == 8, "TimeSyncPayload layout changed");
static_assert(sizeof(TimeSyncAckPayload) == 24, "TimeSyncAckPayload layout changed");
static_assert(sizeof(ActionProgressPayload) == 20, "ActionProgressPayload layout changed");
static_assert(sizeof(ActionProgressRecord) == 24, "ActionProgressRecord layout changed");
static_assert(sizeof(CanDiagnosticsPayload) == kCanDiagnosticsPayloadSize,
              "CanDiagnosticsPayload layout changed");
static_assert(sizeof(CanTimingProfilePayload) == kCanTimingProfilePayloadSize,
              "CanTimingProfilePayload layout changed");
static_assert(sizeof(StatePayload) == 508, "StatePayload layout changed");

constexpr size_t kMaxPayload = kMaxDecodedFrame - sizeof(PacketHeader) - kCrcSize;

struct Packet
{
    PacketHeader header{};
    std::array<uint8_t, kMaxPayload> payload{};
};

uint32_t Crc32c(const uint8_t* data, size_t length, uint32_t initial = 0);

size_t CobsEncode(const uint8_t* input, size_t input_length,
                  uint8_t* output, size_t output_capacity);

DecodeStatus CobsDecode(const uint8_t* input, size_t input_length,
                        uint8_t* output, size_t output_capacity,
                        size_t& output_length);

size_t EncodePacket(const Packet& packet, uint8_t* output, size_t output_capacity);

DecodeStatus DecodePacket(const uint8_t* encoded, size_t encoded_length, Packet& packet);

bool IsKnownMessage(uint8_t raw_type);

class StreamDecoder
{
public:
    // Returns true and writes a packet only when a delimiter completes a valid frame.
    bool Feed(uint8_t byte, Packet& packet);
    size_t dropped_frames() const { return dropped_frames_; }
    DecodeStatus last_error() const { return last_error_; }

private:
    std::array<uint8_t, 600> buffer_{};
    size_t length_ = 0;
    size_t dropped_frames_ = 0;
    DecodeStatus last_error_ = DecodeStatus::Ok;
};

} // namespace dummy::protocol

#endif // DUMMY_BINARY_PROTOCOL_HPP
