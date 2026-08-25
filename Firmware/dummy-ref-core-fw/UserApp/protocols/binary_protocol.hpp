#ifndef DUMMY_BINARY_PROTOCOL_HPP
#define DUMMY_BINARY_PROTOCOL_HPP

#include <array>
#include <cstddef>
#include <cstdint>

namespace dummy::protocol
{

constexpr uint16_t kMagic = 0x4459;
constexpr uint8_t kProtocolVersion = 4;
constexpr size_t kMaxDecodedFrame = 512;
constexpr size_t kCrcSize = 4;
constexpr size_t kActionProgressReplayCapacity = 6;
constexpr uint32_t kCapabilityMultiChannelSequence = 1U << 0U;
constexpr uint32_t kCapabilityTargetKeepalive = 1U << 1U;

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
    HelloAck = 0x81,
    State = 0x82,
    Ack = 0x83,
    Nack = 0x84,
    Fault = 0x85,
    Event = 0x86,
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
};

struct TargetKeepalivePayload
{
    uint32_t action_sequence;
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
    PostCommandFeedback = 2,
    Superseded = 3,
};

constexpr uint8_t kActionProgressCanQueuedExact = 1U << 0;
constexpr uint8_t kActionProgressPostCommandFeedback = 1U << 1;
constexpr uint8_t kActionProgressSuperseded = 1U << 2;
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
    uint32_t post_feedback_time_low_us;
    uint32_t feedback_sweep_id;
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
static_assert(sizeof(JointTargetPayload) == 56, "JointTargetPayload layout changed");
static_assert(sizeof(TargetKeepalivePayload) == 4, "TargetKeepalivePayload layout changed");
static_assert(sizeof(ActionProgressPayload) == 20, "ActionProgressPayload layout changed");
static_assert(sizeof(ActionProgressRecord) == 20, "ActionProgressRecord layout changed");
static_assert(sizeof(StatePayload) == 484, "StatePayload layout changed");

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
