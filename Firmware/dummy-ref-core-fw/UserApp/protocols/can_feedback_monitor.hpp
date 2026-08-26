#ifndef DUMMY_CAN_FEEDBACK_MONITOR_HPP
#define DUMMY_CAN_FEEDBACK_MONITOR_HPP

#include <array>
#include <cstddef>
#include <cstdint>

namespace dummy::protocol
{

constexpr size_t kActuatorNodeCount = 7;
constexpr uint32_t kFeedbackAgeUnknown = 0xFFFFFFFFU;

struct NodeFeedbackStatus
{
    uint32_t position_age_ms = kFeedbackAgeUnknown;
    uint32_t temperature_age_ms = kFeedbackAgeUnknown;
    uint32_t total_position_losses = 0;
    uint16_t consecutive_position_losses = 0;
    float temperature_c = 0.0F;
    uint32_t position_sample_us = 0;
    uint32_t position_sweep_id = 0;
    bool position_seen = false;
    bool temperature_seen = false;
};

struct CoherentFeedbackStatus
{
    std::array<uint32_t, kActuatorNodeCount> position_sample_us{};
    std::array<uint32_t, kActuatorNodeCount> position_sweep_id{};
    uint32_t sweep_id = 0;
    uint32_t max_skew_us = 0;
    bool valid = false;
};

// Tracks the request/response protocol actually implemented by the CtrlStep
// motor boards. Node 0 is a broadcast request and therefore arms nodes 1..7.
// Unsigned 32-bit microsecond subtraction deliberately remains valid across the
// hardware timer wrap for the short safety intervals used here.
class CanFeedbackMonitor
{
public:
    explicit CanFeedbackMonitor(uint32_t coherent_max_skew_us = 30000U)
        : coherent_max_skew_us_(coherent_max_skew_us)
    {}
    void OnPositionRequest(uint8_t node_id, uint32_t now_us,
                           uint32_t sweep_id = 0U);
    bool OnPositionResponse(uint8_t node_id, uint32_t now_us);
    void OnPositionTimeout(uint8_t node_id);
    void OnTemperatureRequest(uint8_t node_id, uint32_t now_us);
    bool OnTemperatureResponse(uint8_t node_id, uint32_t now_us,
                               float temperature_c);
    void OnTemperatureTimeout(uint8_t node_id);

    std::array<NodeFeedbackStatus, kActuatorNodeCount> Snapshot(uint32_t now_us) const;
    CoherentFeedbackStatus CoherentSnapshot() const;
    void CancelPendingRequests();
    void Reset();

private:
    struct NodeState
    {
        uint32_t last_position_us = 0;
        uint32_t position_sweep_id = 0;
        uint32_t pending_sweep_id = 0;
        uint32_t last_temperature_us = 0;
        uint32_t total_position_losses = 0;
        uint16_t consecutive_position_losses = 0;
        float temperature_c = 0.0F;
        bool position_pending = false;
        bool temperature_pending = false;
        bool position_seen = false;
        bool temperature_seen = false;
    };

    template<typename Callback>
    void ForRequestedNodes(uint8_t node_id, Callback callback);

    std::array<NodeState, kActuatorNodeCount> nodes_{};
    uint32_t current_sweep_id_ = 1U;
    uint8_t current_sweep_request_mask_ = 0U;
    CoherentFeedbackStatus coherent_{};
    uint32_t coherent_max_skew_us_ = 30000U;
};

} // namespace dummy::protocol

#endif // DUMMY_CAN_FEEDBACK_MONITOR_HPP
