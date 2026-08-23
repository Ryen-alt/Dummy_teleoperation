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
    bool position_seen = false;
    bool temperature_seen = false;
};

// Tracks the request/response protocol actually implemented by the CtrlStep
// motor boards. Node 0 is a broadcast request and therefore arms nodes 1..7.
// Unsigned 32-bit microsecond subtraction deliberately remains valid across the
// hardware timer wrap for the short safety intervals used here.
class CanFeedbackMonitor
{
public:
    void OnPositionRequest(uint8_t node_id, uint32_t now_us);
    void OnPositionResponse(uint8_t node_id, uint32_t now_us);
    void OnTemperatureRequest(uint8_t node_id, uint32_t now_us);
    void OnTemperatureResponse(uint8_t node_id, uint32_t now_us, float temperature_c);

    std::array<NodeFeedbackStatus, kActuatorNodeCount> Snapshot(uint32_t now_us) const;
    void Reset();

private:
    struct NodeState
    {
        uint32_t last_position_us = 0;
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
};

} // namespace dummy::protocol

#endif // DUMMY_CAN_FEEDBACK_MONITOR_HPP
