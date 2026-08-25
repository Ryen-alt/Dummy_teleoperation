#include "common_inc.h"
#include "feedback_runtime.hpp"
#include "configurations/robot_config_generated.hpp"

namespace dummy::protocol
{
namespace
{
CanFeedbackMonitor feedback_monitor(
    dummy::generated_config::kCoherentMaxSkewUs);
volatile uint8_t position_response_mask = 0U;
volatile uint8_t temperature_response_mask = 0U;
uint8_t runtime_status = 0U;
bool feedback_ready = false;
uint32_t readiness_sweep_id = 0U;
uint8_t consecutive_coherent_sweeps = 0U;
CanDiagnosticsPayload can_diagnostics{};

uint8_t NodeMask(uint8_t node_id)
{
    return node_id >= 1U && node_id <= kActuatorNodeCount
        ? static_cast<uint8_t>(1U << (node_id - 1U)) : 0U;
}
}

void RecordPositionFeedbackRequest(uint8_t node_id)
{
    taskENTER_CRITICAL();
    feedback_monitor.OnPositionRequest(node_id, micros());
    taskEXIT_CRITICAL();
}

void RecordPositionFeedbackResponse(uint8_t node_id)
{
    // Called from the CAN RX ISR. Normal task code masks interrupts while it
    // mutates or snapshots the monitor.
    feedback_monitor.OnPositionResponse(node_id, micros());
    position_response_mask = static_cast<uint8_t>(
        position_response_mask | NodeMask(node_id));
}

void RecordPositionFeedbackTimeout(uint8_t node_id)
{
    taskENTER_CRITICAL();
    feedback_monitor.OnPositionTimeout(node_id);
    taskEXIT_CRITICAL();
}

void RecordTemperatureFeedbackRequest(uint8_t node_id)
{
    taskENTER_CRITICAL();
    feedback_monitor.OnTemperatureRequest(node_id, micros());
    taskEXIT_CRITICAL();
}

void RecordTemperatureFeedbackResponse(uint8_t node_id, float temperature_c)
{
    feedback_monitor.OnTemperatureResponse(node_id, micros(), temperature_c);
    temperature_response_mask = static_cast<uint8_t>(
        temperature_response_mask | NodeMask(node_id));
}

void RecordTemperatureFeedbackTimeout(uint8_t node_id)
{
    taskENTER_CRITICAL();
    feedback_monitor.OnTemperatureTimeout(node_id);
    taskEXIT_CRITICAL();
}

FeedbackResponseEvents ConsumeFeedbackResponseEvents()
{
    taskENTER_CRITICAL();
    const FeedbackResponseEvents events{
        position_response_mask,
        temperature_response_mask,
    };
    position_response_mask = 0U;
    temperature_response_mask = 0U;
    taskEXIT_CRITICAL();
    return events;
}

std::array<NodeFeedbackStatus, kActuatorNodeCount> ReadCanFeedbackStatus(uint32_t now_us)
{
    taskENTER_CRITICAL();
    const auto snapshot = feedback_monitor.Snapshot(now_us);
    taskEXIT_CRITICAL();
    return snapshot;
}

CoherentFeedbackStatus ReadCoherentFeedbackStatus()
{
    taskENTER_CRITICAL();
    const auto snapshot = feedback_monitor.CoherentSnapshot();
    taskEXIT_CRITICAL();
    return snapshot;
}

void PublishCanRuntimeStatus(uint8_t status)
{
    taskENTER_CRITICAL();
    runtime_status = status;
    taskEXIT_CRITICAL();
}

uint8_t ReadCanRuntimeStatus()
{
    taskENTER_CRITICAL();
    const uint8_t status = runtime_status;
    taskEXIT_CRITICAL();
    return status;
}

void PublishCanFeedbackReady(bool ready)
{
    taskENTER_CRITICAL();
    const auto coherent = feedback_monitor.CoherentSnapshot();
    if (!ready || !coherent.valid)
    {
        feedback_ready = false;
        consecutive_coherent_sweeps = 0U;
        readiness_sweep_id = coherent.sweep_id;
    }
    else if (coherent.sweep_id != 0U &&
             coherent.sweep_id != readiness_sweep_id)
    {
        readiness_sweep_id = coherent.sweep_id;
        if (consecutive_coherent_sweeps < 3U)
            ++consecutive_coherent_sweeps;
        feedback_ready = consecutive_coherent_sweeps >= 3U;
    }
    taskEXIT_CRITICAL();
}

bool ReadCanFeedbackReady()
{
    taskENTER_CRITICAL();
    const bool ready = feedback_ready;
    taskEXIT_CRITICAL();
    return ready;
}

void PublishCanDiagnostics(const CanDiagnosticsPayload& diagnostics)
{
    taskENTER_CRITICAL();
    can_diagnostics = diagnostics;
    taskEXIT_CRITICAL();
}

CanDiagnosticsPayload ReadCanDiagnostics()
{
    taskENTER_CRITICAL();
    const CanDiagnosticsPayload snapshot = can_diagnostics;
    taskEXIT_CRITICAL();
    return snapshot;
}

} // namespace dummy::protocol
