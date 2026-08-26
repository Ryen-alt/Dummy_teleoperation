#include "common_inc.h"
#include "feedback_runtime.hpp"
#include "configurations/robot_config_generated.hpp"
#include "../../../can_transport_contract.h"

namespace dummy::protocol
{
namespace
{
CanFeedbackMonitor feedback_monitor(
    dummy::generated_config::kCoherentMaxSkewUs);
volatile uint8_t position_response_mask = 0U;
volatile uint8_t temperature_response_mask = 0U;
volatile uint32_t unexpected_position_response_count = 0U;
volatile uint32_t unexpected_temperature_response_count = 0U;
uint8_t runtime_status = 0U;
bool feedback_ready = false;
uint32_t readiness_sweep_id = 0U;
uint8_t consecutive_coherent_sweeps = 0U;
CanDiagnosticsPayload can_diagnostics{};
MotorTransportDiagnostics motor_transport_diagnostics{};

uint8_t NodeMask(uint8_t node_id)
{
    return node_id >= 1U && node_id <= kActuatorNodeCount
        ? static_cast<uint8_t>(1U << (node_id - 1U)) : 0U;
}
}

void RecordPositionFeedbackRequest(uint8_t node_id, uint32_t sweep_id)
{
    taskENTER_CRITICAL();
    feedback_monitor.OnPositionRequest(node_id, micros(), sweep_id);
    taskEXIT_CRITICAL();
}

bool RecordPositionFeedbackResponse(uint8_t node_id)
{
    // Called from the CAN RX ISR. Normal task code masks interrupts while it
    // mutates or snapshots the monitor.
    if (!feedback_monitor.OnPositionResponse(node_id, micros()))
    {
        if (unexpected_position_response_count != UINT32_MAX)
            ++unexpected_position_response_count;
        return false;
    }
    position_response_mask = static_cast<uint8_t>(
        position_response_mask | NodeMask(node_id));
    return true;
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
    if (!feedback_monitor.OnTemperatureResponse(node_id, micros(), temperature_c))
    {
        if (unexpected_temperature_response_count != UINT32_MAX)
            ++unexpected_temperature_response_count;
        return;
    }
    temperature_response_mask = static_cast<uint8_t>(
        temperature_response_mask | NodeMask(node_id));
}

void RecordMotorTransportDiagnostics(uint8_t node_id, const uint8_t* data,
                                     uint32_t length)
{
    if (node_id < 1U || node_id > kActuatorNodeCount || data == nullptr ||
        length < 8U ||
        data[DUMMY_MOTOR_DIAGNOSTICS_FORMAT_OFFSET] !=
            DUMMY_MOTOR_DIAGNOSTICS_FORMAT_V2)
        return;
    const size_t index = node_id - 1U;
    motor_transport_diagnostics.valid_mask = static_cast<uint8_t>(
        motor_transport_diagnostics.valid_mask | NodeMask(node_id));
    motor_transport_diagnostics.tx_drop[index] =
        data[DUMMY_MOTOR_TX_DROP_OFFSET];
    motor_transport_diagnostics.rx_error[index] =
        data[DUMMY_MOTOR_RX_ERROR_OFFSET];
    motor_transport_diagnostics.busoff[index] =
        data[DUMMY_MOTOR_BUSOFF_OFFSET];
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
        unexpected_position_response_count,
        unexpected_temperature_response_count,
    };
    position_response_mask = 0U;
    temperature_response_mask = 0U;
    unexpected_position_response_count = 0U;
    unexpected_temperature_response_count = 0U;
    taskEXIT_CRITICAL();
    return events;
}

void CancelPendingFeedbackRequests()
{
    taskENTER_CRITICAL();
    feedback_monitor.CancelPendingRequests();
    position_response_mask = 0U;
    temperature_response_mask = 0U;
    taskEXIT_CRITICAL();
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

MotorTransportDiagnostics ReadMotorTransportDiagnostics()
{
    taskENTER_CRITICAL();
    const MotorTransportDiagnostics snapshot = motor_transport_diagnostics;
    taskEXIT_CRITICAL();
    return snapshot;
}

void ResetMotorTransportDiagnostics()
{
    taskENTER_CRITICAL();
    motor_transport_diagnostics = {};
    taskEXIT_CRITICAL();
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
