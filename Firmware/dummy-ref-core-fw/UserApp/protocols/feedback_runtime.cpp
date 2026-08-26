#include "common_inc.h"
#include "feedback_runtime.hpp"
#include "configurations/robot_config_generated.hpp"
#include "published_double_buffer.hpp"
#include "../../../can_transport_contract.h"

namespace dummy::protocol
{
namespace
{
CanFeedbackMonitor feedback_monitor(
    dummy::generated_config::kCoherentMaxSkewUs);
uint8_t position_response_mask = 0U;
uint8_t temperature_response_mask = 0U;
uint32_t unexpected_position_response_count = 0U;
uint32_t unexpected_temperature_response_count = 0U;
uint8_t runtime_status = 0U;
bool feedback_ready = false;
uint32_t readiness_sweep_id = 0U;
uint8_t consecutive_coherent_sweeps = 0U;
MotorTransportDiagnostics motor_transport_diagnostics{};

struct FeedbackPublishedSnapshot
{
    std::array<NodeFeedbackStatus, kActuatorNodeCount> nodes{};
    CoherentFeedbackStatus coherent{};
    MotorTransportDiagnostics motor_transport{};
};

PublishedDoubleBuffer<FeedbackPublishedSnapshot> feedback_snapshot;
PublishedDoubleBuffer<CanDiagnosticsPayload> can_diagnostics_snapshot;

uint8_t NodeMask(uint8_t node_id)
{
    return node_id >= 1U && node_id <= kActuatorNodeCount
        ? static_cast<uint8_t>(1U << (node_id - 1U)) : 0U;
}
}

void RecordPositionFeedbackRequest(uint8_t node_id, uint32_t sweep_id)
{
    feedback_monitor.OnPositionRequest(node_id, micros(), sweep_id);
}

bool RecordPositionFeedbackResponse(uint8_t node_id, uint32_t received_us)
{
    if (!feedback_monitor.OnPositionResponse(node_id, received_us))
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
    feedback_monitor.OnPositionTimeout(node_id);
}

void RecordTemperatureFeedbackRequest(uint8_t node_id)
{
    feedback_monitor.OnTemperatureRequest(node_id, micros());
}

void RecordTemperatureFeedbackResponse(uint8_t node_id, float temperature_c,
                                       uint32_t received_us)
{
    if (!feedback_monitor.OnTemperatureResponse(
            node_id, received_us, temperature_c))
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
    feedback_monitor.OnTemperatureTimeout(node_id);
}

FeedbackResponseEvents ConsumeFeedbackResponseEvents()
{
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
    return events;
}

void CancelPendingFeedbackRequests()
{
    feedback_monitor.CancelPendingRequests();
    position_response_mask = 0U;
    temperature_response_mask = 0U;
}

void PublishFeedbackSnapshot(uint32_t now_us)
{
    FeedbackPublishedSnapshot snapshot{};
    snapshot.nodes = feedback_monitor.Snapshot(now_us);
    snapshot.coherent = feedback_monitor.CoherentSnapshot();
    snapshot.motor_transport = motor_transport_diagnostics;
    (void) feedback_snapshot.TryPublish(snapshot);
}

std::array<NodeFeedbackStatus, kActuatorNodeCount> ReadCanFeedbackStatus(
    uint32_t now_us)
{
    (void) now_us;
    return feedback_snapshot.Read().nodes;
}

CoherentFeedbackStatus ReadCoherentFeedbackStatus()
{
    return feedback_snapshot.Read().coherent;
}

MotorTransportDiagnostics ReadMotorTransportDiagnostics()
{
    return feedback_snapshot.Read().motor_transport;
}

void ResetMotorTransportDiagnostics()
{
    motor_transport_diagnostics = {};
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
    const auto coherent = feedback_snapshot.Read().coherent;
    taskENTER_CRITICAL();
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
    (void) can_diagnostics_snapshot.TryPublish(diagnostics);
}

CanDiagnosticsPayload ReadCanDiagnostics()
{
    return can_diagnostics_snapshot.Read();
}

} // namespace dummy::protocol
