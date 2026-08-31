#include "common_inc.h"
#include "feedback_runtime.hpp"
#include "configurations/robot_config_generated.hpp"
#include "published_double_buffer.hpp"
#include "can_timing_profiler.hpp"
#include "../../../can_transport_contract.h"

#include <algorithm>

namespace dummy::protocol
{
namespace
{
CanFeedbackMonitor feedback_monitor(
    dummy::generated_config::kCoherentMaxSkewUs);
uint8_t position_response_mask = 0U;
uint8_t temperature_response_mask = 0U;
uint8_t timing_profile_response_mask = 0U;
std::array<uint8_t, kActuatorNodeCount> timing_profile_response_page{};
std::array<std::array<uint8_t, 8U>, kActuatorNodeCount>
    timing_profile_response_data{};
uint32_t unexpected_position_response_count = 0U;
uint32_t unexpected_temperature_response_count = 0U;
uint32_t unexpected_timing_profile_response_count = 0U;
uint8_t runtime_status = 0U;
bool feedback_ready = false;
uint32_t readiness_sweep_id = 0U;
uint8_t consecutive_coherent_sweeps = 0U;
MotorTransportDiagnostics motor_transport_diagnostics{};
CanTimingProfiler can_timing_profiler{};

struct FeedbackPublishedSnapshot
{
    std::array<NodeFeedbackStatus, kActuatorNodeCount> nodes{};
    CoherentFeedbackStatus coherent{};
    MotorTransportDiagnostics motor_transport{};
};

PublishedDoubleBuffer<FeedbackPublishedSnapshot> feedback_snapshot;
PublishedDoubleBuffer<CanDiagnosticsPayload> can_diagnostics_snapshot;
PublishedDoubleBuffer<CanTimingProfilePayload> can_timing_profile_snapshot;

uint8_t NodeMask(uint8_t node_id)
{
    return node_id >= 1U && node_id <= kActuatorNodeCount
        ? static_cast<uint8_t>(1U << (node_id - 1U)) : 0U;
}
}

void RecordPositionFeedbackRequest(uint8_t node_id, uint32_t sweep_id)
{
    const uint32_t now_us = micros();
    feedback_monitor.OnPositionRequest(node_id, now_us, sweep_id);
}

void RecordPositionTimingStart(uint8_t node_id, uint32_t completed_us)
{
    can_timing_profiler.RecordPositionRequest(node_id, completed_us);
}

bool RecordPositionFeedbackResponse(uint8_t node_id, uint32_t received_us)
{
    can_timing_profiler.RecordPositionResponse(node_id, received_us);
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
    can_timing_profiler.RecordPositionTimeout(node_id);
}

void RecordTemperatureFeedbackRequest(uint8_t node_id)
{
    const uint32_t now_us = micros();
    feedback_monitor.OnTemperatureRequest(node_id, now_us);
}

void RecordTemperatureTimingStart(uint8_t node_id, uint32_t completed_us)
{
    can_timing_profiler.RecordTemperatureRequest(node_id, completed_us);
}

void RecordTemperatureFeedbackResponse(uint8_t node_id, float temperature_c,
                                       uint32_t received_us)
{
    can_timing_profiler.RecordTemperatureResponse(node_id, received_us);
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
    can_timing_profiler.RecordTemperatureTimeout(node_id);
}

bool RecordMotorTimingProfile(uint8_t node_id, const uint8_t* data,
                              uint32_t length)
{
    if (node_id < 1U || node_id > kActuatorNodeCount || data == nullptr ||
        length < 8U || data[0] != DUMMY_MOTOR_TIMING_FORMAT_V1 ||
        data[1] >= DUMMY_MOTOR_TIMING_PAGE_COUNT)
    {
        if (unexpected_timing_profile_response_count != UINT32_MAX)
            ++unexpected_timing_profile_response_count;
        return false;
    }
    const size_t index = node_id - 1U;
    std::copy_n(data, 8U, timing_profile_response_data[index].begin());
    timing_profile_response_mask = static_cast<uint8_t>(
        timing_profile_response_mask | NodeMask(node_id));
    timing_profile_response_page[index] = data[1];
    return true;
}

bool AcceptMotorTimingProfile(uint8_t node_id, uint8_t page)
{
    if (node_id < 1U || node_id > kActuatorNodeCount ||
        page >= DUMMY_MOTOR_TIMING_PAGE_COUNT)
        return false;
    const auto& data = timing_profile_response_data[node_id - 1U];
    if (data[0] != DUMMY_MOTOR_TIMING_FORMAT_V1 || data[1] != page)
        return false;
    return can_timing_profiler.RecordMotorPage(
        node_id, data.data(), static_cast<uint32_t>(data.size()));
}

FeedbackResponseEvents ConsumeFeedbackResponseEvents()
{
    const FeedbackResponseEvents events{
        position_response_mask,
        temperature_response_mask,
        timing_profile_response_mask,
        timing_profile_response_page,
        unexpected_position_response_count,
        unexpected_temperature_response_count,
        unexpected_timing_profile_response_count,
    };
    position_response_mask = 0U;
    temperature_response_mask = 0U;
    timing_profile_response_mask = 0U;
    unexpected_position_response_count = 0U;
    unexpected_temperature_response_count = 0U;
    unexpected_timing_profile_response_count = 0U;
    return events;
}

void CancelPendingFeedbackRequests()
{
    feedback_monitor.CancelPendingRequests();
    position_response_mask = 0U;
    temperature_response_mask = 0U;
    timing_profile_response_mask = 0U;
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

void ResetCanTimingProfile(uint32_t session_epoch, uint64_t start_us)
{
    can_timing_profiler.Reset(session_epoch, start_us);
}

void SetCanTimingProfileActive(bool active)
{
    can_timing_profiler.SetActive(active);
}

void SetCanTimingProfileEpochStable(bool stable)
{
    can_timing_profiler.SetEpochStable(stable);
}

void PublishCanTimingProfile(uint64_t now_us,
                             const CanDispatchDiagnostics& scheduler)
{
    (void) can_timing_profile_snapshot.TryPublish(
        can_timing_profiler.MakePayload(now_us, scheduler));
}

CanTimingProfilePayload ReadCanTimingProfile()
{
    return can_timing_profile_snapshot.Read();
}

} // namespace dummy::protocol
