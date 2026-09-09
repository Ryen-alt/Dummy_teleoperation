// Include the production bridges so tests can drive the real parser and pin
// the real snapshot without adding a firmware injection command or test ABI.
#include "../../UserApp/protocols/feedback_runtime.cpp"
#include "../../Bsp/communication/interface_usb.cpp"
#include "../../UserApp/protocols/binary_state_bridge.cpp"
#include "scheduled_actuator_request.hpp"

#include <cassert>
#include <iostream>

DummyRobot robot;
unsigned test_critical_depth = 0;
uint32_t test_now_us = 1000;
uint32_t micros() { return test_now_us; }
void ASCII_protocol_parse_stream(const uint8_t*, size_t, StreamSink&) {}

using namespace dummy::protocol;

namespace {
template<typename Payload>
void Send(MessageType type, uint32_t epoch, uint32_t sequence, const Payload& payload)
{
    Packet packet{};
    packet.header.magic = kMagic;
    packet.header.version = kProtocolVersion;
    packet.header.message_type = static_cast<uint8_t>(type);
    packet.header.session_id = epoch;
    packet.header.sequence = sequence;
    packet.header.payload_length = sizeof(payload);
    std::memcpy(packet.payload.data(), &payload, sizeof(payload));
    std::array<uint8_t, kMaxDecodedFrame + 16> wire{};
    const size_t length = EncodePacket(packet, wire.data(), wire.size());
    assert(length);
    ProcessBinaryBytes(wire.data(), length, test_now_us);
}

void Hello(uint32_t epoch)
{
    HelloPayload hello{};
    std::copy(dummy::generated_config::kConfigSha256.begin(),
              dummy::generated_config::kConfigSha256.end(), hello.config_sha256);
    Send(MessageType::Hello, epoch, 1, hello);
    assert(binary_session.hello_valid());
}

void NewSession(uint32_t epoch)
{
    binary_session = ControlSession(MakeBinarySessionConfig(), "test-fw");
    ResetBinaryActionProgress();
    Hello(epoch);
    binary_session.SetControlReady(true);
    Send(MessageType::AcquireControl, epoch, 2, AcquireControlPayload{500});
    Send(MessageType::SetMode, epoch, 3, SetModePayload{static_cast<uint8_t>(ControlMode::Teleop)});
    assert(binary_session.lease_active());
    assert(binary_session.mode() == ControlMode::Teleop);
}

void Target(uint32_t epoch, uint32_t sequence)
{
    JointTargetPayload target{};
    std::copy(dummy::generated_config::kInitialPoseRad.begin(),
              dummy::generated_config::kInitialPoseRad.end(), target.target);
    std::copy(dummy::generated_config::kMaxVelocityRadS.begin(),
              dummy::generated_config::kMaxVelocityRadS.end(), target.max_velocity);
    target.target[6] = 0.5F;
    target.valid_for_ms = 200;
    target.control_tick_id = sequence;
    Send(MessageType::SetJointTarget, epoch, sequence, target);
    assert(binary_session.active_target().valid);
    assert(TryStartBinaryTargetDispatch(sequence, binary_session.session_id()));
}

void FeedbackRound(uint32_t now, uint32_t sweep)
{
    test_now_us = now;
    for (uint8_t node = 1; node <= 7; ++node) {
        RecordPositionFeedbackRequest(node, sweep);
        assert(RecordPositionFeedbackResponse(node, now));
        SealJointPositionSample(node, now);
    }
    PublishFeedbackSnapshot(now);
    LatchCoherentRobotMeasurement();
}

FeedbackSafetyOutput SafetyTick(FeedbackSafetySupervisor& supervisor, uint64_t now)
{
    const auto input = ReadFeedbackSafetyInput(now, binary_session.lease_active(),
                                             false, {}, {});
    const auto safety = supervisor.Update(input);
    PublishCanFeedbackReady(safety.arm_position_valid && safety.gripper_position_valid);
    ApplyBinarySafetyOutcome(safety);
    return safety;
}

void TestPublicationFailureAndPausedDispatcher(bool fail_publication, uint64_t base)
{
    test_now_us = static_cast<uint32_t>(base);
    NewSession(fail_publication ? 101 : 102);
    FeedbackSafetySupervisor supervisor(FeedbackSafetyConfig{});
    FeedbackRound(static_cast<uint32_t>(base), 100);
    assert(SafetyTick(supervisor, base).arm_position_valid);
    auto exercise = [&] {
        // After one successful swap the pinned old slot becomes the next
        // write slot. Every attempted publication then fails, while the real
        // monitor continues receiving fresh frames.
        FeedbackRound(static_cast<uint32_t>(base + 1000), 101);
        const auto baseline = ReadFeedbackRuntimeProgress();
        for (uint32_t elapsed = 5000; elapsed <= 105000; elapsed += 5000) {
            if (fail_publication)
                FeedbackRound(static_cast<uint32_t>(base + elapsed), 101 + elapsed);
            const auto safety = SafetyTick(supervisor, base + elapsed);
            if (elapsed < 100000)
                assert(!(safety.hold_reason_bits & kHoldReasonDispatcherStalled));
        }
        const auto safety = ReadBinarySafetyTelemetry();
        assert(safety.hold_reason_bits & kHoldReasonDispatcherStalled);
        assert(safety.hold_reason_bits & kHoldReasonFeedbackStale);
        assert(binary_session.mode() == ControlMode::Hold);
        assert(!binary_session.active_target().valid);
        assert(!ReadCanFeedbackReady());
        assert(!(ReadRobotStateForBinaryProtocol(base + 105000, safety).validity & kStatePositionValid));
        const auto progress = ReadFeedbackRuntimeProgress();
        assert(progress.last_publish_us == baseline.last_publish_us);
        assert(progress.publish_failure_count > baseline.publish_failure_count || !fail_publication);
        // The lease remains owned in HOLD, so independent safety must still
        // escalate persistent lost feedback, without a dispatcher wake.
        const auto fault = SafetyTick(supervisor, base + 505000);
        assert(fault.fault_bits & kFaultFeedbackLost);
        assert(binary_session.mode() == ControlMode::Fault);
        assert(SelectActuatorPublishDecision({true, false, false, true, true})
               == ActuatorPublishDecision::Fault);
    };
    if (fail_publication)
        feedback_snapshot.WithRead([&](const auto&) { exercise(); });
    else
        exercise();
    // Recovery republishes feedback, but neither the safety latch nor an old
    // target can reactivate motion automatically.
    FeedbackRound(static_cast<uint32_t>(base + 510000), 900000);
    assert(SafetyTick(supervisor, base + 510000).arm_position_valid);
    assert(binary_session.mode() == ControlMode::Fault);
    assert(!binary_session.active_target().valid);
}

void TestFailureIsTerminalInActualActionLedger()
{
    test_now_us = 1000;
    NewSession(201);
    Target(201, 4);
    RecordBinaryTargetCanQueuedExact(4, 2000, 1, binary_session.session_id());
    RecordBinaryTargetFailed(4, 3000, binary_session.session_id());
    const size_t event_count = binary_progress_event_write;
    RecordBinaryTargetCanTxCompleteExact(4, 4000, 2000, binary_session.session_id());
    RecordBinaryCoherentSweep(2, 5000, 4500, binary_session.session_id());
    assert(binary_progress_event_write == event_count);
    assert(ProgressRecord(4).flags == (kActionProgressCanQueuedExact | kActionProgressFailed));
    // Even re-delivering the queued notification cannot restart a terminal
    // action, and a later safety preemption cannot overwrite its first cause.
    RecordBinaryTargetCanQueuedExact(4, 6000, 2, 201);
    RecordBinaryTargetPreemptedBySafety(4, 7000, 201);
    assert(binary_progress_event_write == event_count);

    Target(201, 5);
    RecordBinaryTargetPreemptedBySafety(5, 8000, 201);
    const size_t preempted_events = binary_progress_event_write;
    RecordBinaryTargetCanQueuedExact(5, 9000, 3, 201);
    RecordBinaryTargetCanTxCompleteExact(5, 10000, 1000, 201);
    RecordBinaryTargetSuperseded(5, 11000, 201);
    assert(binary_progress_event_write == preempted_events);
    assert(ProgressRecord(5).flags == kActionProgressPreemptedBySafety);
}

void TestSessionSwitchClearsLedgerAndRejectsOldCompletions()
{
    test_now_us = 1000;
    NewSession(301);
    Target(301, 4);
    TargetCompletionTracker tracker;
    const TargetFanoutKey old_key{301, 4, 1};
    assert(tracker.Begin(old_key, 1000));
    RecordBinaryTargetCanQueuedExact(4, 1100, 1, binary_session.session_id());
    RequestBinaryRuntimeHold();
    binary_session.Tick(600000); // release expired ownership before new HELLO
    test_now_us = 600000;
    Hello(302); // actual parser resets the replay ledger on epoch transition
    assert(binary_progress_count == 0);
    assert(binary_progress_event_read == binary_progress_event_write);
    assert(!ReadBinaryControlSnapshot(600000).target.valid);
    binary_session.SetControlReady(true);
    Send(MessageType::AcquireControl, 302, 2, AcquireControlPayload{500});
    Send(MessageType::SetMode, 302, 3, SetModePayload{static_cast<uint8_t>(ControlMode::Teleop)});
    Target(302, 4); // deliberately reuse the same action sequence
    const ScheduledActuatorRequest stale_mailbox{ScheduledActuatorMode::Stream, {}, 4, 301};
    assert(!stale_mailbox.HasTargetFor(binary_session.session_id()));
    auto fresh_mailbox = stale_mailbox;
    fresh_mailbox.session_epoch = 302;
    assert(fresh_mailbox.HasTargetFor(binary_session.session_id()));
    tracker.Cancel();
    const TargetFanoutKey new_key{302, 4, 2};
    assert(tracker.Begin(new_key, 600000));
    RecordBinaryTargetCanQueuedExact(4, 600100, 10, binary_session.session_id());
    // Simulate the USB task switching epochs after the CAN task read its
    // control snapshot, before the task commits an old completion/failure.
    const size_t before_stale = binary_progress_event_write;
    RecordBinaryTargetCanTxCompleteExact(4, 600200, 100, 301);
    RecordBinaryTargetFailed(4, 600200, 301);
    RecordBinaryTargetPreemptedBySafety(4, 600200, 301);
    RecordBinaryTargetCanQueuedExact(4, 600200, 10, 301);
    assert(!TryStartBinaryTargetDispatch(4, 301));
    assert(binary_progress_event_write == before_stale);
    for (uint8_t node = 1; node <= 7; ++node)
        assert(tracker.RecordCompletion(old_key, node, true, 600200) == TargetCompletionResult::Ignored);
    assert(!(ProgressRecord(4).flags & kActionProgressCanTxCompleteExact));
    for (uint8_t node = 1; node <= 7; ++node) {
        const auto result = tracker.RecordCompletion(new_key, node, true, 600300 + node);
        if (result == TargetCompletionResult::CompleteExact)
            RecordBinaryTargetCanTxCompleteExact(4, 600307, tracker.last_fanout_us(), binary_session.session_id());
    }
    RecordBinaryCoherentSweep(11, 601000, 600500, binary_session.session_id());
    assert(ProgressRecord(4).flags & kActionProgressPostCommandFeedback);
    assert(binary_progress_count == 1);
    const size_t events = binary_progress_event_write;
    RecordBinaryTargetCanTxCompleteExact(4, 602000, 2000, binary_session.session_id());
    RecordBinaryCoherentSweep(12, 603000, 602500, binary_session.session_id());
    assert(binary_progress_event_write == events);
}
}

int main()
{
    TestPublicationFailureAndPausedDispatcher(true, 1000000);
    TestPublicationFailureAndPausedDispatcher(false, 0xFFFF0000ULL);
    TestSessionSwitchClearsLedgerAndRejectsOldCompletions();
    TestFailureIsTerminalInActualActionLedger();
    PublishFeedbackSnapshot(0U); // valid publish exactly at uint32 clock wrap
    const auto wrapped = ReadFeedbackSafetyInput(100000U, true, false, {}, {});
    assert(wrapped.dispatcher_published);
    assert(wrapped.dispatcher_progress_age_ms == 100U);
    assert(test_critical_depth == 0);
    std::cout << "actual runtime/USB/safety integration tests passed\n";
}
