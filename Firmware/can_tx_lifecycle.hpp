#ifndef DUMMY_CAN_TX_LIFECYCLE_HPP
#define DUMMY_CAN_TX_LIFECYCLE_HPP

#include <cstdint>

// Single-flight CAN TX channel lifecycle shared verbatim between the bxCAN
// transport (Bsp/communication/interface_can.cpp) and the host fault-injection
// regression (tests/host_protocol). The transport applies the returned
// transition; the tests drive this same table (doc 07 R01).
//
// Invariants:
//   - exactly one business terminal (Complete/Aborted/Error) per enqueue;
//   - the TX token is released exactly once per acquisition: never while the
//     hardware mailbox can still hold the old frame (RecoveryRequired), and
//     never fabricated into Complete without hardware evidence.
namespace dummy::can_tx
{

enum class State : uint8_t
{
    Idle,
    InFlight,
    AbortRequested,
    RecoveryRequired,
    Blocked,
};

enum class Event : uint8_t
{
    EnqueueSucceeded,      // mailbox admission succeeded
    EnqueueFailed,         // HAL_CAN_AddTxMessage returned an error
    CompleteCallback,      // complete IRQ for the active mailbox
    AbortCallback,         // abort IRQ for the active mailbox
    StaleCallback,         // IRQ while Idle, or for another mailbox
    AbortAccepted,         // HAL_CAN_AbortTxRequest returned HAL_OK
    AbortRejected,         // HAL_CAN_AbortTxRequest returned an error
    MailboxEndedComplete,  // recovery poll: mailbox empty and TXOK set
    MailboxEndedAborted,   // recovery poll: mailbox empty, TXOK clear
    MailboxStillPending,   // recovery poll: mailbox not empty
    RecoveryExhausted,     // bounded recovery attempts used up
    ChannelReset,          // controlled peripheral reset completed
};

enum class Terminal : uint8_t
{
    None,      // no business outcome on this transition
    Complete,  // evidence-backed successful transmission
    Aborted,   // abort finished, or mailbox ended without TXOK
    Error,     // admission failed, or recovery required
};

struct Transition
{
    State next = State::Idle;
    Terminal terminal = Terminal::None;
    bool release_token = false;
    bool count_stale = false;
    bool count_recovery = false;
};

inline Transition Advance(State state, Event event)
{
    Transition transition{};
    switch (state)
    {
    case State::Idle:
        transition.next = State::Idle;
        switch (event)
        {
        case Event::EnqueueSucceeded:
            transition.next = State::InFlight;
            break;
        case Event::EnqueueFailed:
            transition.terminal = Terminal::Error;
            transition.release_token = true;
            break;
        case Event::StaleCallback:
            transition.count_stale = true;
            break;
        default:
            // Any event without a live transaction is stale evidence.
            transition.count_stale = true;
            break;
        }
        break;

    case State::InFlight:
        transition.next = State::InFlight;
        switch (event)
        {
        case Event::CompleteCallback:
            transition.next = State::Idle;
            transition.terminal = Terminal::Complete;
            transition.release_token = true;
            break;
        case Event::AbortCallback:
            transition.next = State::Idle;
            transition.terminal = Terminal::Aborted;
            transition.release_token = true;
            break;
        case Event::AbortAccepted:
            transition.next = State::AbortRequested;
            break;
        case Event::AbortRejected:
            // Rejection says nothing about whether the hardware request is
            // gone. End the business transaction, but retain channel credit
            // until recovery observes an empty mailbox or a verified reset.
            transition.next = State::RecoveryRequired;
            transition.terminal = Terminal::Error;
            transition.count_recovery = true;
            break;
        case Event::StaleCallback:
            transition.count_stale = true;
            break;
        case Event::ChannelReset:
            transition.next = State::Idle;
            transition.terminal = Terminal::Error;
            transition.release_token = true;
            transition.count_recovery = true;
            break;
        default:
            break;
        }
        break;

    case State::AbortRequested:
        transition.next = State::AbortRequested;
        switch (event)
        {
        case Event::CompleteCallback:
            transition.next = State::Idle;
            transition.terminal = Terminal::Complete;
            transition.release_token = true;
            break;
        case Event::AbortCallback:
            transition.next = State::Idle;
            transition.terminal = Terminal::Aborted;
            transition.release_token = true;
            break;
        case Event::MailboxEndedComplete:
            transition.next = State::Idle;
            transition.terminal = Terminal::Complete;
            transition.release_token = true;
            transition.count_recovery = true;
            break;
        case Event::MailboxEndedAborted:
            transition.next = State::Idle;
            transition.terminal = Terminal::Aborted;
            transition.release_token = true;
            transition.count_recovery = true;
            break;
        case Event::MailboxStillPending:
            // Hardware still owns the mailbox: deliver the business terminal
            // now but KEEP the token; a new frame must never share the
            // channel with the old one.
            transition.next = State::RecoveryRequired;
            transition.terminal = Terminal::Error;
            transition.count_recovery = true;
            break;
        case Event::StaleCallback:
            transition.count_stale = true;
            break;
        case Event::AbortAccepted:
            // Idempotent re-abort during recovery attempts.
            break;
        case Event::AbortRejected:
            // Re-abort failed: keep waiting under the bounded attempt count.
            break;
        case Event::ChannelReset:
            transition.next = State::Idle;
            transition.terminal = Terminal::Aborted;
            transition.release_token = true;
            transition.count_recovery = true;
            break;
        default:
            break;
        }
        break;

    case State::RecoveryRequired:
        transition.next = State::RecoveryRequired;
        switch (event)
        {
        case Event::MailboxEndedComplete:
        case Event::MailboxEndedAborted:
        case Event::CompleteCallback:
        case Event::AbortCallback:
        case Event::ChannelReset:
            // Business terminal was already delivered on entry; converge the
            // hardware state and return the token exactly once.
            transition.next = State::Idle;
            transition.release_token = true;
            transition.count_recovery = true;
            break;
        case Event::RecoveryExhausted:
            transition.next = State::Blocked;
            transition.count_recovery = true;
            break;
        case Event::MailboxStillPending:
            transition.count_recovery = true;
            break;
        case Event::StaleCallback:
            transition.count_stale = true;
            break;
        default:
            break;
        }
        break;

    case State::Blocked:
        transition.next = State::Blocked;
        switch (event)
        {
        case Event::ChannelReset:
            transition.next = State::Idle;
            transition.release_token = true;
            transition.count_recovery = true;
            break;
        default:
            transition.count_stale = true;
            break;
        }
        break;
    }
    return transition;
}

} // namespace dummy::can_tx

#endif // DUMMY_CAN_TX_LIFECYCLE_HPP
