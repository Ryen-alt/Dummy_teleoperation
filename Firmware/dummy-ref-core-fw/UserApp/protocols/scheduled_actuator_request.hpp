#ifndef DUMMY_SCHEDULED_ACTUATOR_REQUEST_HPP
#define DUMMY_SCHEDULED_ACTUATOR_REQUEST_HPP

#include <array>
#include <cstdint>

namespace dummy::protocol
{
enum class ScheduledActuatorMode : uint8_t { Idle, Stream, Hold, Fault };

struct ScheduledActuatorRequest
{
    ScheduledActuatorMode mode = ScheduledActuatorMode::Idle;
    std::array<float, 7> position{};
    uint32_t sequence = 0U;
    uint32_t session_epoch = 0U;

    bool HasTargetFor(uint32_t epoch) const
    {
        return mode == ScheduledActuatorMode::Stream && sequence != 0U &&
            epoch != 0U && session_epoch == epoch;
    }
};
}
#endif
