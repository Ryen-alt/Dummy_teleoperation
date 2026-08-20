#ifndef DUMMY_MONOTONIC_MICROS_HPP
#define DUMMY_MONOTONIC_MICROS_HPP

#include <cstdint>

namespace dummy::protocol
{

// Extends a wrapping 32-bit microsecond counter without treating a small
// backwards sample as a full wrap. Some boards compose micros() from two
// timer domains, so adjacent samples can regress slightly at a millisecond
// boundary even though the underlying clock has not wrapped.
class MonotonicMicros32
{
public:
    uint64_t Extend(uint32_t current)
    {
        if (!initialized_)
        {
            initialized_ = true;
            last_raw_ = current;
            return current;
        }

        if (current < last_raw_)
        {
            constexpr uint32_t kWrapThreshold = uint32_t{1} << 31U;
            const uint32_t backwards = last_raw_ - current;
            if (backwards > kWrapThreshold)
                epoch_ += uint64_t{1} << 32U;
            else
                current = last_raw_;
        }

        last_raw_ = current;
        return epoch_ + current;
    }

private:
    bool initialized_ = false;
    uint32_t last_raw_ = 0;
    uint64_t epoch_ = 0;
};

} // namespace dummy::protocol

#endif // DUMMY_MONOTONIC_MICROS_HPP
