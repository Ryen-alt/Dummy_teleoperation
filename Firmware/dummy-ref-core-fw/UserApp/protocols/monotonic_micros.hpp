#ifndef DUMMY_MONOTONIC_MICROS_HPP
#define DUMMY_MONOTONIC_MICROS_HPP

#include <cstdint>

namespace dummy::protocol
{

// Measure a short interval between two samples of the wrapping 32-bit
// microsecond clock.  An ISR may capture `then_us` just after a task captured
// `now_us`; that small apparent future timestamp must be treated as zero, not
// as an interval near 2^32 us.  Genuine counter wrap still produces a small
// positive result.
inline uint32_t RecentElapsedMicros32(uint32_t now_us, uint32_t then_us)
{
    const uint32_t elapsed_us = now_us - then_us;
    constexpr uint32_t kHalfRange = uint32_t{1} << 31U;
    return elapsed_us >= kHalfRange ? 0U : elapsed_us;
}

// Reconstruct a recent 32-bit sample timestamp against an extended current
// time. CAN RX may publish a sample a few microseconds after the caller took
// its `now_us` snapshot. Treat that small apparent future offset as zero age;
// otherwise unsigned subtraction would turn it into an age near 2^32 and the
// following uint64 subtraction would underflow near 2^64.
inline uint64_t ExtendRecentMicros32(uint64_t now_us, uint32_t sample_low_us)
{
    const uint32_t now_low_us = static_cast<uint32_t>(now_us);
    constexpr uint32_t kHalfRange = uint32_t{1} << 31U;
    if (sample_low_us > now_low_us &&
        sample_low_us - now_low_us < kHalfRange)
    {
        return now_us;
    }

    const uint32_t age_us = RecentElapsedMicros32(now_low_us, sample_low_us);
    return static_cast<uint64_t>(age_us) > now_us
        ? now_us : now_us - age_us;
}

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
