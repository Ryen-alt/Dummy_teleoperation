#ifndef DUMMY_PUBLISHED_DOUBLE_BUFFER_HPP
#define DUMMY_PUBLISHED_DOUBLE_BUFFER_HPP

#include <array>
#include <atomic>
#include <cstdint>

namespace dummy::protocol
{

// One task writes the inactive slot and atomically publishes it. Readers pin
// the selected slot while copying, so the writer can skip (never block on) a
// slot still in use. This removes long global interrupt masking from snapshot
// readers without permitting a partially copied value.
template<typename T>
class PublishedDoubleBuffer
{
public:
    bool TryPublish(const T& value)
    {
        const uint8_t active = active_index_.load(std::memory_order_acquire);
        const uint8_t inactive = static_cast<uint8_t>(active ^ 1U);
        if (reader_count_[inactive].load(std::memory_order_acquire) != 0U)
            return false;
        slots_[inactive] = value;
        active_index_.store(inactive, std::memory_order_release);
        return true;
    }

    T Read() const
    {
        return WithRead([](const T& value) { return value; });
    }

    // Keep a slot pinned for a bounded visitor. The visitor must not retain
    // references to the snapshot. Also supports exercising publisher back-
    // pressure without replacing the production snapshot implementation.
    template<typename Visitor>
    auto WithRead(Visitor visitor) const
    {
        for (;;)
        {
            const uint8_t index = active_index_.load(
                std::memory_order_acquire);
            reader_count_[index].fetch_add(1U, std::memory_order_acq_rel);
            if (active_index_.load(std::memory_order_acquire) == index)
            {
                struct Pin
                {
                    std::atomic<uint32_t>& count;
                    ~Pin() { count.fetch_sub(1U, std::memory_order_release); }
                } pin{reader_count_[index]};
                return visitor(slots_[index]);
            }
            reader_count_[index].fetch_sub(1U, std::memory_order_release);
        }
    }

private:
    std::array<T, 2U> slots_{};
    mutable std::array<std::atomic<uint32_t>, 2U> reader_count_{};
    std::atomic<uint8_t> active_index_{0U};
};

} // namespace dummy::protocol

#endif // DUMMY_PUBLISHED_DOUBLE_BUFFER_HPP
