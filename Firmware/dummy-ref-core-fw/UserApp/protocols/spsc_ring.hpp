#ifndef DUMMY_SPSC_RING_HPP
#define DUMMY_SPSC_RING_HPP

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace dummy::protocol
{

// Fixed-capacity single-producer/single-consumer ring. Monotonic sequence
// counters preserve all Capacity slots (there is no sacrificed sentinel slot)
// and release/acquire publication keeps an ISR producer independent from its
// task-context consumer.
template<typename T, size_t Capacity>
class SpscRing
{
    static_assert(Capacity > 0U, "SPSC capacity must be positive");
    static_assert(std::is_trivially_copyable<T>::value,
                  "SPSC entries must be trivially copyable");

public:
    bool Push(const T& value)
    {
        const uint32_t write = write_sequence_.load(
            std::memory_order_relaxed);
        const uint32_t read = read_sequence_.load(
            std::memory_order_acquire);
        if (write - read >= Capacity)
            return false;
        entries_[write % Capacity] = value;
        write_sequence_.store(write + 1U, std::memory_order_release);
        return true;
    }

    bool Pop(T& value)
    {
        const uint32_t read = read_sequence_.load(
            std::memory_order_relaxed);
        const uint32_t write = write_sequence_.load(
            std::memory_order_acquire);
        if (read == write)
            return false;
        value = entries_[read % Capacity];
        read_sequence_.store(read + 1U, std::memory_order_release);
        return true;
    }

    size_t Size() const
    {
        const uint32_t write = write_sequence_.load(
            std::memory_order_acquire);
        const uint32_t read = read_sequence_.load(
            std::memory_order_acquire);
        return static_cast<size_t>(write - read);
    }

    constexpr size_t capacity() const { return Capacity; }

private:
    std::array<T, Capacity> entries_{};
    std::atomic<uint32_t> read_sequence_{0U};
    std::atomic<uint32_t> write_sequence_{0U};
};

} // namespace dummy::protocol

#endif // DUMMY_SPSC_RING_HPP
