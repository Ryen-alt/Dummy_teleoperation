#ifndef DUMMY_IEEE754_FINITE_HPP
#define DUMMY_IEEE754_FINITE_HPP

#include <cstdint>
#include <cstring>

namespace dummy::protocol
{

// Explicit IEEE-754 binary32 classification for protocol and safety
// boundaries (doc 07 R05). Unlike std::isfinite, this integer bit-pattern
// test cannot be folded into a constant "true" by -Ofast/-ffinite-math-only
// builds. Values whose exponent field is all ones are NaN or infinity.
inline bool Ieee754IsFinite(float value)
{
    static_assert(sizeof(float) == sizeof(uint32_t),
                  "binary32 float required");
    uint32_t bits = 0U;
    std::memcpy(&bits, &value, sizeof(bits));
    constexpr uint32_t kExponentMask = 0x7F800000U;
    return (bits & kExponentMask) != kExponentMask;
}

} // namespace dummy::protocol

#endif // DUMMY_IEEE754_FINITE_HPP
