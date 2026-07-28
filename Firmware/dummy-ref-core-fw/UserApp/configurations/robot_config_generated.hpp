#ifndef DUMMY_ROBOT_CONFIG_GENERATED_HPP
#define DUMMY_ROBOT_CONFIG_GENERATED_HPP

#include <array>
#include <cstdint>

namespace dummy::generated_config
{

// Generated from upper_computer:Upper/dummy-host/configs/robot_config.yaml.
// The source values came from legacy degree-based firmware and are NOT calibrated.
constexpr uint32_t kConfigVersion = 1;
constexpr bool kHardwareParametersVerified = false;
// Keep false until the 200 Hz consumer atomically adopts latest-wins targets.
constexpr bool kExternalTargetExecutionReady = false;
constexpr std::array<uint8_t, 32> kConfigSha256 = {
    0xb9, 0xd5, 0x5e, 0x04, 0xfb, 0x8f, 0xfa, 0xfb,
    0xd8, 0xa2, 0x3a, 0x74, 0xee, 0xb0, 0x37, 0x63,
    0x43, 0x61, 0x54, 0xaf, 0xfe, 0x3c, 0x72, 0x2e,
    0x15, 0x30, 0x2a, 0x0a, 0x00, 0x0b, 0x80, 0xb4,
};
constexpr std::array<float, 6> kJointMinRad = {
    -2.967060F, -1.274090F, 0.610865F, -3.141593F, -2.094395F, -12.566371F,
};
constexpr std::array<float, 6> kJointMaxRad = {
    2.967060F, 1.570796F, 3.141593F, 3.141593F, 2.094395F, 12.566371F,
};
constexpr std::array<float, 6> kMaxVelocityRadS = {
    0.35F, 0.35F, 0.35F, 0.50F, 0.50F, 0.70F,
};

} // namespace dummy::generated_config

#endif // DUMMY_ROBOT_CONFIG_GENERATED_HPP
