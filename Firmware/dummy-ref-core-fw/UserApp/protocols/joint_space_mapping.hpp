#ifndef DUMMY_JOINT_SPACE_MAPPING_HPP
#define DUMMY_JOINT_SPACE_MAPPING_HPP

#include "../configurations/robot_config_generated.hpp"

#include <cstddef>

namespace dummy::protocol
{

// The binary host API uses the URDF joint convention. DummyRobot's historical
// motion/FK code continues to use the legacy firmware convention internally.
//
//   q_urdf = sign * (q_firmware - firmware_zero)
//   q_firmware = firmware_zero + sign * q_urdf
//
// kJointZeroOffsetRad is the legacy firmware angle observed at URDF q=0.
inline float LegacyFirmwareDegreesToUrdfRadians(float firmware_degrees,
                                                 size_t joint_index)
{
    constexpr float kDegreesToRadians = 0.01745329251994329577F;
    const float firmware_radians = firmware_degrees * kDegreesToRadians;
    return static_cast<float>(generated_config::kJointSign[joint_index]) *
           (firmware_radians - generated_config::kJointZeroOffsetRad[joint_index]);
}

inline float UrdfRadiansToLegacyFirmwareRadians(float urdf_radians,
                                                 size_t joint_index)
{
    return generated_config::kJointZeroOffsetRad[joint_index] +
           static_cast<float>(generated_config::kJointSign[joint_index]) * urdf_radians;
}

} // namespace dummy::protocol

#endif // DUMMY_JOINT_SPACE_MAPPING_HPP
