#ifndef DUMMY_JOINT_SPACE_MAPPING_HPP
#define DUMMY_JOINT_SPACE_MAPPING_HPP

#include "../configurations/robot_config_generated.hpp"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

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

// Each arm motor exposes a single-turn absolute encoder and reconstructs
// multi-turn position only while that motor board remains powered.  With the
// 50:1 reduction, one lost motor revolution aliases the output joint by 7.2
// degrees.  The main controller therefore cannot claim an absolute arm pose
// after a motor reboot until an operator supplies the six known URDF joint
// positions.  The supplied pose selects integer motor-turn branches only; it
// never changes the encoder zero or invents a fractional correction.
constexpr float kAbsoluteJointSeedModuloToleranceRad = 0.02F;

enum class AbsoluteJointSeedResult : uint8_t
{
    Ok = 0,
    NonFinite = 1,
    OutOfRange = 2,
    ModuloMismatch = 3,
    AlreadySeeded = 4,
};

class AbsoluteJointPositionResolver
{
public:
    AbsoluteJointSeedResult Seed(
        const std::array<float, 6>& raw_legacy_degrees,
        const std::array<float, 6>& reference_urdf_rad)
    {
        if (valid_)
            return AbsoluteJointSeedResult::AlreadySeeded;

        constexpr float kDegreesToRadians = 0.01745329251994329577F;
        constexpr float kRadiansToDegrees = 57.295779513082320876F;
        constexpr float kTwoPi = 6.2831853071795864769F;
        std::array<float, 6> candidate_offsets{};
        for (size_t index = 0; index < candidate_offsets.size(); ++index)
        {
            const float raw_degrees = raw_legacy_degrees[index];
            const float reference = reference_urdf_rad[index];
            if (!std::isfinite(raw_degrees) || !std::isfinite(reference))
                return AbsoluteJointSeedResult::NonFinite;
            if (reference < generated_config::kJointMinRad[index] ||
                reference > generated_config::kJointMaxRad[index])
                return AbsoluteJointSeedResult::OutOfRange;

            const float expected_legacy_rad =
                UrdfRadiansToLegacyFirmwareRadians(reference, index);
            const float raw_legacy_rad = raw_degrees * kDegreesToRadians;
            const float alias_period_rad =
                kTwoPi / generated_config::kJointReduction[index];
            const float branch_turns = std::round(
                (expected_legacy_rad - raw_legacy_rad) / alias_period_rad);
            const float branch_offset_rad = branch_turns * alias_period_rad;
            const float residual_rad = expected_legacy_rad -
                (raw_legacy_rad + branch_offset_rad);
            if (!std::isfinite(branch_turns) ||
                std::fabs(residual_rad) >
                    kAbsoluteJointSeedModuloToleranceRad)
                return AbsoluteJointSeedResult::ModuloMismatch;
            candidate_offsets[index] =
                branch_offset_rad * kRadiansToDegrees;
        }

        branch_offset_degrees_ = candidate_offsets;
        generation_ = 1U;
        valid_ = true;
        return AbsoluteJointSeedResult::Ok;
    }

    bool valid() const { return valid_; }
    uint32_t generation() const { return generation_; }

    float ResolveLegacyDegrees(float raw_legacy_degrees,
                               size_t joint_index) const
    {
        return raw_legacy_degrees +
            (valid_ ? branch_offset_degrees_[joint_index] : 0.0F);
    }

    float MotorLocalTargetDegrees(float target_legacy_degrees,
                                  float init_pose_degrees,
                                  size_t joint_index) const
    {
        return target_legacy_degrees - init_pose_degrees -
            (valid_ ? branch_offset_degrees_[joint_index] : 0.0F);
    }

private:
    std::array<float, 6> branch_offset_degrees_{};
    uint32_t generation_ = 0U;
    bool valid_ = false;
};

} // namespace dummy::protocol

#endif // DUMMY_JOINT_SPACE_MAPPING_HPP
