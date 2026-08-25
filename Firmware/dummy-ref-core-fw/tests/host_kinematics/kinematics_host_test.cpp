#include "6dof_kinematic.h"
#include "joint_space_mapping.hpp"

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <limits>

namespace
{
constexpr float kRadiansToDegrees = 57.295779513082320876F;

struct GoldenVector
{
    std::array<float, 6> urdf_joint_rad;
    std::array<float, 3> firmware_position_m;
    std::array<float, 9> firmware_rotation;
};

constexpr std::array<GoldenVector, 3> kGoldenVectors = {{
    GoldenVector{{0.0F, 0.052359878F, 0.0F, 0.0F, 0.0F, 0.0F},
     {0.117111690F, 0.0F, 0.180200920F},
     {-0.342020661F, 0.0F, 0.939692438F,
       0.0F, 1.0F, 0.0F,
       -0.939692438F, 0.0F, -0.342020661F}},
    GoldenVector{{0.3F, 1.0F, -1.0F, 0.4F, 0.2F, 0.5F},
     {0.201533228F, 0.054324277F, 0.265580088F},
     {-0.098931909F, -0.488150835F, 0.867133915F,
       -0.843421340F, 0.503563941F, 0.187253475F,
       -0.528065324F, -0.712834001F, -0.461535335F}},
    GoldenVector{{-0.5F, 1.5F, -2.0F, -0.7F, 0.6F, 1.2F},
     {0.244505554F, -0.092539102F, 0.345901906F},
     {-0.278341174F, -0.018993966F, 0.960294485F,
       -0.440704942F, 0.890872240F, -0.110117435F,
       -0.853408098F, -0.453856647F, -0.256337196F}},
}};

DOF6Kinematic MakeSolver()
{
    return DOF6Kinematic(0.15475F, 0.03500F, 0.14600F, 0.115455F, 0.05200F, 0.09900F);
}

DOF6Kinematic::Joint6D_t ToLegacyDegrees(const std::array<float, 6>& urdf_joint_rad)
{
    DOF6Kinematic::Joint6D_t result{};
    for (std::size_t index = 0; index < urdf_joint_rad.size(); ++index)
    {
        result.a[index] = dummy::protocol::UrdfRadiansToLegacyFirmwareRadians(
            urdf_joint_rad[index], index) * kRadiansToDegrees;
    }
    return result;
}

bool IsUsableBranch(const DOF6Kinematic::IKSolves_t& solutions, int branch)
{
    for (int flag = 0; flag < 3; ++flag)
    {
        if (solutions.solFlag[branch][flag] == 0)
            return false;
    }
    for (float joint : solutions.config[branch].a)
    {
        if (!std::isfinite(joint))
            return false;
    }
    return true;
}

float MaximumJointDifference(const DOF6Kinematic::Joint6D_t& left,
                             const DOF6Kinematic::Joint6D_t& right)
{
    float maximum = 0.0F;
    for (int index = 0; index < 6; ++index)
        maximum = std::max(maximum, std::fabs(left.a[index] - right.a[index]));
    return maximum;
}

void AssertPoseNear(const DOF6Kinematic::Pose6D_t& pose,
                    const GoldenVector& expected)
{
    const std::array<float, 3> position = {pose.X, pose.Y, pose.Z};
    for (std::size_t index = 0; index < position.size(); ++index)
        assert(std::fabs(position[index] - expected.firmware_position_m[index]) < 2.0e-6F);
    for (std::size_t index = 0; index < expected.firmware_rotation.size(); ++index)
        assert(std::fabs(pose.R[index] - expected.firmware_rotation[index]) < 2.0e-5F);
}

void TestForwardGoldenVectorsAndInverseRoundTrip()
{
    DOF6Kinematic solver = MakeSolver();
    for (std::size_t vector_index = 0; vector_index < kGoldenVectors.size(); ++vector_index)
    {
        const GoldenVector& golden = kGoldenVectors[vector_index];
        const DOF6Kinematic::Joint6D_t seed = ToLegacyDegrees(golden.urdf_joint_rad);
        DOF6Kinematic::Pose6D_t pose{};
        assert(solver.SolveFK(seed, pose));
        AssertPoseNear(pose, golden);

        // The historical analytic solver wraps the home pose's legacy J3=180
        // degree boundary to zero.  Keep its FK as a regression vector, but do
        // not pretend that boundary branch has passed IK continuity validation.
        if (vector_index == 0)
            continue;

        DOF6Kinematic::Pose6D_t target = pose;
        target.X *= 1000.0F;
        target.Y *= 1000.0F;
        target.Z *= 1000.0F;
        target.hasR = true;
        DOF6Kinematic::IKSolves_t solutions{};
        assert(solver.SolveIK(target, seed, solutions));

        float best_difference = std::numeric_limits<float>::infinity();
        for (int branch = 0; branch < 8; ++branch)
        {
            if (!IsUsableBranch(solutions, branch))
                continue;
            best_difference = std::min(
                best_difference,
                MaximumJointDifference(seed, solutions.config[branch]));
        }
        assert(std::isfinite(best_difference));
        assert(best_difference < 0.02F);
    }
}

void TestUnreachablePoseHasNoUsableBranch()
{
    DOF6Kinematic solver = MakeSolver();
    const DOF6Kinematic::Joint6D_t seed = ToLegacyDegrees(kGoldenVectors[1].urdf_joint_rad);
    DOF6Kinematic::Pose6D_t pose{};
    assert(solver.SolveFK(seed, pose));
    pose.X = 10000.0F;
    pose.Y *= 1000.0F;
    pose.Z *= 1000.0F;
    pose.hasR = true;
    DOF6Kinematic::IKSolves_t solutions{};
    assert(solver.SolveIK(pose, seed, solutions));
    for (int branch = 0; branch < 8; ++branch)
        assert(!IsUsableBranch(solutions, branch));
}
} // namespace

int main()
{
    TestForwardGoldenVectorsAndInverseRoundTrip();
    TestUnreachablePoseHasNoUsableBranch();
    return 0;
}
