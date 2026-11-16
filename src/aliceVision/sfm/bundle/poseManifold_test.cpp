// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#define BOOST_TEST_MODULE poseManifold

#include <aliceVision/sfm/bundle/manifolds/pose.hpp>

#include <boost/test/unit_test.hpp>

#include <array>
#include <cmath>
#include <limits>
#include <random>
#include <vector>

using namespace aliceVision;
using namespace aliceVision::sfm;

using RowMajorMatrix = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

namespace {

/// The three lock modes that keep some part of the pose refined: none, rotation, center.
const std::vector<std::pair<bool, bool>> lockModes = {{false, false}, {true, false}, {false, true}};

/// Rotation angles, from 0 to pi, including both sides of the small angle switch of the jacobians.
const std::vector<double> angles = {0.0, 1e-14, 1e-10, 1e-8, 1e-6, 9.999e-5, 1e-4, 1.0001e-4, 1e-3, 0.1, 1.0, 2.0, 3.0, M_PI - 1e-3, M_PI - 1e-6, M_PI};

/// Random pose block [angleAxis(3), center(3)] with the given rotation angle.
std::array<double, 6> randomPose(std::mt19937& gen, double angle)
{
    std::uniform_real_distribution<double> dist(-1.0, 1.0);
    Eigen::Vector3d axis(dist(gen), dist(gen), dist(gen));
    axis.normalize();
    const Eigen::Vector3d a = axis * angle;
    return {a(0), a(1), a(2), 100.0 * dist(gen), 100.0 * dist(gen), 100.0 * dist(gen)};
}

RowMajorMatrix plusJacobian(const PoseCameraFrameManifold& manifold, const double* x)
{
    RowMajorMatrix J(manifold.AmbientSize(), manifold.TangentSize());
    manifold.PlusJacobian(x, J.data());
    return J;
}

RowMajorMatrix minusJacobian(const PoseCameraFrameManifold& manifold, const double* x)
{
    RowMajorMatrix J(manifold.TangentSize(), manifold.AmbientSize());
    manifold.MinusJacobian(x, J.data());
    return J;
}

}  // namespace

BOOST_AUTO_TEST_CASE(poseManifold_sizes)
{
    BOOST_CHECK_EQUAL(PoseCameraFrameManifold(false, false).TangentSize(), 6);
    BOOST_CHECK_EQUAL(PoseCameraFrameManifold(true, false).TangentSize(), 3);
    BOOST_CHECK_EQUAL(PoseCameraFrameManifold(false, true).TangentSize(), 3);

    for (const auto& [lockRotation, lockCenter] : lockModes)
    {
        BOOST_CHECK_EQUAL(PoseCameraFrameManifold(lockRotation, lockCenter).AmbientSize(), 6);
    }
}

BOOST_AUTO_TEST_CASE(poseManifold_plusIsCameraFrame)
{
    // A center step along the tangent z axis moves the camera along its optical axis,
    // and a rotation step turns it around its own center.
    std::mt19937 gen(0);
    const PoseCameraFrameManifold manifold(false, false);

    for (int i = 0; i < 20; ++i)
    {
        const std::array<double, 6> x = randomPose(gen, 1.0);
        Eigen::Matrix3d R;
        ceres::AngleAxisToRotationMatrix(x.data(), R.data());

        const double depthStep[6] = {0, 0, 0, 0, 0, 2.0};
        std::array<double, 6> moved;
        BOOST_REQUIRE(manifold.Plus(x.data(), depthStep, moved.data()));
        const Eigen::Vector3d displacement(moved[3] - x[3], moved[4] - x[4], moved[5] - x[5]);
        BOOST_CHECK_SMALL((displacement - 2.0 * R.row(2).transpose()).norm(), 1e-12);

        const double rotationStep[6] = {0.1, -0.2, 0.3, 0, 0, 0};
        std::array<double, 6> turned;
        BOOST_REQUIRE(manifold.Plus(x.data(), rotationStep, turned.data()));
        Eigen::Matrix3d Rturned;
        ceres::AngleAxisToRotationMatrix(turned.data(), Rturned.data());
        const Eigen::Matrix3d expected = SO3::expm(Eigen::Vector3d(0.1, -0.2, 0.3)) * R;
        BOOST_CHECK_SMALL((Rturned - expected).cwiseAbs().maxCoeff(), 1e-12);
        BOOST_CHECK_EQUAL(turned[3], x[3]);
        BOOST_CHECK_EQUAL(turned[4], x[4]);
        BOOST_CHECK_EQUAL(turned[5], x[5]);
    }
}

BOOST_AUTO_TEST_CASE(poseManifold_plusJacobianMatchesFiniteDifferences)
{
    std::mt19937 gen(1);
    const double h = 1e-6;

    for (const auto& [lockRotation, lockCenter] : lockModes)
    {
        const PoseCameraFrameManifold manifold(lockRotation, lockCenter);
        const int tangentSize = manifold.TangentSize();

        for (double angle : angles)
        {
            // Plus returns the canonical angle-axis: finite differences across the pi wrap are meaningless.
            if (angle > M_PI - 1e-4)
                continue;

            for (int i = 0; i < 10; ++i)
            {
                const std::array<double, 6> x = randomPose(gen, angle);
                const RowMajorMatrix J = plusJacobian(manifold, x.data());

                for (int k = 0; k < tangentSize; ++k)
                {
                    std::array<double, 6> delta{};
                    std::array<double, 6> plus, minus;
                    delta[k] = h;
                    BOOST_REQUIRE(manifold.Plus(x.data(), delta.data(), plus.data()));
                    delta[k] = -h;
                    BOOST_REQUIRE(manifold.Plus(x.data(), delta.data(), minus.data()));

                    for (int r = 0; r < 6; ++r)
                    {
                        const double numerical = (plus[r] - minus[r]) / (2.0 * h);
                        BOOST_TEST_CONTEXT("lockRotation " << lockRotation << ", lockCenter " << lockCenter << ", angle " << angle)
                        {
                            BOOST_CHECK_SMALL((numerical - J(r, k)) / (1.0 + std::abs(J(r, k))), 1e-7);
                        }
                    }
                }
            }
        }
    }
}

BOOST_AUTO_TEST_CASE(poseManifold_minusInvertsPlus)
{
    std::mt19937 gen(2);
    std::uniform_real_distribution<double> dist(-0.05, 0.05);

    for (const auto& [lockRotation, lockCenter] : lockModes)
    {
        const PoseCameraFrameManifold manifold(lockRotation, lockCenter);
        const int tangentSize = manifold.TangentSize();

        for (double angle : angles)
        {
            if (angle > M_PI - 1e-4)
                continue;

            for (int i = 0; i < 10; ++i)
            {
                const std::array<double, 6> x = randomPose(gen, angle);
                std::array<double, 6> delta{};
                for (int k = 0; k < tangentSize; ++k)
                    delta[k] = dist(gen);

                std::array<double, 6> y, recovered{};
                BOOST_REQUIRE(manifold.Plus(x.data(), delta.data(), y.data()));
                BOOST_REQUIRE(manifold.Minus(y.data(), x.data(), recovered.data()));

                for (int k = 0; k < tangentSize; ++k)
                {
                    BOOST_CHECK_SMALL(recovered[k] - delta[k], 1e-12);
                }
            }
        }
    }
}

BOOST_AUTO_TEST_CASE(poseManifold_minusJacobianInvertsPlusJacobian)
{
    std::mt19937 gen(3);

    for (const auto& [lockRotation, lockCenter] : lockModes)
    {
        const PoseCameraFrameManifold manifold(lockRotation, lockCenter);
        const int tangentSize = manifold.TangentSize();

        for (double angle : angles)
        {
            for (int i = 0; i < 10; ++i)
            {
                const std::array<double, 6> x = randomPose(gen, angle);
                const RowMajorMatrix Jplus = plusJacobian(manifold, x.data());
                const RowMajorMatrix Jminus = minusJacobian(manifold, x.data());

                BOOST_CHECK(Jplus.allFinite());
                BOOST_CHECK(Jminus.allFinite());

                const Eigen::MatrixXd product = Jminus * Jplus;
                BOOST_CHECK_SMALL((product - Eigen::MatrixXd::Identity(tangentSize, tangentSize)).cwiseAbs().maxCoeff(), 1e-12);
            }
        }
    }
}

BOOST_AUTO_TEST_CASE(poseManifold_rotationJacobianAtPi)
{
    // At |a| = pi, the K^2 coefficient of the inverse left jacobian is 1 / pi^2 (cot(pi / 2) = 0).
    const PoseCameraFrameManifold manifold(false, true);
    const Eigen::Vector3d a = Eigen::Vector3d(1.0, 2.0, -0.5).normalized() * M_PI;
    const double x[6] = {a(0), a(1), a(2), 0, 0, 0};

    const RowMajorMatrix J = plusJacobian(manifold, x);
    const Eigen::Matrix3d K = SO3::skew(a);
    const Eigen::Matrix3d expected = Eigen::Matrix3d::Identity() - 0.5 * K + (1.0 / (M_PI * M_PI)) * K * K;

    BOOST_CHECK_SMALL((J.block(0, 0, 3, 3) - expected).cwiseAbs().maxCoeff(), 1e-12);
}

BOOST_AUTO_TEST_CASE(poseManifold_lockedPartsAreExact)
{
    // The locked part of the block is copied by Plus: it must not drift through the angle-axis round trip.
    std::mt19937 gen(4);

    for (double angle : angles)
    {
        const std::array<double, 6> x = randomPose(gen, angle);
        const double step[3] = {0.01, -0.02, 0.03};

        std::array<double, 6> y;
        BOOST_REQUIRE(PoseCameraFrameManifold(true, false).Plus(x.data(), step, y.data()));
        for (int k = 0; k < 3; ++k)
            BOOST_CHECK_EQUAL(y[k], x[k]);

        BOOST_REQUIRE(PoseCameraFrameManifold(false, true).Plus(x.data(), step, y.data()));
        for (int k = 3; k < 6; ++k)
            BOOST_CHECK_EQUAL(y[k], x[k]);
    }
}

BOOST_AUTO_TEST_CASE(poseManifold_rejectsNonFinite)
{
    const PoseCameraFrameManifold manifold(false, false);
    const double zeroStep[6] = {0, 0, 0, 0, 0, 0};
    std::array<double, 6> out;

    const double nanPose[6] = {std::numeric_limits<double>::quiet_NaN(), 0, 0, 0, 0, 0};
    BOOST_CHECK(!manifold.Plus(nanPose, zeroStep, out.data()));

    const double pose[6] = {0.1, 0.2, 0.3, 1, 2, 3};
    const double infStep[6] = {0, 0, 0, std::numeric_limits<double>::infinity(), 0, 0};
    BOOST_CHECK(!manifold.Plus(pose, infStep, out.data()));

    std::array<double, 6> delta;
    BOOST_CHECK(!manifold.Minus(nanPose, pose, delta.data()));
}
