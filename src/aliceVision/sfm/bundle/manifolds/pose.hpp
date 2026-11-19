// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/geometry/lie.hpp>
#include <Eigen/Dense>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <algorithm>
#include <cmath>

namespace aliceVision {
namespace sfm {

/**
 * @brief Manifold for a pose block stored as [angleAxis(3), center(3)],
 * where angleAxis encodes cam_R_world and center is the camera center in world coordinates.
 *
 * The tangent update [w, v] is expressed in the camera frame:
 *  - R' = exp(w) * R  (rotation around the camera center, in camera axes)
 *  - c' = c + R^T * v (center displacement along the camera axes)
 *
 * The step is therefore independent of the world origin, and the solver damping / jacobi scaling
 * distinguishes depth (camera z) from lateral (camera x/y) moves.
 * Rotation and/or center may be locked, which removes them from the tangent space.
 */
class PoseCameraFrameManifold : public ceres::Manifold
{
  public:
    PoseCameraFrameManifold(bool lockRotation, bool lockCenter)
      : _lockRotation(lockRotation),
        _lockCenter(lockCenter)
    {}

    ~PoseCameraFrameManifold() override = default;

    bool Plus(const double* x, const double* delta, double* x_plus_delta) const override
    {
        const Eigen::Matrix3d R = getRotation(x);
        const Eigen::Vector3d c(x[3], x[4], x[5]);

        Eigen::Vector3d w = Eigen::Vector3d::Zero();
        Eigen::Vector3d v = Eigen::Vector3d::Zero();
        splitDelta(delta, w, v);

        // locked parts are copied, so they stay bit-exact (no angle-axis / matrix round trip)
        if (_lockRotation)
        {
            std::copy(x, x + 3, x_plus_delta);
        }
        else
        {
            setRotation(SO3::expm(w) * R, x_plus_delta);
        }

        if (_lockCenter)
        {
            std::copy(x + 3, x + 6, x_plus_delta + 3);
        }
        else
        {
            const Eigen::Vector3d cnew = c + R.transpose() * v;
            x_plus_delta[3] = cnew(0);
            x_plus_delta[4] = cnew(1);
            x_plus_delta[5] = cnew(2);
        }

        return allFinite(x_plus_delta, AmbientSize());
    }

    bool PlusJacobian(const double* x, double* jacobian) const override
    {
        Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> J(jacobian, AmbientSize(), TangentSize());
        J.setZero();

        int col = 0;
        if (!_lockRotation)
        {
            // d log(exp(w) * exp(a)) / dw at w = 0 is the inverse left jacobian of SO3 at a
            const Eigen::Vector3d a(x[0], x[1], x[2]);
            J.block<3, 3>(0, col) = inverseLeftJacobian(a);
            col += 3;
        }
        if (!_lockCenter)
        {
            J.block<3, 3>(3, col) = getRotation(x).transpose();
        }

        return true;
    }

    bool Minus(const double* y, const double* x, double* delta) const override
    {
        const Eigen::Matrix3d Rx = getRotation(x);

        int idx = 0;
        if (!_lockRotation)
        {
            const Eigen::Matrix3d Rdiff = getRotation(y) * Rx.transpose();
            ceres::RotationMatrixToAngleAxis(Rdiff.data(), delta + idx);
            idx += 3;
        }
        if (!_lockCenter)
        {
            const Eigen::Vector3d v = Rx * Eigen::Vector3d(y[3] - x[3], y[4] - x[4], y[5] - x[5]);
            delta[idx++] = v(0);
            delta[idx++] = v(1);
            delta[idx++] = v(2);
        }

        return allFinite(delta, TangentSize());
    }

    bool MinusJacobian(const double* x, double* jacobian) const override
    {
        Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> J(jacobian, TangentSize(), AmbientSize());
        J.setZero();

        int row = 0;
        if (!_lockRotation)
        {
            // inverse of PlusJacobian rotation block: left jacobian of SO3 at a
            const Eigen::Vector3d a(x[0], x[1], x[2]);
            J.block<3, 3>(row, 0) = leftJacobian(a);
            row += 3;
        }
        if (!_lockCenter)
        {
            J.block<3, 3>(row, 3) = getRotation(x);
        }

        return true;
    }

    int AmbientSize() const override { return 6; }

    int TangentSize() const override { return (_lockRotation ? 0 : 3) + (_lockCenter ? 0 : 3); }

  private:
    static Eigen::Matrix3d getRotation(const double* x)
    {
        return SO3::expm(Eigen::Vector3d(x[0], x[1], x[2]));
    }

    static void setRotation(const Eigen::Matrix3d& R, double* x) { ceres::RotationMatrixToAngleAxis(R.data(), x); }

    void splitDelta(const double* delta, Eigen::Vector3d& w, Eigen::Vector3d& v) const
    {
        int idx = 0;
        if (!_lockRotation)
        {
            w = Eigen::Vector3d(delta[idx], delta[idx + 1], delta[idx + 2]);
            idx += 3;
        }
        if (!_lockCenter)
        {
            v = Eigen::Vector3d(delta[idx], delta[idx + 1], delta[idx + 2]);
        }
    }

    static bool allFinite(const double* values, int size)
    {
        for (int i = 0; i < size; ++i)
        {
            if (!std::isfinite(values[i]))
            {
                return false;
            }
        }
        return true;
    }

    // Below this angle, the closed forms lose precision (cancellation) and Taylor expansions are used.
    static constexpr double smallAngle = 1e-4;

    /**
     * @brief Left jacobian of SO3: I + (1 - cos(t)) / t^2 * K + (t - sin(t)) / t^3 * K^2, with K = skew(a), t = |a|
     */
    static Eigen::Matrix3d leftJacobian(const Eigen::Vector3d& a)
    {
        const double theta = a.norm();
        const double theta2 = theta * theta;
        const Eigen::Matrix3d K = SO3::skew(a);

        double c1, c2;
        if (theta < smallAngle)
        {
            c1 = 0.5 - theta2 / 24.0;
            c2 = 1.0 / 6.0 - theta2 / 120.0;
        }
        else
        {
            // 1 - cos(t) = 2 sin^2(t / 2) avoids the cancellation for small angles
            const double s = std::sin(0.5 * theta);
            c1 = 2.0 * s * s / theta2;
            c2 = (theta - std::sin(theta)) / (theta2 * theta);
        }

        return Eigen::Matrix3d::Identity() + c1 * K + c2 * K * K;
    }

    /**
     * @brief Inverse left jacobian of SO3: I - K / 2 + (1 - (t / 2) cot(t / 2)) / t^2 * K^2, with K = skew(a), t = |a|
     * Defined for t < 2 pi (blocks hold canonical angle-axis, t <= pi). At t = pi the K^2 coefficient is 1 / pi^2.
     */
    static Eigen::Matrix3d inverseLeftJacobian(const Eigen::Vector3d& a)
    {
        const double theta = a.norm();
        const double theta2 = theta * theta;
        const Eigen::Matrix3d K = SO3::skew(a);

        double c2;
        if (theta < smallAngle)
        {
            c2 = 1.0 / 12.0 + theta2 / 720.0;
        }
        else
        {
            const double half = 0.5 * theta;
            c2 = (1.0 - half / std::tan(half)) / theta2;
        }

        return Eigen::Matrix3d::Identity() - 0.5 * K + c2 * K * K;
    }

    bool _lockRotation;
    bool _lockCenter;
};

}  // namespace sfm
}  // namespace aliceVision
