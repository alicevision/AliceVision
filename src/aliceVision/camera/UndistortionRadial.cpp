// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "UndistortionRadial.hpp"

namespace aliceVision {
namespace camera {

Vec2 UndistortionRadialK3::undistortNormalized(const Vec2& p) const
{
    const double& k1 = _undistortionParams[0];
    const double& k2 = _undistortionParams[1];
    const double& k3 = _undistortionParams[2];

    const double r = sqrt(p(0) * p(0) + p(1) * p(1));

    const double r2 = r * r;
    const double r4 = r2 * r2;
    const double r6 = r4 * r2;
    const double r_coeff = (1. + k1 * r2 + k2 * r4 + k3 * r6);

    return p * r_coeff;
}

Eigen::Matrix<double, 2, 2> UndistortionRadialK3::getDerivativeUndistortNormalizedwrtPoint(const Vec2& p) const
{
    const double& k1 = _undistortionParams[0];
    const double& k2 = _undistortionParams[1];
    const double& k3 = _undistortionParams[2];

    const double r2 = p(0) * p(0) + p(1) * p(1);
    const double eps = 1e-21;
    if (r2 < eps)
    {
        return Eigen::Matrix2d::Identity();
    }

    const double r4 = r2 * r2;
    const double r6 = r4 * r2;

    const double r_coeff = 1.0 + k1 * r2 + k2 * r4 + k3 * r6;
    const double d_coeff = 2.0 * k1 + 4.0 * k2 * r2 + 6.0 * k3 * r4;

    Eigen::Matrix2d ret;
    ret(0, 0) = r_coeff + p(0) * d_coeff * p(0);
    ret(0, 1) = p(0) * d_coeff * p(1);
    ret(1, 0) = p(1) * d_coeff * p(0);
    ret(1, 1) = r_coeff + p(1) * d_coeff * p(1);

    return ret;
}

Eigen::Matrix<double, 2, Eigen::Dynamic> UndistortionRadialK3::getDerivativeUndistortNormalizedwrtParameters(const Vec2& p) const
{
    const double r2 = p(0) * p(0) + p(1) * p(1);
    const double eps = 1e-21;
    if (r2 < eps)
    {
        return Eigen::Matrix<double, 2, 3>::Zero();
    }

    const double r4 = r2 * r2;
    const double r6 = r4 * r2;

    Eigen::Matrix<double, 2, 3> ret;
    ret(0, 0) = p(0) * r2;
    ret(0, 1) = p(0) * r4;
    ret(0, 2) = p(0) * r6;
    ret(1, 0) = p(1) * r2;
    ret(1, 1) = p(1) * r4;
    ret(1, 2) = p(1) * r6;

    return ret;
}

Vec2 UndistortionRadialK3::inverseNormalized(const Vec2& p) const
{
    const double epsilon = 1e-8;
    Vec2 distorted_value = p;

    Vec2 diff = undistortNormalized(distorted_value) - p;

    int iter = 0;
    while (diff.norm() > epsilon)
    {
        distorted_value = distorted_value - getDerivativeUndistortNormalizedwrtPoint(distorted_value).inverse() * diff;
        diff = undistortNormalized(distorted_value) - p;
        iter++;
        if (iter > 100)
            break;
    }

    return distorted_value;
}

}  // namespace camera
}  // namespace aliceVision
