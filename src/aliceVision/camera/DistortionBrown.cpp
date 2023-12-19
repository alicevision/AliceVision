// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "DistortionBrown.hpp"
#include <aliceVision/system/Logger.hpp>
#include <vector>

namespace aliceVision {
namespace camera {

Vec2 DistortionBrown::distoFunction(const std::vector<double>& params, const Vec2& p)
{
    const double &k1 = params[0], k2 = params[1], k3 = params[2], t1 = params[3], t2 = params[4];
    const double r2 = p(0) * p(0) + p(1) * p(1);
    const double r4 = r2 * r2;
    const double r6 = r4 * r2;
    const double k_diff = (k1 * r2 + k2 * r4 + k3 * r6);
    const double t_x = t2 * (r2 + 2 * p(0) * p(0)) + 2 * t1 * p(0) * p(1);
    const double t_y = t1 * (r2 + 2 * p(1) * p(1)) + 2 * t2 * p(0) * p(1);
    Vec2 d(p(0) * k_diff + t_x, p(1) * k_diff + t_y);

    return d;
}

Vec2 DistortionBrown::addDistortion(const Vec2& p) const 
{ 
    return (p + distoFunction(_distortionParams, p)); 
}

Eigen::Matrix2d DistortionBrown::getDerivativeAddDistoWrtPt(const Vec2& p) const 
{
    const double k1 = _distortionParams[0];
    const double k2 = _distortionParams[1];
    const double k3 = _distortionParams[2];
    const double t1 = _distortionParams[3]; 
    const double t2 = _distortionParams[4];

    const double r2 = p(0) * p(0) + p(1) * p(1);
    const double r4 = r2 * r2;
    const double r6 = r4 * r2;
    double px = p(0);
    double py = p(1);

    Eigen::Matrix2d ret;
    ret(0, 0) = k1*r2 + k2*r4 + k3*r6 + 2*px*px*(k1 + 2*k2*r2 + 3*k3*r4) + 6*px*t2;
    ret(0, 1) = 2*py*(px*(k1 + 2*k2*r2 + 3*k3*r4) + 2*t1 + t2);
    ret(1, 0) = 2*px*(py*(k1 + 2*k2*r2 + 3*k3*r4) + t1 + 2*t2);
    ret(1, 1) = k1*r2 + k2*r4 + k3*r6 + 2*py*py*(k1 + 2*k2*r2 + 3*k3*r4) + 6*py*t1;

    return ret;
}

Eigen::MatrixXd DistortionBrown::getDerivativeAddDistoWrtDisto(const Vec2& p) const
{
    const double k1 = _distortionParams[0];
    const double k2 = _distortionParams[1];
    const double k3 = _distortionParams[2];
    const double t1 = _distortionParams[3]; 
    const double t2 = _distortionParams[4];

    const double r2 = p(0) * p(0) + p(1) * p(1);
    const double r4 = r2 * r2;
    const double r6 = r4 * r2;
    double px = p(0);
    double py = p(1);

    Eigen::Matrix<double, 2, 5> ret;
    ret(0, 0) = px*r2;
    ret(0, 1) = px*r4;
    ret(0, 2) = px*r6;
    ret(0, 3) = 2*py*py;
    ret(0, 4) = 3*px*px + py*py;

    ret(1, 0) = py*r2;
    ret(1, 1) = py*r4;
    ret(1, 2) = py*r6;
    ret(1, 3) = px*px + 3.0 * py * py;
    ret(1, 4) = 2.0 * px * px;

    return ret;
}

Vec2 DistortionBrown::removeDistortion(const Vec2& p) const
{
    const double epsilon = 1e-8;  // criteria to stop the iteration
    Vec2 p_u = p;

    while ((addDistortion(p_u) - p).lpNorm<1>() > epsilon)  // manhattan distance between the two points
    {
        p_u = p - distoFunction(_distortionParams, p_u);
    }

    return p_u;
}

Eigen::Matrix2d DistortionBrown::getDerivativeRemoveDistoWrtPt(const Vec2& p) const
{
    ALICEVISION_THROW_ERROR("Brown inverse jacobian are not implemented");
    return Eigen::Matrix2d::Identity();
}

Eigen::MatrixXd DistortionBrown::getDerivativeRemoveDistoWrtDisto(const Vec2& p) const
{
    ALICEVISION_THROW_ERROR("Brown inverse jacobian are not implemented");
    return Eigen::Matrix<double, 2, 5>::Zero();
}

}  // namespace camera
}  // namespace aliceVision
