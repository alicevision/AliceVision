// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "DistortionFisheye1.hpp"

namespace aliceVision {
namespace camera {

Vec2 DistortionFisheye1::addDistortion(const Vec2& p) const
{
    const double eps = 1e-8;
    const double& k1 = _distortionParams.at(0);
    const double r = sqrt(p(0) * p(0) + p(1) * p(1));
    if (k1 * r < eps)
    {
        return p;
    }

    const double coef = (std::atan(2.0 * r * std::tan(0.5 * k1)) / k1) / r;

    return p * coef;
}

Eigen::Matrix2d DistortionFisheye1::getDerivativeAddDistoWrtPt(const Vec2& p) const
{
    const double& k1 = _distortionParams.at(0);
    const double eps = 1e-8;
    const double r = sqrt(p(0) * p(0) + p(1) * p(1));
    if (k1 * r < eps)
    {
        return Eigen::Matrix2d::Identity();
    }

    Eigen::Matrix<double, 1, 2> d_r_d_p;
    d_r_d_p(0) = p(0) / r;
    d_r_d_p(1) = p(1) / r;

    const double part1 = 2.0 * r * std::tan(0.5 * k1);
    const double part2 = std::atan(part1);
    const double part3 = k1 * r;
    const double coef = part2 / part3;

    double d_part3_d_r = k1;
    double d_part2_d_part1 = 1.0 / (part1 * part1 + 1.0);
    double d_part1_d_r = 2.0 * std::tan(0.5 * k1);

    double d_coef_d_part2 = 1.0 / part3;
    double d_coef_d_part3 = -part2 / (part3 * part3);
    double d_coef_d_r = d_coef_d_part2 * d_part2_d_part1 * d_part1_d_r + d_coef_d_part3 * d_part3_d_r;
    Eigen::Matrix<double, 1, 2> d_coef_d_p = d_coef_d_r * d_r_d_p;

    return Eigen::Matrix2d::Identity() * coef + p * d_coef_d_p;
}

Eigen::MatrixXd DistortionFisheye1::getDerivativeAddDistoWrtDisto(const Vec2& p) const
{
    const double& k1 = _distortionParams.at(0);
    const double eps = 1e-21;
    const double r = sqrt(p(0) * p(0) + p(1) * p(1));
    if (k1 * r < eps)
    {
        return Eigen::Matrix<double, 2, 1>::Zero();
    }

    /* const double eps = 1e-8;
    const double& k1 = _distortionParams.at(0);
    const double r = sqrt(p(0) * p(0) + p(1) * p(1));
    if (k1 * r < eps)
    {
        return p;
    }

    const double coef = (std::atan(2.0 * r * std::tan(0.5 * k1)) / k1) / r;

    return p * coef;
}*/

    const double part1 = 2.0 * r * std::tan(0.5 * k1);
    const double part2 = std::atan(part1);
    const double part3 = k1 * r;
    const double coef = part2 / part3;

    double ca = cos(0.5 * k1);
    double d_part1_d_params = r / (ca * ca);
    double d_part3_d_params = r;
    double d_part2_d_part1 = 1.0 / (part1 * part1 + 1.0);
    double d_coef_d_part2 = 1.0 / part3;
    double d_coef_d_part3 = -part2 / (part3 * part3);

    double d_coef_d_params = d_coef_d_part3 * d_part3_d_params + d_coef_d_part2 * d_part2_d_part1 * d_part1_d_params;

    return p * d_coef_d_params;
}

Vec2 DistortionFisheye1::removeDistortion(const Vec2& p) const
{
    const double eps = 1e-8;
    const double& k1 = _distortionParams.at(0);
    const double r = std::hypot(p(0), p(1));
    if (k1 * r < eps)
    {
        return p;
    }

    const double coef = 0.5 * std::tan(r * k1) / (std::tan(0.5 * k1) * r);
    return p * coef;
}

Eigen::Matrix2d DistortionFisheye1::getDerivativeRemoveDistoWrtPt(const Vec2& p) const
{
    const double& k1 = _distortionParams.at(0);
    const double eps = 1e-8;
    const double r = sqrt(p(0) * p(0) + p(1) * p(1));
    if (k1 * r < eps)
    {
        return Eigen::Matrix2d::Identity();
    }

    const Vec2 undist = removeDistortion(p);

    const Eigen::Matrix2d Jinv = getDerivativeAddDistoWrtPt(undist);

    return Jinv.inverse();
}

Eigen::MatrixXd DistortionFisheye1::getDerivativeRemoveDistoWrtDisto(const Vec2& p) const
{
    const double& k1 = _distortionParams.at(0);
    double r_dist = sqrt(p(0) * p(0) + p(1) * p(1));
    const double eps = 1e-8;
    if (k1 * r_dist < eps)
    {
        return Eigen::Matrix<double, 2, 1>::Zero();
    }

    const Vec2 p_undist = removeDistortion(p);
    const double r = sqrt(p_undist(0) * p_undist(0) + p_undist(1) * p_undist(1));

    const double part1 = 2.0 * r * std::tan(0.5 * k1);
    const double part2 = std::atan(part1);
    const double part3 = k1 * r;
    const double coef = part2 / part3;

    double ca = cos(0.5 * k1);
    double d_part1_d_params = r / (ca * ca);
    double d_part3_d_params = r;
    double d_part2_d_part1 = 1.0 / (part1 * part1 + 1.0);
    double d_coef_d_part2 = (part2 * part2) / part3;
    double d_coef_d_part3 = -part2 / (part3 * part3);
    double d_coef_d_params = d_coef_d_part3 * d_part3_d_params + d_coef_d_part2 * d_part2_d_part1 * d_part1_d_params;

    // p'/coef
    Eigen::Matrix<double, 2, 1> ret;
    ret(0, 0) = -p(0) * d_coef_d_params / (coef * coef);
    ret(1, 0) = -p(1) * d_coef_d_params / (coef * coef);

    return ret;
}

}  // namespace camera
}  // namespace aliceVision
