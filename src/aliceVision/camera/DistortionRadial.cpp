// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "DistortionRadial.hpp"

namespace aliceVision {
namespace camera {

double DistortionRadialK1::distoFunctor(const std::vector<double> & params, double r2)
{
    const double& k1 = params[0];
    return r2 * Square(1.+r2*k1);
}

Vec2 DistortionRadialK1::addDistortion(const Vec2 & p) const
{
    const double& k1 = _distortionParams.at(0);

    const double r2 = p(0)*p(0) + p(1)*p(1);
    const double r_coeff = (1. + k1*r2);

    return (p * r_coeff);
}

Eigen::Matrix2d DistortionRadialK1::getDerivativeAddDistoWrtPt(const Vec2 & p) const
{
    const double k1 = _distortionParams[0];

    const double r = sqrt(p(0)*p(0) + p(1)*p(1));
    const double eps = 1e-8;
    if (r < eps) {
        return Eigen::Matrix2d::Identity();
    }

    Eigen::Matrix<double, 1, 2> d_r_d_p;
    d_r_d_p(0) = p(0) / r;
    d_r_d_p(1) = p(1) / r;

    const double r2 = r * r;
    const double r_coeff = 1.0 + k1 * r2;

    const double d_r_coeff_d_r = 2.0 * k1 * r;
    const Eigen::Matrix<double, 1, 2> d_r_coeff_d_p = d_r_coeff_d_r * d_r_d_p;

    return Eigen::Matrix2d::Identity() * r_coeff + p * d_r_coeff_d_p;
}

Eigen::MatrixXd DistortionRadialK1::getDerivativeAddDistoWrtDisto(const Vec2 & p) const
{
    const double& k1 = _distortionParams[0];

    const double r = sqrt(p(0)*p(0) + p(1)*p(1));
    const double eps = 1e-8;
    if (r < eps)
    {
        return Eigen::Matrix<double, 2, 1>::Zero();
    }

    const Eigen::MatrixXd ret = p * r * r;

    return ret;
}

Vec2 DistortionRadialK1::removeDistortion(const Vec2& p) const
{
    // Compute the radius from which the point p comes from thanks to a bisection
    // Minimize disto(radius(p')^2) == actual Squared(radius(p))

    const double r2 = p(0)*p(0) + p(1)*p(1);
    const double radius = (r2 == 0) ? 1. : ::sqrt(radial_distortion::bisection_Radius_Solve(_distortionParams, r2, distoFunctor) / r2);
    return radius * p;
}

Eigen::Matrix2d DistortionRadialK1::getDerivativeRemoveDistoWrtPt(const Vec2 & p) const
{
    const double r = sqrt(p(0)*p(0) + p(1)*p(1));
    const double eps = 1e-8;
    if (r < eps)
    {
        return Eigen::Matrix2d::Identity();
    }

    const Vec2 undist = removeDistortion(p);

    const Eigen::Matrix2d Jinv = getDerivativeAddDistoWrtPt(undist);

    return Jinv.inverse();
}

Eigen::MatrixXd DistortionRadialK1::getDerivativeRemoveDistoWrtDisto(const Vec2 & p) const
{
    double r_dist = sqrt(p(0)*p(0) + p(1)*p(1));
    const double eps = 1e-8;
    if (r_dist < eps)
    {
        return Eigen::Matrix<double, 2, 1>::Zero();
    }

    const Vec2 p_undist = removeDistortion(p);

    const double& k1 = _distortionParams[0];

    const double r = sqrt(p_undist(0)*p_undist(0) + p_undist(1)*p_undist(1));
    const double r2 = r * r;

    const double r_coeff = 1.0 + k1 * r2;

    Eigen::Matrix<double, 1, 1> d_rcoeff_d_params;
    d_rcoeff_d_params(0, 0) = r2;

    Eigen::Matrix<double, 2, 1> ret;
    ret(0, 0) = - (p(0) * d_rcoeff_d_params(0, 0)) / (r_coeff * r_coeff);
    ret(1, 0) = - (p(1) * d_rcoeff_d_params(0, 0)) / (r_coeff * r_coeff);

    return ret;
}

double DistortionRadialK1::getUndistortedRadius(double r) const
{
    return std::sqrt(radial_distortion::bisection_Radius_Solve(_distortionParams, r * r, distoFunctor));
}

double DistortionRadialK3::distoFunctor(const std::vector<double> & params, double r2)
{
    const double k1 = params[0], k2 = params[1], k3 = params[2];
    return r2 * Square(1.+r2*(k1+r2*(k2+r2*k3)));
}

Vec2 DistortionRadialK3::addDistortion(const Vec2 & p) const
{
    const double& k1 = _distortionParams[0];
    const double& k2 = _distortionParams[1];
    const double& k3 = _distortionParams[2];

    const double r = sqrt(p(0)*p(0) + p(1)*p(1));

    const double r2 = r * r;
    const double r4 = r2 * r2;
    const double r6 = r4 * r2;
    const double r_coeff = (1. + k1 * r2 + k2 * r4 + k3 * r6);

    return (p * r_coeff);
}

Eigen::Matrix2d DistortionRadialK3::getDerivativeAddDistoWrtPt(const Vec2 & p) const
{
    const double& k1 = _distortionParams[0];
    const double& k2 = _distortionParams[1];
    const double& k3 = _distortionParams[2];

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

Eigen::MatrixXd DistortionRadialK3::getDerivativeAddDistoWrtDisto(const Vec2 & p) const
{
    const double r2 = p(0)*p(0) + p(1)*p(1);
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

Vec2 DistortionRadialK3::removeDistortion(const Vec2& p) const
{
    // Compute the radius from which the point p comes from thanks to a bisection
    // Minimize disto(radius(p')^2) == actual Squared(radius(p))

    const double r2 = p(0)*p(0) + p(1)*p(1);
    const double radius = (r2 == 0) ? //1. : ::sqrt(bisectionSolve(_distortionParams, r2) / r2);
        1. :
        ::sqrt(radial_distortion::bisection_Radius_Solve(_distortionParams, r2, distoFunctor) / r2);
    return radius * p;
}

Eigen::Matrix2d DistortionRadialK3::getDerivativeRemoveDistoWrtPt(const Vec2 & p) const
{
    const double r = sqrt(p(0)*p(0) + p(1)*p(1));
    const double eps = 1e-8;
    if (r < eps) {
        return Eigen::Matrix2d::Identity();
    }

    const Vec2 undist = removeDistortion(p);

    const Eigen::Matrix2d Jinv = getDerivativeAddDistoWrtPt(undist);

    return Jinv.inverse();
}

Eigen::MatrixXd DistortionRadialK3::getDerivativeRemoveDistoWrtDisto(const Vec2 & p) const
{
    double r_dist = sqrt(p(0)*p(0) + p(1)*p(1));
    const double eps = 1e-8;
    if (r_dist < eps) {
        return Eigen::Matrix<double, 2, 3>::Zero();
    }

    Vec2 p_undist = removeDistortion(p);

    const double& k1 = _distortionParams[0];
    const double& k2 = _distortionParams[1];
    const double& k3 = _distortionParams[2];

    const double r = sqrt(p_undist(0) * p_undist(0) + p_undist(1) * p_undist(1));
    const double r2 = r * r;
    const double r4 = r2 * r2;
    const double r6 = r4 * r2;

    const double r_coeff = 1.0 + k1 * r2 + k2 * r4 + k3 * r6;

    Eigen::Matrix<double, 1, 3> d_rcoeff_d_params;
    d_rcoeff_d_params(0, 0) = r2;
    d_rcoeff_d_params(0, 1) = r4;
    d_rcoeff_d_params(0, 2) = r6;

    Eigen::Matrix<double, 2, 3> ret;
    ret(0, 0) = - (p(0) * d_rcoeff_d_params(0, 0)) / (r_coeff * r_coeff);
    ret(0, 1) = - (p(0) * d_rcoeff_d_params(0, 1)) / (r_coeff * r_coeff);
    ret(0, 2) = - (p(0) * d_rcoeff_d_params(0, 2)) / (r_coeff * r_coeff);
    ret(1, 0) = - (p(1) * d_rcoeff_d_params(0, 0)) / (r_coeff * r_coeff);
    ret(1, 1) = - (p(1) * d_rcoeff_d_params(0, 1)) / (r_coeff * r_coeff);
    ret(1, 2) = - (p(1) * d_rcoeff_d_params(0, 2)) / (r_coeff * r_coeff);

    return ret;
}

double DistortionRadialK3::getUndistortedRadius(double r) const
{
    return std::sqrt(radial_distortion::bisection_Radius_Solve(_distortionParams, r * r, distoFunctor));
}

double DistortionRadialK3PT::distoFunctor(const std::vector<double> & params, double r2)
{
    const double& k1 = params[0];
    const double& k2 = params[1];
    const double& k3 = params[2];

    const double r4 = r2 * r2;
    const double r6 = r4 * r2;

    const double r_coeff = (1.0 + k1*r2 + k2*r4 + k3*r6) / (1.0 + k1 + k2 + k3);

    return r2 * Square(r_coeff);
}

Vec2 DistortionRadialK3PT::addDistortion(const Vec2 & p) const
{
    const double& k1 = _distortionParams[0];
    const double& k2 = _distortionParams[1];
    const double& k3 = _distortionParams[2];

    const double r = sqrt(p(0)*p(0) + p(1)*p(1));

    const double r2 = r * r;
    const double r4 = r2 * r2;
    const double r6 = r4 * r2;

    const double r_coeff = (1.0 + k1*r2 + k2*r4 + k3*r6) / (1.0 + k1 + k2 + k3);

    return (p * r_coeff);
}

Eigen::Matrix2d DistortionRadialK3PT::getDerivativeAddDistoWrtPt(const Vec2 & p) const
{
    const double& k1 = _distortionParams[0];
    const double& k2 = _distortionParams[1];
    const double& k3 = _distortionParams[2];

    const double r = sqrt(p(0)*p(0) + p(1)*p(1));
    if (r < 1e-12) {
        return Eigen::Matrix2d::Identity();
    }

    Eigen::Matrix<double, 1, 2> d_r_d_p;
    d_r_d_p(0) = p(0) / r;
    d_r_d_p(1) = p(1) / r;

    const double r2 = r * r;
    const double r3 = r2 * r;
    const double r4 = r2 * r2;
    const double r5 = r4 * r;
    const double r6 = r4 * r2;

    const double r_coeff = (1.0 + k1*r2 + k2*r4 + k3*r6) / (1.0 + k1 + k2 + k3);

    double d_r_coeff_d_r = (2.0 * k1 * r + 4.0 * k2 * r3 + 6.0 * k3 * r5) / (1.0 + k1 + k2 + k3);
    Eigen::Matrix<double, 1, 2> d_r_coeff_d_p = d_r_coeff_d_r * d_r_d_p;

    return Eigen::Matrix2d::Identity() * r_coeff + p * d_r_coeff_d_p;
}

Eigen::MatrixXd DistortionRadialK3PT::getDerivativeAddDistoWrtDisto(const Vec2 & p) const
{
    const double& k1 = _distortionParams[0];
    const double& k2 = _distortionParams[1];
    const double& k3 = _distortionParams[2];

    const double r = sqrt(p(0)*p(0) + p(1)*p(1));
    const double eps = 1e-8;
    if (r < eps)
    {
        return Eigen::Matrix<double, 2, 3>::Zero();
    }

    const double r2 = r * r;
    const double r4 = r2 * r2;
    const double r6 = r4 * r2;

    const double denum = (1.0 + k1 + k2 + k3);
    const double num = (1.0 + k1*r2 + k2*r4 + k3*r6);
    /*const double r_coeff = num / denum;*/

    const double denum2 = denum * denum;
    Eigen::Matrix<double, 1, 3> d_num_d_params;
    d_num_d_params(0, 0) = r2;
    d_num_d_params(0, 1) = r4;
    d_num_d_params(0, 2) = r6;

    Eigen::Matrix<double, 1, 3> d_denum_d_params;
    d_denum_d_params(0, 0) = 1;
    d_denum_d_params(0, 1) = 1;
    d_denum_d_params(0, 2) = 1;

    Eigen::Matrix<double, 1, 3> d_rcoeff_d_params = (denum * d_num_d_params - num * d_denum_d_params) / denum2;
    Eigen::MatrixXd ret = p * d_rcoeff_d_params;

    return ret;
}

Eigen::Matrix2d DistortionRadialK3PT::getDerivativeRemoveDistoWrtPt(const Vec2 & p) const
{
    const double r_dist = sqrt(p(0)*p(0) + p(1)*p(1));
    const double eps = 1e-8;
    if (r_dist < eps)
    {
        return Eigen::Matrix2d::Identity();
    }

    Vec2 undist = removeDistortion(p);

    Eigen::Matrix2d Jinv = getDerivativeAddDistoWrtPt(undist);

    return Jinv.inverse();
}

Eigen::MatrixXd DistortionRadialK3PT::getDerivativeRemoveDistoWrtDisto(const Vec2 & p) const
{
    const double& k1 = _distortionParams[0];
    const double& k2 = _distortionParams[1];
    const double& k3 = _distortionParams[2];

    const double r_dist = sqrt(p(0)*p(0) + p(1)*p(1));
    const double eps = 1e-8;
    if (r_dist < eps) {
        return Eigen::Matrix<double, 2, 3>::Zero();
    }

    const Vec2 p_undist = removeDistortion(p);
    const double r = sqrt(p_undist(0) * p_undist(0) + p_undist(1) * p_undist(1));

    const double r2 = r * r;
    const double r4 = r2 * r2;
    const double r6 = r4 * r2;

    const double denum = (1.0 + k1 + k2 + k3);
    const double num = (1.0 + k1*r2 + k2*r4 + k3*r6);
    const double r_coeff = num / denum;

    double denum2 = denum * denum;
    Eigen::Matrix<double, 1, 3> d_num_d_params;
    d_num_d_params(0, 0) = r2;
    d_num_d_params(0, 1) = r4;
    d_num_d_params(0, 2) = r6;

    Eigen::Matrix<double, 1, 3> d_denum_d_params;
    d_denum_d_params(0, 0) = 1;
    d_denum_d_params(0, 1) = 1;
    d_denum_d_params(0, 2) = 1;

    Eigen::Matrix<double, 1, 3> d_rcoeff_d_params = (denum * d_num_d_params - num * d_denum_d_params) / denum2;

    Eigen::Matrix<double, 2, 3> ret;
    ret(0, 0) = - (p(0) * d_rcoeff_d_params(0, 0)) / (r_coeff * r_coeff);
    ret(0, 1) = - (p(0) * d_rcoeff_d_params(0, 1)) / (r_coeff * r_coeff);
    ret(0, 2) = - (p(0) * d_rcoeff_d_params(0, 2)) / (r_coeff * r_coeff);
    ret(1, 0) = - (p(1) * d_rcoeff_d_params(0, 0)) / (r_coeff * r_coeff);
    ret(1, 1) = - (p(1) * d_rcoeff_d_params(0, 1)) / (r_coeff * r_coeff);
    ret(1, 2) = - (p(1) * d_rcoeff_d_params(0, 2)) / (r_coeff * r_coeff);

    return ret;
}

Vec2 DistortionRadialK3PT::removeDistortion(const Vec2& p) const
{
    // Compute the radius from which the point p comes from thanks to a bisection
    // Minimize disto(radius(p')^2) == actual Squared(radius(p))

    const double r2 = p(0)*p(0) + p(1)*p(1);
    const double radius = (r2 == 0) ? //1. : ::sqrt(bisectionSolve(_distortionParams, r2) / r2);
        1. :
        ::sqrt(radial_distortion::bisection_Radius_Solve(_distortionParams, r2, distoFunctor, 1e-12) / r2);

    const Vec2 p_undist = radius * p;
    return p_undist;
}

double DistortionRadialK3PT::getUndistortedRadius(double r) const
{
    return std::sqrt(radial_distortion::bisection_Radius_Solve(_distortionParams, r * r, distoFunctor));
}

} // namespace camera
} // namespace aliceVision
