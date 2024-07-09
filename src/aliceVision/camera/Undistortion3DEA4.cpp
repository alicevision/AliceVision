// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Undistortion3DEA4.hpp"

namespace aliceVision {
namespace camera {

/**
 * theta, r = polar coordinates of x, y
 * x = cos(theta) * r
 * y = sin(theta) * r
 * r2 = r * r
 * r4 = r2 * r2
 * 
 * 3DE equation : 
 * xd = xu * (1.0 + cx02 * r2 + cx04 * r4 + cx22 * r2 * cos_2theta + cx24 * r4 * cos_2theta + cx44 * r4 * cos_4theta)
 * yd = yu * (1.0 + cy02 * r2 + cy04 * r4 + cy22 * r2 * cos_2theta + cy24 * r4 * cos_2theta + cy44 * r4 * cos_4theta)
 * 
 * cos2_theta = cos(theta) * cos(theta)
 * cos(theta) = x / r
 * sin(theta) = y / r
 * cos2_theta = x*x / r2
 * sin2_theta = y*y / r2
 * cos_2theta = cos2_theta - sin2_theta
 * cos_4theta = cos4_theta - 6 * cos2_theta * sin2_theta + sin4_theta
*/

Vec2 Undistortion3DEAnamorphic4::undistortNormalizedBase(const Vec2& p) const
{
    const double& cx02 = _undistortionParams[0];
    const double& cy02 = _undistortionParams[1];
    const double& cx22 = _undistortionParams[2];
    const double& cy22 = _undistortionParams[3];
    const double& cx04 = _undistortionParams[4];
    const double& cy04 = _undistortionParams[5];
    const double& cx24 = _undistortionParams[6];
    const double& cy24 = _undistortionParams[7];
    const double& cx44 = _undistortionParams[8];
    const double& cy44 = _undistortionParams[9];

    const double& xu = p.x();
    const double& yu = p.y();

    const double r2 = xu * xu + yu * yu;
    const double r4 = r2 * r2;
    const double theta = std::atan2(yu, xu);
    const double cos_2theta = cos(2.0 * theta);
    const double cos_4theta = cos(4.0 * theta);
   
    // Compute dist
    Vec2 np;
    np.x() = xu * (1.0 + cx02 * r2 + cx04 * r4 + cx22 * r2 * cos_2theta + cx24 * r4 * cos_2theta + cx44 * r4 * cos_4theta);
    np.y() = yu * (1.0 + cy02 * r2 + cy04 * r4 + cy22 * r2 * cos_2theta + cy24 * r4 * cos_2theta + cy44 * r4 * cos_4theta);

    return np;
}

Vec2 Undistortion3DEAnamorphic4::undistortNormalized(const Vec2& p) const
{
    const double& phi = _undistortionParams[10];
    const double& sqx = _undistortionParams[11];
    const double& sqy = _undistortionParams[12];
    const double& pa = _pixelAspectRatio;

    const double xu = p.x() / pa;
    const double yu = p.y();

    const double cphi = cos(phi);
    const double sphi = sin(phi);

    Vec2 rotated;
    rotated.x() = cphi * xu - sphi * yu;
    rotated.y() = sphi * xu + cphi * yu;

    const Vec2 ptu = undistortNormalizedBase(rotated);

    Vec2 squeezed;
    squeezed.x() = ptu.x() * pa * sqx;
    squeezed.y() = ptu.y() * sqy;

    Vec2 np;
    np.x() = cphi * squeezed.x() + sphi * squeezed.y();
    np.y() = - sphi * squeezed.x() + cphi * squeezed.y();

    return np;
}

Eigen::Matrix<double, 2, 2> Undistortion3DEAnamorphic4::getDerivativeUndistortNormalizedwrtPointBase(const Vec2& p) const
{
    const double& cx02 = _undistortionParams[0];
    const double& cy02 = _undistortionParams[1];
    const double& cx22 = _undistortionParams[2];
    const double& cy22 = _undistortionParams[3];
    const double& cx04 = _undistortionParams[4];
    const double& cy04 = _undistortionParams[5];
    const double& cx24 = _undistortionParams[6];
    const double& cy24 = _undistortionParams[7];
    const double& cx44 = _undistortionParams[8];
    const double& cy44 = _undistortionParams[9];

    const double cx_xx = cx02 + cx22;
    const double cx_yy = cx02 - cx22;
    const double cx_xxyy = 2 * cx04 - 6 * cx44;
    const double cx_xxxx = cx04 + cx24 + cx44;
    const double cx_yyyy = cx04 - cx24 + cx44;

    const double cy_xx = cy02 + cy22;
    const double cy_yy = cy02 - cy22;
    const double cy_xxyy = 2 * cy04 - 6 * cy44;
    const double cy_xxxx = cy04 + cy24 + cy44;
    const double cy_yyyy = cy04 - cy24 + cy44;

    const double& xu = p.x();
    const double& yu = p.y();

    const double xx = xu * xu;
    const double yy = yu * yu;
    const double xy = xu * yu;
    const double xxxx = xx * xx;
    const double yyyy = yy * yy;
    const double xxyy = xx * yy;
    const double xxxy = xx * xy;
    const double xyyy = xy * xy;

    const double distx = 1.0 + xx * cx_xx + yy * cx_yy + xxxx * cx_xxxx + xxyy * cx_xxyy + yyyy * cx_yyyy;
    const double disty = 1.0 + xx * cy_xx + yy * cy_yy + xxxx * cy_xxxx + xxyy * cy_xxyy + yyyy * cy_yyyy;

    // Compute dist
    Vec2 np;
    np.x() = xu * distx;
    np.y() = yu * disty;

    const double d_xx_d_x = 2.0 * xu;
    const double d_xxxx_d_x = 4.0 * xu * xx; 
    const double d_xxyy_d_x = 2.0 * xu * yy; 
    const double d_yy_d_y = 2.0 * yu;
    const double d_yyyy_d_y = 4.0 * yu * yy;
    const double d_xxyy_d_y = 2.0 * yu * xx;

    const double d_distx_d_x = cx_xx * d_xx_d_x + cx_xxxx * d_xxxx_d_x + cx_xxyy * d_xxyy_d_x; 
    const double d_distx_d_y = cx_yy * d_yy_d_y + cx_xxyy * d_xxyy_d_y + cx_yyyy * d_yyyy_d_y;
    const double d_disty_d_x = cy_xx * d_xx_d_x + cy_xxxx * d_xxxx_d_x + cy_xxyy * d_xxyy_d_x; 
    const double d_disty_d_y = cy_yy * d_yy_d_y + cy_xxyy * d_xxyy_d_y + cy_yyyy * d_yyyy_d_y;

    Eigen::Matrix2d d_np_d_ptu;
    d_np_d_ptu(0, 0) = distx + xu * d_distx_d_x;
    d_np_d_ptu(0, 1) = xu * d_distx_d_y;
    d_np_d_ptu(1, 0) = yu * d_disty_d_x;
    d_np_d_ptu(1, 1) = disty + yu * d_disty_d_y;

    return d_np_d_ptu;
}

Eigen::Matrix<double, 2, 2> Undistortion3DEAnamorphic4::getDerivativeUndistortNormalizedwrtPoint(const Vec2& p) const
{
    const double& phi = _undistortionParams[10];
    const double& sqx = _undistortionParams[11];
    const double& sqy = _undistortionParams[12];
    const double& pa = _pixelAspectRatio;

    const double cphi = cos(phi);
    const double sphi = sin(phi);

    const double xu = p.x() / pa;
    const double yu = p.y();

    Eigen::Matrix2d d_pt_d_p;
    d_pt_d_p(0, 0) = 1.0 / pa;
    d_pt_d_p(0, 1) = 0.0;
    d_pt_d_p(1, 0) = 0.0;
    d_pt_d_p(1, 1) = 1.0;

    Vec2 rotated;
    rotated.x() = cphi * xu - sphi * yu;
    rotated.y() = sphi * xu + cphi * yu;
    
    Eigen::Matrix2d d_rotated_d_pt;
    d_rotated_d_pt(0, 0) = cphi;
    d_rotated_d_pt(0, 1) = -sphi;
    d_rotated_d_pt(1, 0) = sphi;
    d_rotated_d_pt(1, 1) = cphi;

    const Vec2 ptu = undistortNormalizedBase(rotated);

    Vec2 squeezed;
    squeezed.x() = ptu.x() * pa * sqx;
    squeezed.y() = ptu.y() * sqy;

    Eigen::Matrix2d d_squeezed_d_ptu;
    d_squeezed_d_ptu(0, 0) = pa * sqx;
    d_squeezed_d_ptu(0, 1) = 0.0;
    d_squeezed_d_ptu(1, 0) = 0.0;
    d_squeezed_d_ptu(1, 1) = sqy;

    Vec2 np;
    np.x() = cphi * squeezed.x() + sphi * squeezed.y();
    np.y() = - sphi * squeezed.x() + cphi * squeezed.y();

    Eigen::Matrix2d d_np_d_squeezed;
    d_np_d_squeezed(0, 0) = cphi;
    d_np_d_squeezed(0, 1) = sphi;
    d_np_d_squeezed(1, 0) = -sphi;
    d_np_d_squeezed(1, 1) = cphi;

    return d_np_d_squeezed * d_squeezed_d_ptu * getDerivativeUndistortNormalizedwrtPointBase(rotated) * d_rotated_d_pt * d_pt_d_p;
}

Eigen::Matrix<double, 2, Eigen::Dynamic> Undistortion3DEAnamorphic4::getDerivativeUndistortNormalizedwrtParametersBase(const Vec2& p) const
{
    const double& cx02 = _undistortionParams[0];
    const double& cy02 = _undistortionParams[1];
    const double& cx22 = _undistortionParams[2];
    const double& cy22 = _undistortionParams[3];
    const double& cx04 = _undistortionParams[4];
    const double& cy04 = _undistortionParams[5];
    const double& cx24 = _undistortionParams[6];
    const double& cy24 = _undistortionParams[7];
    const double& cx44 = _undistortionParams[8];
    const double& cy44 = _undistortionParams[9];

    const double& xu = p.x();
    const double& yu = p.y();

    const double r2 = xu * xu + yu * yu;
    const double r4 = r2 * r2;
    const double theta = std::atan2(yu, xu);
    const double cos_2theta = cos(2.0 * theta);
    const double cos_4theta = cos(4.0 * theta);
   
    // Compute dist
    const double xd = xu * (1.0 + cx02 * r2 + cx04 * r4 + cx22 * r2 * cos_2theta + cx24 * r4 * cos_2theta + cx44 * r4 * cos_4theta);
    const double yd = yu * (1.0 + cy02 * r2 + cx04 * r4 + cx22 * r2 * cos_2theta + cx24 * r4 * cos_2theta + cx44 * r4 * cos_4theta);

    Eigen::Matrix<double, 2, 10> d_ptd_d_params = Eigen::Matrix<double, 2, 10>::Zero();
    d_ptd_d_params(0, 0) = r2 * xu;
    d_ptd_d_params(0, 2) = r2 * cos_2theta * xu;
    d_ptd_d_params(0, 4) = r4 * xu;
    d_ptd_d_params(0, 6) = r4 * cos_2theta * xu;
    d_ptd_d_params(0, 8) = r4 * cos_4theta * xu;
    d_ptd_d_params(1, 1) = r2 * yu;
    d_ptd_d_params(1, 3) = r2 * cos_2theta * yu;
    d_ptd_d_params(1, 5) = r4 * yu;
    d_ptd_d_params(1, 7) = r4 * cos_2theta * yu;
    d_ptd_d_params(1, 9) = r4 * cos_4theta * yu;

    return d_ptd_d_params;
}

Eigen::Matrix<double, 2, Eigen::Dynamic> Undistortion3DEAnamorphic4::getDerivativeUndistortNormalizedwrtParameters(const Vec2& p) const
{
    const double& phi = _undistortionParams[10];
    const double& sqx = _undistortionParams[11];
    const double& sqy = _undistortionParams[12];
    const double& pa = _pixelAspectRatio;

    const double cphi = cos(phi);
    const double sphi = sin(phi);

    const double xu = p.x() / pa;
    const double yu = p.y();

    Vec2 rotated;
    rotated.x() = cphi * xu - sphi * yu;
    rotated.y() = sphi * xu + cphi * yu;

    Eigen::Matrix<double, 2, 1> d_rotated_d_phi;
    d_rotated_d_phi(0, 0) = xu * (-sphi) + yu * (-cphi);
    d_rotated_d_phi(1, 0) = xu * (cphi) + yu * (-sphi);

    const Vec2 ptu = undistortNormalizedBase(rotated);

    Vec2 squeezed;
    squeezed.x() = ptu.x() * pa * sqx;
    squeezed.y() = ptu.y() * sqy;

    Vec2 np;
    np.x() = cphi * squeezed.x() + sphi * squeezed.y();
    np.y() = - sphi * squeezed.x() + cphi * squeezed.y();

    Eigen::Matrix2d d_squeezed_d_params;
    d_squeezed_d_params(0, 0) = pa * ptu.x();
    d_squeezed_d_params(0, 1) = 0.0;
    d_squeezed_d_params(1, 0) = 0.0;
    d_squeezed_d_params(1, 1) = ptu.y();

    Eigen::Matrix2d d_squeezed_d_ptu;
    d_squeezed_d_ptu(0, 0) = pa * sqx;
    d_squeezed_d_ptu(0, 1) = 0.0;
    d_squeezed_d_ptu(1, 0) = 0.0;
    d_squeezed_d_ptu(1, 1) = sqy;

    Eigen::Matrix2d d_np_d_squeezed;
    d_np_d_squeezed(0, 0) = cphi;
    d_np_d_squeezed(0, 1) = sphi;
    d_np_d_squeezed(1, 0) = -sphi;
    d_np_d_squeezed(1, 1) = cphi;

    Eigen::Matrix<double, 2, 1> d_np_d_phi;
    d_np_d_phi(0, 0) = squeezed.x() * (-sphi) + squeezed.y() * (cphi);
    d_np_d_phi(1, 0) = squeezed.x() * (-cphi) + squeezed.y() * (-sphi);
    

    Eigen::Matrix<double, 2, 13> J = Eigen::Matrix<double, 2, 13>::Zero();

    J.block<2, 10>(0, 0) = d_np_d_squeezed * d_squeezed_d_ptu * getDerivativeUndistortNormalizedwrtParametersBase(rotated);
    J.block<2, 1>(0, 10) = d_np_d_phi + d_np_d_squeezed * d_squeezed_d_ptu * getDerivativeUndistortNormalizedwrtPointBase(rotated) * d_rotated_d_phi;
    J.block<2, 2>(0, 11) =  d_np_d_squeezed * d_squeezed_d_params;
    
    return J;
}

Vec2 Undistortion3DEAnamorphic4::inverseNormalized(const Vec2& p) const
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