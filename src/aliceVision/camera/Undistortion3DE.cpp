// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Undistortion3DE.hpp"

namespace aliceVision {
namespace camera {

Vec2 Undistortion3DEAnamorphic4::undistortNormalized(const Vec2& p) const
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
    const double& phi = _undistortionParams[10];
    const double& sqx = _undistortionParams[11];
    const double& sqy = _undistortionParams[12];

    const double cphi = std::cos(phi);
    const double sphi = std::sin(phi);

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

    const double& x = p.x();
    const double& y = p.y();

    // First rotate axis
    const double xr = cphi * x + sphi * y;
    const double yr = -sphi * x + cphi * y;

    const double xx = xr * xr;
    const double xxxx = xx * xx;
    const double yy = yr * yr;
    const double yyyy = yy * yy;
    const double xxyy = xx * yy;

    // Compute dist
    const double xd = xr * (1.0 + xx * cx_xx + yy * cx_yy + xxxx * cx_xxxx + xxyy * cx_xxyy + yyyy * cx_yyyy);
    const double yd = yr * (1.0 + xx * cy_xx + yy * cy_yy + xxxx * cy_xxxx + xxyy * cy_xxyy + yyyy * cy_yyyy);

    // Squeeze axis
    const double squizzed_x = xd * sqx;
    const double squizzed_y = yd * sqy;

    // Unrotate axis
    Vec2 np;
    np.x() = cphi * squizzed_x - sphi * squizzed_y;
    np.y() = sphi * squizzed_x + cphi * squizzed_y;

    return np;
}

Eigen::Matrix<double, 2, 2> Undistortion3DEAnamorphic4::getDerivativeUndistortNormalizedwrtPoint(const Vec2& p) const
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
    const double& phi = _undistortionParams[10];
    const double& sqx = _undistortionParams[11];
    const double& sqy = _undistortionParams[12];

    const double cphi = std::cos(phi);
    const double sphi = std::sin(phi);

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

    const double& x = p.x();
    const double& y = p.y();

    // First rotate axis
    const double xr = cphi * x + sphi * y;
    const double yr = -sphi * x + cphi * y;

    const double xx = xr * xr;
    const double yy = yr * yr;
    const double xy = xr * yr;
    const double xxxx = xx * xx;
    const double yyyy = yy * yy;
    const double xxyy = xx * yy;
    const double xxxy = xx * xy;
    const double xyyy = xy * xy;

    // Compute dist
    const double xd = xr * (1.0 + xx * cx_xx + yy * cx_yy + xxxx * cx_xxxx + xxyy * cx_xxyy + yyyy * cx_yyyy);
    const double yd = yr * (1.0 + xx * cy_xx + yy * cy_yy + xxxx * cy_xxxx + xxyy * cy_xxyy + yyyy * cy_yyyy);

    // Squeeze axis
    const double squizzed_x = xd * sqx;
    const double squizzed_y = yd * sqy;

    Eigen::Matrix2d d_np_d_squizzed;
    d_np_d_squizzed(0, 0) = cphi;
    d_np_d_squizzed(0, 1) = -sphi;
    d_np_d_squizzed(1, 0) = sphi;
    d_np_d_squizzed(1, 1) = cphi;

    Eigen::Matrix2d d_squizzed_d_d;
    d_squizzed_d_d(0, 0) = sqx;
    d_squizzed_d_d(0, 1) = 0;
    d_squizzed_d_d(1, 0) = 0;
    d_squizzed_d_d(1, 1) = sqy;

    Eigen::Matrix2d d_d_d_r;
    d_d_d_r(0, 0) = 1.0 + 3.0 * xx * cx_xx + yy * cx_yy + 5.0 * xxxx * cx_xxxx + 3.0 * xxyy * cx_xxyy + yyyy * cx_yyyy;
    d_d_d_r(0, 1) = 2.0 * xy * cx_yy + 2.0 * xxxy * cx_xxyy + 4.0 * xyyy * cx_yyyy;
    d_d_d_r(1, 0) = 2.0 * xy * cy_xx + 4.0 * xxxy * cy_xxxx + 2.0 * xyyy * cy_xxyy;
    d_d_d_r(1, 1) = 1.0 + xx * cy_xx + 3.0 * yy * cy_yy + xxxx * cy_xxxx + 3.0 * xxyy * cy_xxyy + 5.0 * yyyy * cy_yyyy;

    Eigen::Matrix2d d_r_d_p;
    d_r_d_p(0, 0) = 1;
    d_r_d_p(0, 1) = 0;
    d_r_d_p(1, 0) = 0;
    d_r_d_p(1, 1) = 1;

    return d_np_d_squizzed * d_squizzed_d_d * d_d_d_r * d_r_d_p;
}

Eigen::Matrix<double, 2, Eigen::Dynamic> Undistortion3DEAnamorphic4::getDerivativeUndistortNormalizedwrtParameters(const Vec2& p) const
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
    const double& phi = _undistortionParams[10];
    const double& sqx = _undistortionParams[11];
    const double& sqy = _undistortionParams[12];

    const double cphi = std::cos(phi);
    const double sphi = std::sin(phi);

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

    const double& x = p.x();
    const double& y = p.y();

    // First rotate axis
    const double xr = cphi * x + sphi * y;
    const double yr = -sphi * x + cphi * y;

    const double xx = xr * xr;
    const double yy = yr * yr;
    const double xy = xr * yr;
    const double xxx = xx * xr;
    const double yyy = yy * yr;
    const double xxy = xx * yr;
    const double xyy = xr * yy;

    const double xxxx = xx * xx;
    const double yyyy = yy * yy;
    const double xxyy = xx * yy;
    const double xxxy = xx * xy;
    const double xyyy = xy * xy;

    const double xxxxx = xxxx * xr;
    const double xxxxy = xxxx * yr;
    const double xxxyy = xxx * yy;
    const double yyyyy = yyyy * yr;
    const double xyyyy = xr * yyyy;
    const double xxyyy = xx * yyy;

    // Compute dist
    const double xd = xr * (1.0 + xx * cx_xx + yy * cx_yy + xxxx * cx_xxxx + xxyy * cx_xxyy + yyyy * cx_yyyy);
    const double yd = yr * (1.0 + xx * cy_xx + yy * cy_yy + xxxx * cy_xxxx + xxyy * cy_xxyy + yyyy * cy_yyyy);

    // Squeeze axis
    const double squizzed_x = xd * sqx;
    const double squizzed_y = yd * sqy;

    Eigen::Matrix2d d_np_d_squizzed;
    d_np_d_squizzed(0, 0) = 1;
    d_np_d_squizzed(0, 1) = 0;
    d_np_d_squizzed(1, 0) = 0;
    d_np_d_squizzed(1, 1) = 1;

    Eigen::Matrix2d d_squizzed_d_d;
    d_squizzed_d_d(0, 0) = sqx;
    d_squizzed_d_d(0, 1) = 0;
    d_squizzed_d_d(1, 0) = 0;
    d_squizzed_d_d(1, 1) = sqy;

    Eigen::Matrix<double, 2, 14> d_squizzed_d_disto = Eigen::Matrix<double, 2, 14>::Zero();
    d_squizzed_d_disto(0, 11) = xd;
    d_squizzed_d_disto(0, 12) = 0;
    d_squizzed_d_disto(1, 11) = 0;
    d_squizzed_d_disto(1, 12) = yd;

    Eigen::Matrix2d d_d_d_r;
    d_d_d_r(0, 0) = 1.0 + 3.0 * xx * cx_xx + yy * cx_yy + 5.0 * xxxx * cx_xxxx + 3.0 * xxyy * cx_xxyy + yyyy * cx_yyyy;
    d_d_d_r(0, 1) = 2.0 * xy * cx_yy + 2.0 * xxxy * cx_xxyy + 4.0 * xyyy * cx_yyyy;
    d_d_d_r(1, 0) = 2.0 * xy * cy_xx + 4.0 * xxxy * cy_xxxx + 2.0 * xyyy * cy_xxyy;
    d_d_d_r(1, 1) = 1.0 + xx * cy_xx + 3.0 * yy * cy_yy + xxxx * cy_xxxx + 3.0 * xxyy * cy_xxyy + 5.0 * yyyy * cy_yyyy;

    Eigen::Matrix<double, 2, 10> d_d_d_distop = Eigen::Matrix<double, 2, 10>::Zero();
    d_d_d_distop(0, 0) = xxx;
    d_d_d_distop(0, 1) = xyy;
    d_d_d_distop(0, 2) = xxxxx;
    d_d_d_distop(0, 3) = xxxyy;
    d_d_d_distop(0, 4) = xyyyy;
    d_d_d_distop(1, 5) = xxy;
    d_d_d_distop(1, 6) = yyy;
    d_d_d_distop(1, 7) = xxxxy;
    d_d_d_distop(1, 8) = xxyyy;
    d_d_d_distop(1, 9) = yyyyy;

    Eigen::Matrix<double, 10, 14> d_distop_d_disto = Eigen::Matrix<double, 10, 14>::Zero();
    d_distop_d_disto(0, 0) = 1.0;
    d_distop_d_disto(0, 2) = 1.0;
    d_distop_d_disto(1, 0) = 1.0;
    d_distop_d_disto(1, 2) = -1.0;
    d_distop_d_disto(2, 4) = 1.0;
    d_distop_d_disto(2, 6) = 1.0;
    d_distop_d_disto(2, 8) = 1.0;
    d_distop_d_disto(3, 4) = 2.0;
    d_distop_d_disto(3, 8) = -6.0;
    d_distop_d_disto(4, 4) = 1.0;
    d_distop_d_disto(4, 6) = -1.0;
    d_distop_d_disto(4, 8) = 1.0;

    d_distop_d_disto(5, 1) = 1.0;
    d_distop_d_disto(5, 3) = 1.0;
    d_distop_d_disto(6, 1) = 1.0;
    d_distop_d_disto(6, 3) = -1.0;
    d_distop_d_disto(7, 5) = 2.0;
    d_distop_d_disto(7, 9) = -6.0;
    d_distop_d_disto(8, 5) = 1.0;
    d_distop_d_disto(8, 7) = 1.0;
    d_distop_d_disto(8, 9) = 1.0;
    d_distop_d_disto(9, 5) = 1.0;
    d_distop_d_disto(9, 7) = -1.0;
    d_distop_d_disto(9, 9) = 1.0;

    Eigen::Matrix<double, 2, 14> J = (d_np_d_squizzed * d_squizzed_d_disto) + (d_np_d_squizzed * d_squizzed_d_d * d_d_d_distop * d_distop_d_disto);

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
