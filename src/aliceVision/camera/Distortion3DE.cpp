// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Distortion3DE.hpp"

namespace aliceVision {
namespace camera {

Vec2 Distortion3DERadial4::addDistortion(const Vec2& p) const
{
    const double& c2 = _distortionParams[0];
    const double& c4 = _distortionParams[1];
    const double& u1 = _distortionParams[2];
    const double& v1 = _distortionParams[3];
    const double& u3 = _distortionParams[4];
    const double& v3 = _distortionParams[5];

    const double& x = p.x();
    const double& y = p.y();

    const double xx = x * x;
    const double yy = y * y;
    const double xy = x * y;
    const double r2 = xx + yy;
    const double r4 = r2 * r2;

    const double p1 = 1.0 + c2 * r2 + c4 * r4;
    const double p2 = r2 + 2.0 * xx;
    const double p3 = r2 + 2.0 * yy;

    const double p4 = u1 + u3 * r2;
    const double p5 = v1 + v3 * r2;
    const double p6 = 2.0 * xy;

    Vec2 np;
    np.x() = x * p1 + p2 * p4 + p6 * p5;
    np.y() = y * p1 + p3 * p5 + p6 * p4;

    return np;
}

Eigen::Matrix2d Distortion3DERadial4::getDerivativeAddDistoWrtPt(const Vec2& p) const
{
    const double& c2 = _distortionParams[0];
    const double& c4 = _distortionParams[1];
    const double& u1 = _distortionParams[2];
    const double& v1 = _distortionParams[3];
    const double& u3 = _distortionParams[4];
    const double& v3 = _distortionParams[5];

    const double& x = p.x();
    const double& y = p.y();

    const double xx = x * x;
    const double yy = y * y;
    const double xy = x * y;
    const double r2 = xx + yy;
    const double r4 = r2 * r2;

    const double eps = 1e-16;
    if (r2 < eps)
    {
        return Eigen::Matrix<double, 2, 2>::Zero();
    }

    const double p1 = 1.0 + c2 * r2 + c4 * r4;
    const double p2 = r2 + 2.0 * xx;
    const double p3 = r2 + 2.0 * yy;

    const double p4 = u1 + u3 * r2;
    const double p5 = v1 + v3 * r2;
    const double p6 = 2.0 * xy;

    Eigen::Matrix<double, 1, 2> d_r2_d_p;
    d_r2_d_p(0, 0) = 2.0 * x;
    d_r2_d_p(0, 1) = 2.0 * y;

    const double d_p1_d_r2 = c2 + 2.0 * c4 * r2;
    const double d_p4_d_r2 = u3;
    const double d_p5_d_r2 = v3;

    Eigen::Matrix<double, 1, 2> d_p1_d_p = d_p1_d_r2 * d_r2_d_p;

    Eigen::Matrix<double, 1, 2> d_p2_d_p;
    d_p2_d_p(0, 0) = d_r2_d_p(0, 0) + 4.0 * x;
    d_p2_d_p(0, 1) = d_r2_d_p(0, 1);

    Eigen::Matrix<double, 1, 2> d_p3_d_p;
    d_p3_d_p(0, 0) = d_r2_d_p(0, 0);
    d_p3_d_p(0, 1) = d_r2_d_p(0, 1) + 4.0 * y;

    Eigen::Matrix<double, 1, 2> d_p4_d_p = d_p4_d_r2 * d_r2_d_p;
    Eigen::Matrix<double, 1, 2> d_p5_d_p = d_p5_d_r2 * d_r2_d_p;

    Eigen::Matrix<double, 1, 2> d_p6_d_p;
    d_p6_d_p(0, 0) = 2.0 * y;
    d_p6_d_p(0, 1) = 2.0 * x;

    Eigen::Matrix<double, 1, 2> d_x_d_p;
    d_x_d_p(0, 0) = 1.0;
    d_x_d_p(0, 1) = 0.0;

    Eigen::Matrix<double, 1, 2> d_y_d_p;
    d_y_d_p(0, 0) = 0.0;
    d_y_d_p(0, 1) = 1.0;

    Eigen::Matrix<double, 2, 2> ret;
    ret.block<1, 2>(0, 0) = (x * d_p1_d_p + d_x_d_p * p1) + (p2 * d_p4_d_p + d_p2_d_p * p4) + (p6 * d_p5_d_p + d_p6_d_p * p5);
    ret.block<1, 2>(1, 0) = (y * d_p1_d_p + d_y_d_p * p1) + (p3 * d_p5_d_p + d_p3_d_p * p5) + (p6 * d_p4_d_p + d_p6_d_p * p4);

    return ret;
}

Eigen::MatrixXd Distortion3DERadial4::getDerivativeAddDistoWrtDisto(const Vec2& p) const
{
    const double& c2 = _distortionParams[0];
    const double& c4 = _distortionParams[1];
    const double& u1 = _distortionParams[2];
    const double& v1 = _distortionParams[3];
    const double& u3 = _distortionParams[4];
    const double& v3 = _distortionParams[5];

    const double& x = p.x();
    const double& y = p.y();

    const double xx = x * x;
    const double yy = y * y;
    const double xy = x * y;
    const double r2 = xx + yy;
    const double r4 = r2 * r2;

    const double eps = 1e-8;
    if (r2 < eps)
    {
        return Eigen::Matrix<double, 2, 6>::Zero();
    }

    const double p1 = 1.0 + c2 * r2 + c4 * r4;
    const double p2 = r2 + 2.0 * xx;
    const double p3 = r2 + 2.0 * yy;

    const double p4 = u1 + u3 * r2;
    const double p5 = v1 + v3 * r2;
    const double p6 = 2.0 * xy;

    Eigen::Matrix<double, 1, 6> d_p1_d_disto;
    d_p1_d_disto(0, 0) = r2;
    d_p1_d_disto(0, 1) = r4;
    d_p1_d_disto(0, 2) = 0;
    d_p1_d_disto(0, 3) = 0;
    d_p1_d_disto(0, 4) = 0;
    d_p1_d_disto(0, 5) = 0;

    Eigen::Matrix<double, 1, 6> d_p4_d_disto;
    d_p4_d_disto(0, 0) = 0;
    d_p4_d_disto(0, 1) = 0;
    d_p4_d_disto(0, 2) = 1.0;
    d_p4_d_disto(0, 3) = 0;
    d_p4_d_disto(0, 4) = r2;
    d_p4_d_disto(0, 5) = 0;

    Eigen::Matrix<double, 1, 6> d_p5_d_disto;
    d_p5_d_disto(0, 0) = 0;
    d_p5_d_disto(0, 1) = 0;
    d_p5_d_disto(0, 2) = 0;
    d_p5_d_disto(0, 3) = 1.0;
    d_p5_d_disto(0, 4) = 0;
    d_p5_d_disto(0, 5) = r2;

    Eigen::Matrix<double, 2, 6> ret;

    ret.block<1, 6>(0, 0) = x * d_p1_d_disto + p2 * d_p4_d_disto + p6 * d_p5_d_disto;
    ret.block<1, 6>(1, 0) = y * d_p1_d_disto + p3 * d_p5_d_disto + p6 * d_p4_d_disto;

    return ret;
}

Vec2 Distortion3DERadial4::removeDistortion(const Vec2& p) const
{
    const double epsilon = 1e-8;
    Vec2 undistorted_value = p;

    Vec2 diff = addDistortion(undistorted_value) - p;

    int iter = 0;
    while (diff.norm() > epsilon)
    {
        undistorted_value = undistorted_value - getDerivativeAddDistoWrtPt(undistorted_value).inverse() * diff;
        diff = addDistortion(undistorted_value) - p;
        iter++;
        if (iter > 100)
            break;
    }

    return undistorted_value;
}

Vec2 Distortion3DEAnamorphic4::addDistortion(const Vec2& p) const
{
    const double& cx02 = _distortionParams[0];
    const double& cy02 = _distortionParams[1];
    const double& cx22 = _distortionParams[2];
    const double& cy22 = _distortionParams[3];
    const double& cx04 = _distortionParams[4];
    const double& cy04 = _distortionParams[5];
    const double& cx24 = _distortionParams[6];
    const double& cy24 = _distortionParams[7];
    const double& cx44 = _distortionParams[8];
    const double& cy44 = _distortionParams[9];
    const double& phi = _distortionParams[10];
    const double& sqx = _distortionParams[11];
    const double& sqy = _distortionParams[12];

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
    const double xxxx = xx * xx;
    const double yyyy = yy * yy;
    const double xxyy = xx * yy;

    // Compute dist
    const double xd = xr * (1.0 + xx * cx_xx + yy * cx_yy + xxxx * cx_xxxx + xxyy * cx_xxyy + yyyy * cx_yyyy);
    const double yd = yr * (1.0 + xx * cy_xx + yy * cy_yy + xxxx * cy_xxxx + xxyy * cy_xxyy + yyyy * cy_yyyy);

    // Squeeze axis
    const double squizzed_x = xd * sqx;
    const double squizzed_y = yd * sqy;

    // Unrotate axis
    Vec2 np{cphi * squizzed_x - sphi * squizzed_y, sphi * squizzed_x + cphi * squizzed_y};

    return np;
}

Eigen::Matrix2d Distortion3DEAnamorphic4::getDerivativeAddDistoWrtPt(const Vec2& p) const
{
    const double& cx02 = _distortionParams[0];
    const double& cy02 = _distortionParams[1];
    const double& cx22 = _distortionParams[2];
    const double& cy22 = _distortionParams[3];
    const double& cx04 = _distortionParams[4];
    const double& cy04 = _distortionParams[5];
    const double& cx24 = _distortionParams[6];
    const double& cy24 = _distortionParams[7];
    const double& cx44 = _distortionParams[8];
    const double& cy44 = _distortionParams[9];
    const double& phi = _distortionParams[10];
    const double& sqx = _distortionParams[11];
    const double& sqy = _distortionParams[12];

    const double cphi = std::cos(phi);
    const double sphi = std::sin(phi);

    const double cx_xx = cx02 + cx22;
    const double cx_yy = cx02 - cx22;
    const double cx_xxyy = 2 * cx04 - 6 * cx44;
    const double cx_xxxx = cx04 + cx24 + cx44;
    const double cx_yyyy = cx04 - cx24 + cx44;

    const double cy_xx = cy02 + cy22;
    const double cy_yy = cy02 - cy22;
    const double cy_xxyy = cy04 + cy24 + cy44;
    const double cy_xxxx = 2 * cy04 - 6 * cy44;
    const double cy_yyyy = cy04 - cy24 + cy44;

    const double& x = p.x();
    const double& y = p.y();

    // First rotate axis
    double xr = cphi * x + sphi * y;
    double yr = -sphi * x + cphi * y;

    const double xx = xr * xr;
    const double yy = yr * yr;
    const double xy = xr * yr;
    const double xxxx = xx * xx;
    const double yyyy = yy * yy;
    const double xxyy = xx * yy;
    const double xxxy = xx * xy;
    const double xyyy = xy * yy;

    // Compute dist
    const double xd = xr * (1.0 + xx * cx_xx + yy * cx_yy + xxxx * cx_xxxx + xxyy * cx_xxyy + yyyy * cx_yyyy);
    const double yd = yr * (1.0 + xx * cy_xx + yy * cy_yy + xxxx * cy_xxxx + xxyy * cy_xxyy + yyyy * cy_yyyy);

    Eigen::Matrix2d d_np_d_squizzed;
    d_np_d_squizzed(0, 0) = cphi;
    d_np_d_squizzed(0, 1) = -sphi;
    d_np_d_squizzed(1, 0) = sphi;
    d_np_d_squizzed(1, 1) = cphi;

    Eigen::Matrix2d d_squizzed_d_d = Eigen::Matrix2d::Zero();
    d_squizzed_d_d(0, 0) = sqx;
    d_squizzed_d_d(1, 1) = sqy;

    Eigen::Matrix2d d_d_d_r;
    d_d_d_r(0, 0) = 1.0 + 3.0 * xx * cx_xx + yy * cx_yy + 5.0 * xxxx * cx_xxxx + 3.0 * xxyy * cx_xxyy + yyyy * cx_yyyy;
    d_d_d_r(0, 1) = 2.0 * xy * cx_yy + 2.0 * xxxy * cx_xxyy + 4.0 * xyyy * cx_yyyy;
    d_d_d_r(1, 0) = 2.0 * xy * cy_xx + 4.0 * xxxy * cy_xxxx + 2.0 * xyyy * cy_xxyy;
    d_d_d_r(1, 1) = 1.0 + xx * cy_xx + 3.0 * yy * cy_yy + xxxx * cy_xxxx + 3.0 * xxyy * cy_xxyy + 5.0 * yyyy * cy_yyyy;

    Eigen::Matrix2d d_r_d_p = Eigen::Matrix2d::Identity();

    return d_np_d_squizzed * d_squizzed_d_d * d_d_d_r * d_r_d_p;
}

Eigen::MatrixXd Distortion3DEAnamorphic4::getDerivativeAddDistoWrtDisto(const Vec2& p) const
{
    const double& cx02 = _distortionParams[0];
    const double& cy02 = _distortionParams[1];
    const double& cx22 = _distortionParams[2];
    const double& cy22 = _distortionParams[3];
    const double& cx04 = _distortionParams[4];
    const double& cy04 = _distortionParams[5];
    const double& cx24 = _distortionParams[6];
    const double& cy24 = _distortionParams[7];
    const double& cx44 = _distortionParams[8];
    const double& cy44 = _distortionParams[9];
    const double& phi = _distortionParams[10];
    const double& sqx = _distortionParams[11];
    const double& sqy = _distortionParams[12];

    const double cphi = std::cos(phi);
    const double sphi = std::sin(phi);

    const double cx_xx = cx02 + cx22;
    const double cx_yy = cx02 - cx22;
    const double cx_xxyy = 2 * cx04 - 6 * cx44;
    const double cx_xxxx = cx04 + cx24 + cx44;
    const double cx_yyyy = cx04 - cx24 + cx44;
    const double cy_xx = cy02 + cy22;
    const double cy_yy = cy02 - cy22;
    const double cy_xxyy = cy04 + cy24 + cy44;
    const double cy_xxxx = 2 * cy04 - 6 * cy44;
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
    const double xyyy = xy * yy;

    const double xxxxx = xxxx * xr;
    const double xxxxy = xxxx * yr;
    const double xxxyy = xxx * yy;
    const double yyyyy = yyyy * yr;
    const double xyyyy = xr * yyyy;
    const double xxyyy = xx * yyy;

    // Compute dist
    const double xd = xr * (1.0 + xx * cx_xx + yy * cx_yy + xxxx * cx_xxxx + xxyy * cx_xxyy + yyyy * cx_yyyy);
    const double yd = yr * (1.0 + xx * cy_xx + yy * cy_yy + xxxx * cy_xxxx + xxyy * cy_xxyy + yyyy * cy_yyyy);

    Eigen::Matrix2d d_np_d_squizzed;
    d_np_d_squizzed(0, 0) = cphi;
    d_np_d_squizzed(0, 1) = -sphi;
    d_np_d_squizzed(1, 0) = sphi;
    d_np_d_squizzed(1, 1) = cphi;

    Eigen::Matrix2d d_squizzed_d_d = Eigen::Matrix2d::Zero();
    d_squizzed_d_d(0, 0) = sqx;
    d_squizzed_d_d(1, 1) = sqy;

    Eigen::Matrix<double, 2, 14> d_squizzed_d_disto = Eigen::Matrix<double, 2, 14>::Zero();
    d_squizzed_d_disto(0, 11) = xd;
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

    J.block(0, 10, 2, 4) = Eigen::Matrix<double, 2, 4>::Zero();

    return J;
}

Vec2 Distortion3DEAnamorphic4::removeDistortion(const Vec2& p) const
{
    double epsilon = 1e-8;
    Vec2 undistorted_value = p;

    Vec2 diff = addDistortion(undistorted_value) - p;

    int iter = 0;
    while (diff.norm() > epsilon)
    {
        undistorted_value = undistorted_value - getDerivativeAddDistoWrtPt(undistorted_value).inverse() * diff;
        diff = addDistortion(undistorted_value) - p;
        iter++;
        if (iter > 10)
            break;
    }

    return undistorted_value;
}

Vec2 Distortion3DEClassicLD::addDistortion(const Vec2& p) const
{
    const double& delta = _distortionParams[0];
    const double& invepsilon = _distortionParams[1];
    const double& mux = _distortionParams[2];
    const double& muy = _distortionParams[3];
    const double& q = _distortionParams[4];

    const double eps = 1.0 + std::cos(invepsilon);

    const double cxx = delta * eps;
    const double cxy = (delta + mux) * eps;
    const double cxxx = q * eps;
    const double cxxy = 2.0 * q * eps;
    const double cxyy = q * eps;
    const double cyx = delta + muy;
    const double cyy = delta;
    const double cyxx = q;
    const double cyxy = 2.0 * q;
    const double cyyy = q;

    const double& x = p.x();
    const double& y = p.y();

    const double xx = x * x;
    const double yy = y * y;
    const double xxxx = xx * xx;
    const double yyyy = yy * yy;
    const double xxyy = xx * yy;

    Vec2 np;
    np.x() = x * (1.0 + cxx * xx + cxy * yy + cxxx * xxxx + cxxy * xxyy + cxyy * yyyy);
    np.y() = y * (1.0 + cyx * xx + cyy * yy + cyxx * xxxx + cyxy * xxyy + cyyy * yyyy);

    return np;
}

Eigen::Matrix2d Distortion3DEClassicLD::getDerivativeAddDistoWrtPt(const Vec2& p) const
{
    const double& delta = _distortionParams[0];
    const double& invepsilon = _distortionParams[1];
    const double& mux = _distortionParams[2];
    const double& muy = _distortionParams[3];
    const double& q = _distortionParams[4];

    const double eps = 1.0 + std::cos(invepsilon);

    const double cxx = delta * eps;
    const double cxy = (delta + mux) * eps;
    const double cxxx = q * eps;
    const double cxxy = 2.0 * q * eps;
    const double cxyy = q * eps;
    const double cyx = delta + muy;
    const double cyy = delta;
    const double cyxx = q;
    const double cyxy = 2.0 * q;
    const double cyyy = q;

    const double& x = p.x();
    const double& y = p.y();

    const double xx = x * x;
    const double yy = y * y;
    const double xy = x * y;

    const double xxx = xx * x;
    const double yyy = yy * y;
    const double xxy = xx * y;
    const double xyy = x * yy;

    const double xxxx = xx * xx;
    const double yyyy = yy * yy;
    const double xxyy = xx * yy;

    const double xyyy = x * yyy;
    const double xxxy = xxx * y;

    Eigen::Matrix2d ret;

    ret(0, 0) = 1.0 + 3.0 * cxx * xx + cxy * yy + 5.0 * cxxx * xxxx + 3.0 * cxxy * xxyy + cxyy * yyyy;
    ret(0, 1) = 2.0 * cxy * xy + 2.0 * cxxy * xxxy + 4.0 * cxyy * xyyy;
    ret(1, 0) = 2.0 * cyx * xy + 4.0 * cyxx * xxxy + 2.0 * cyxy * xyyy;
    ret(1, 1) = 1.0 + cyx * xx + 3.0 * cyy * yy + cyxx * xxxx + 3.0 * cyxy * xxyy + 5.0 * cyyy * yyyy;

    return ret;
}

Eigen::MatrixXd Distortion3DEClassicLD::getDerivativeAddDistoWrtDisto(const Vec2& p) const
{
    const double& delta = _distortionParams[0];
    const double& invepsilon = _distortionParams[1];
    const double& mux = _distortionParams[2];
    const double& muy = _distortionParams[3];
    const double& q = _distortionParams[4];

    const double eps = 1.0 + std::cos(invepsilon);

    const double cxx = delta * eps;
    const double cxy = (delta + mux) * eps;
    const double cxxx = q * eps;
    const double cxxy = 2.0 * q * eps;
    const double cxyy = q * eps;
    const double cyx = delta + muy;
    const double cyy = delta;
    const double cyxx = q;
    const double cyxy = 2.0 * q;
    const double cyyy = q;

    const double& x = p.x();
    const double& y = p.y();

    const double xx = x * x;
    const double yy = y * y;

    const double xxx = xx * x;
    const double xyy = x * yy;
    const double xxy = xx * y;
    const double yyy = yy * y;
    const double xxxx = xx * xx;
    const double xxxxx = xxxx * x;
    const double yyyy = yy * yy;
    const double yyyyy = yyyy * y;
    const double xxxxy = xxxx * y;
    const double xxxyy = xxx * yy;
    const double xxyyy = xx * yyy;
    const double xyyyy = x * yyyy;

    Eigen::Matrix<double, 2, 10> ret;

    ret(0, 0) = xxx;
    ret(0, 1) = xyy;
    ret(0, 2) = xxxxx;
    ret(0, 3) = xxxyy;
    ret(0, 4) = xyyyy;
    ret(0, 5) = 0;
    ret(0, 6) = 0;
    ret(0, 7) = 0;
    ret(0, 8) = 0;
    ret(0, 9) = 0;

    ret(1, 0) = 0;
    ret(1, 1) = 0;
    ret(1, 2) = 0;
    ret(1, 3) = 0;
    ret(1, 4) = 0;
    ret(1, 5) = xxy;
    ret(1, 6) = yyy;
    ret(1, 7) = xxxxy;
    ret(1, 8) = xxyyy;
    ret(1, 9) = yyyyy;

    Eigen::Matrix<double, 10, 5> localParams = Eigen::Matrix<double, 10, 5>::Zero();

    const double d_eps_d_invepsilon = -std::sin(invepsilon);

    localParams(0, 0) = eps;
    localParams(0, 1) = delta * d_eps_d_invepsilon;
    localParams(1, 0) = eps;
    localParams(1, 1) = (delta + mux) * d_eps_d_invepsilon;
    localParams(1, 2) = eps;
    localParams(2, 1) = q * d_eps_d_invepsilon;
    localParams(2, 4) = eps;
    localParams(3, 1) = 2.0 * q * d_eps_d_invepsilon;
    localParams(3, 4) = 2.0 * eps;
    localParams(4, 1) = q * d_eps_d_invepsilon;
    localParams(4, 4) = eps;
    localParams(5, 0) = 1;
    localParams(5, 3) = 1;
    localParams(6, 0) = 1;
    localParams(7, 4) = 1;
    localParams(8, 4) = 2;
    localParams(9, 4) = 1;

    return ret * localParams;
}

Vec2 Distortion3DEClassicLD::removeDistortion(const Vec2& p) const
{
    const double epsilon = 1e-8;
    Vec2 undistorted_value = p;

    Vec2 diff = addDistortion(undistorted_value) - p;

    int iter = 0;
    while (diff.norm() > epsilon)
    {
        undistorted_value = undistorted_value - getDerivativeAddDistoWrtPt(undistorted_value).inverse() * diff;
        diff = addDistortion(undistorted_value) - p;
        iter++;
        if (iter > 1000)
            break;
    }

    return undistorted_value;
}

}  // namespace camera
}  // namespace aliceVision
