// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Undistortion3DERadial4.hpp"

namespace aliceVision {
namespace camera {

Vec2 Undistortion3DERadial4::undistortNormalized(const Vec2& p) const
{
    const double & distortion_degree_2 = _undistortionParams[0];
    const double & u_degree_2 = _undistortionParams[1];
    const double & v_degree_2 = _undistortionParams[2];
    const double & quartic_distortion_degree_4 = _undistortionParams[3];
    const double & u_degree_4 = _undistortionParams[4];
    const double & v_degree_4 = _undistortionParams[5];
    const double & phi_cylindric_direction = _undistortionParams[6];
    const double & b_cylindric_bending = _undistortionParams[7];

    const double q = sqrt(1.0 + b_cylindric_bending);
    const double cphi = cos(phi_cylindric_direction);
    const double sphi = sin(phi_cylindric_direction);

    double m11 = cphi * cphi * q + sphi * sphi / q;
    double m12 = (q - 1.0 / q) * cphi * sphi;
    double m21 = (q - 1.0 / q) * cphi * sphi;
    double m22 = cphi * cphi / q + sphi * sphi * q;

    const double x = p.x();
    const double y = p.y();
    const double x2 = x * x;
    const double y2 = y * y;
    const double xy = x * y;
    const double r2 = x2 + y2;
    const double r4 = r2 * r2;

    const double c2 = distortion_degree_2;
    const double c4 = quartic_distortion_degree_4;
    const double u1 = u_degree_2;
    const double v1 = v_degree_2;
    const double u3 = u_degree_4;
    const double v3 = v_degree_4;

    Eigen::Vector2d radial;
    radial.x() = x * (1.0 + c2 * r2 + c4 * r4) + (r2 + 2.0 * x2) * (u1 + u3 * r2) + 2.0 * xy * (v1 + v3 * r2);
    radial.y() = y * (1.0 + c2 * r2 + c4 * r4) + (r2 + 2.0 * y2) * (v1 + v3 * r2) + 2.0 * xy * (u1 + u3 * r2);

    Eigen::Vector2d np;
    np.x() = m11 * radial.x() + m12 * radial.y();
    np.y() = m21 * radial.x() + m22 * radial.y();
    
    return np;
}


Eigen::Matrix<double, 2, 2> Undistortion3DERadial4::getDerivativeUndistortNormalizedwrtPoint(const Vec2& p) const
{
    const double & distortion_degree_2 = _undistortionParams[0];
    const double & u_degree_2 = _undistortionParams[1];
    const double & v_degree_2 = _undistortionParams[2];
    const double & quartic_distortion_degree_4 = _undistortionParams[3];
    const double & u_degree_4 = _undistortionParams[4];
    const double & v_degree_4 = _undistortionParams[5];
    const double & phi_cylindric_direction = _undistortionParams[6];
    const double & b_cylindric_bending = _undistortionParams[7];

    const double q = sqrt(1.0 + b_cylindric_bending);
    const double cphi = cos(phi_cylindric_direction);
    const double sphi = sin(phi_cylindric_direction);

    const double m11 = cphi * cphi * q + sphi * sphi / q;
    const double m12 = (q - 1.0 / q) * cphi * sphi;
    const double m21 = (q - 1.0 / q) * cphi * sphi;
    const double m22 = cphi * cphi / q + sphi * sphi * q;

    const double x = p.x();
    const double y = p.y();
    const double x2 = x * x;
    const double y2 = y * y;
    const double xy = x * y;
    const double r2 = x2 + y2;
    const double r4 = r2 * r2;

    const double c2 = distortion_degree_2;
    const double c4 = quartic_distortion_degree_4;
    const double u1 = u_degree_2;
    const double v1 = v_degree_2;
    const double u3 = u_degree_4;
    const double v3 = v_degree_4;

    Eigen::Vector2d radial;
    radial.x() = x * (1.0 + c2 * r2 + c4 * r4) + (r2 + 2.0 * x2) * (u1 + u3 * r2) + 2.0 * xy * (v1 + v3 * r2);
    radial.y() = y * (1.0 + c2 * r2 + c4 * r4) + (r2 + 2.0 * y2) * (v1 + v3 * r2) + 2.0 * xy * (u1 + u3 * r2);

    Eigen::Vector2d np;
    np.x() = m11 * radial.x() + m12 * radial.y();
    np.y() = m21 * radial.x() + m22 * radial.y();

    Eigen::Matrix2d d_radial_d_pt;
    d_radial_d_pt(0, 0) = c2*r2 + c4*r4 + x2*(2*c2 + 4*c4*r2 + 4.0*v3*y) + x*(6.0*u1 + 6.0*u3*r2 + 2*u3*(3.0*x2 + y2)) + y*(2.0*v1 + 2.0*v3*r2) + 1.0;
    d_radial_d_pt(0, 1) = x*(2.0*v1 + 4.0*v3*y2 + 2.0*v3*r2 + y*(2*c2 + 4*c4*r2)) + y*(2*u1 + 2*u3*r2 + 2*u3*(3.0*x2 + y2));
    d_radial_d_pt(1, 0) = 4.0*u3*x2*y + x*(2*v1 + 2*v3*r2 + 2*v3*(x2 + 3.0*y2) + y*(2*c2 + 4*c4*r2)) + y*(2.0*u1 + 2.0*u3*r2);
    d_radial_d_pt(1, 1) = c2*r2 + c4*r4 + x*(2.0*u1 + 4.0*u3*y2 + 2.0*u3*r2) + y2*(2*c2 + 4*c4*r2) + y*(6.0*v1 + 6.0*v3*r2 + 2*v3*(x2 + 3.0*y2)) + 1.0;

    Eigen::Matrix<double, 2, 2> d_np_d_radial;
    d_np_d_radial(0, 0) = m11;
    d_np_d_radial(0, 1) = m12;
    d_np_d_radial(1, 0) = m21;
    d_np_d_radial(1, 1) = m22;

    return d_np_d_radial * d_radial_d_pt;
}

Eigen::Matrix<double, 2, Eigen::Dynamic> Undistortion3DERadial4::getDerivativeUndistortNormalizedwrtParameters(const Vec2& p) const
{   
    const double & distortion_degree_2 = _undistortionParams[0];
    const double & u_degree_2 = _undistortionParams[1];
    const double & v_degree_2 = _undistortionParams[2];
    const double & quartic_distortion_degree_4 = _undistortionParams[3];
    const double & u_degree_4 = _undistortionParams[4];
    const double & v_degree_4 = _undistortionParams[5];
    const double & phi_cylindric_direction = _undistortionParams[6];
    const double & b_cylindric_bending = _undistortionParams[7];

    const double q = sqrt(1.0 + b_cylindric_bending);
    const double cphi = cos(phi_cylindric_direction);
    const double sphi = sin(phi_cylindric_direction);

    const double d_q_b_cylindric_bending = 1.0 / (2.0 * q);
    const double d_cphi_d_phi = -sphi;
    const double d_sphi_d_phi = cphi;

    Eigen::Matrix<double, 3, 2> d_intermediatecylindricparams_d_cylindricparams;
    d_intermediatecylindricparams_d_cylindricparams(0, 0) = d_cphi_d_phi;
    d_intermediatecylindricparams_d_cylindricparams(0, 1) = 0;
    d_intermediatecylindricparams_d_cylindricparams(1, 0) = d_sphi_d_phi;
    d_intermediatecylindricparams_d_cylindricparams(1, 1) = 0;
    d_intermediatecylindricparams_d_cylindricparams(2, 0) = 0;
    d_intermediatecylindricparams_d_cylindricparams(2, 1) = d_q_b_cylindric_bending;

    const double m11 = cphi * cphi * q + sphi * sphi / q;
    const double m12 = (q - 1.0 / q) * cphi * sphi;
    const double m21 = (q - 1.0 / q) * cphi * sphi;
    const double m22 = cphi * cphi / q + sphi * sphi * q;

    const double d_invq_d_q = - 1.0 / (q * q);

    const double d_m11_d_cphi = 2.0 * cphi * q;
    const double d_m12_d_cphi = (q - 1.0 / q) * sphi;
    const double d_m21_d_cphi = (q - 1.0 / q) * sphi;
    const double d_m22_d_cphi = 2.0 * cphi / q;

    const double d_m11_d_sphi = 2.0 * sphi / q;
    const double d_m12_d_sphi = (q - 1.0 / q) * cphi;
    const double d_m21_d_sphi = (q - 1.0 / q) * cphi;
    const double d_m22_d_sphi = 2.0 * sphi * q;

    const double d_m11_d_q = cphi * cphi + sphi * sphi * d_invq_d_q;
    const double d_m12_d_q = (1.0 - d_invq_d_q) * cphi * sphi;
    const double d_m21_d_q = (q - d_invq_d_q) * cphi * sphi;
    const double d_m22_d_q = cphi * cphi * d_invq_d_q + sphi * sphi;

    Eigen::Matrix<double, 4, 3> d_m_d_intermediatecylindricparams;

    d_m_d_intermediatecylindricparams(0, 0) = d_m11_d_cphi;
    d_m_d_intermediatecylindricparams(1, 0) = d_m21_d_cphi;
    d_m_d_intermediatecylindricparams(2, 0) = d_m11_d_cphi;
    d_m_d_intermediatecylindricparams(3, 0) = d_m22_d_cphi;
    d_m_d_intermediatecylindricparams(0, 1) = d_m11_d_sphi;
    d_m_d_intermediatecylindricparams(1, 1) = d_m21_d_sphi;
    d_m_d_intermediatecylindricparams(2, 1) = d_m11_d_sphi;
    d_m_d_intermediatecylindricparams(3, 1) = d_m22_d_sphi;
    d_m_d_intermediatecylindricparams(0, 2) = d_m11_d_q;
    d_m_d_intermediatecylindricparams(1, 2) = d_m21_d_q;
    d_m_d_intermediatecylindricparams(2, 2) = d_m11_d_q;
    d_m_d_intermediatecylindricparams(3, 2) = d_m22_d_q;

    const double x = p.x();
    const double y = p.y();
    const double x2 = x * x;
    const double y2 = y * y;
    const double xy = x * y;
    const double r2 = x2 + y2;
    const double r4 = r2 * r2;

    const double c2 = distortion_degree_2;
    const double c4 = quartic_distortion_degree_4;
    const double u1 = u_degree_2;
    const double v1 = v_degree_2;
    const double u3 = u_degree_4;
    const double v3 = v_degree_4;

    Eigen::Vector2d radial;
    radial.x() = x * (1.0 + c2 * r2 + c4 * r4) + (r2 + 2.0 * x2) * (u1 + u3 * r2) + 2.0 * xy * (v1 + v3 * r2);
    radial.y() = y * (1.0 + c2 * r2 + c4 * r4) + (r2 + 2.0 * y2) * (v1 + v3 * r2) + 2.0 * xy * (u1 + u3 * r2);

    

    Eigen::Matrix<double, 2, 6> d_radial_d_radialparams;
    d_radial_d_radialparams(0, 0) = x*r2;
    d_radial_d_radialparams(0, 1) = x*r4;
    d_radial_d_radialparams(0, 2) = 3.0*x2 + y2;
    d_radial_d_radialparams(0, 3) = 2.0*xy;
    d_radial_d_radialparams(0, 4) = r2*(3.0*x2 + y2);
    d_radial_d_radialparams(0, 5) = 2.0*xy*r2;
    d_radial_d_radialparams(1, 0) = y*r2;
    d_radial_d_radialparams(1, 1) = y*r4;
    d_radial_d_radialparams(1, 2) = 2.0*xy;
    d_radial_d_radialparams(1, 3) = x2 + 3.0*y2;
    d_radial_d_radialparams(1, 4) = 2.0*xy*r2;
    d_radial_d_radialparams(1, 5) = r2*(x2 + 3.0*y2);

    Eigen::Vector2d np;
    np.x() = m11 * radial.x() + m12 * radial.y();
    np.y() = m21 * radial.x() + m22 * radial.y();
    
    Eigen::Matrix<double, 2, 2> d_np_d_radial;
    d_np_d_radial(0, 0) = m11;
    d_np_d_radial(0, 1) = m12;
    d_np_d_radial(1, 0) = m21;
    d_np_d_radial(1, 1) = m22;

    Eigen::Matrix<double, 2, 4> d_np_d_m;
    d_np_d_m(0, 0) = radial.x();
    d_np_d_m(0, 1) = radial.x();
    d_np_d_m(0, 2) = 0;
    d_np_d_m(0, 3) = 0;
    d_np_d_m(1, 0) = 0;
    d_np_d_m(1, 1) = 0;
    d_np_d_m(1, 2) = radial.y();
    d_np_d_m(1, 3) = radial.y();

    
    Eigen::Matrix<double, 2, 8> J = Eigen::Matrix<double, 2, 8>::Zero();

    J.block<2, 6>(0, 0) = d_np_d_radial * d_radial_d_radialparams;
    J.block<2, 2>(0, 6) = d_np_d_m * d_m_d_intermediatecylindricparams * d_intermediatecylindricparams_d_cylindricparams;

    return J;
}

Vec2 Undistortion3DERadial4::inverseNormalized(const Vec2& p) const
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

