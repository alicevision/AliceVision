// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
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
	const double c = cos(phi_cylindric_direction);
	const double s = sin(phi_cylindric_direction);

    double m11 = c * c * q + s * s / q;
    double m12 = (q - 1.0 / q) * c * s;
    double m21 = (q - 1.0 / q) * c * s;
    double m22 = c * c / q + s * s * q;

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
    Eigen::Matrix<double, 2, 2> J;

    return J;
}

Eigen::Matrix<double, 2, Eigen::Dynamic> Undistortion3DERadial4::getDerivativeUndistortNormalizedwrtParameters(const Vec2& p) const
{   
    Eigen::Matrix<double, 2, 10> J;
    
    
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

