// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Undistortion3DEClassicLD.hpp"

namespace aliceVision {
namespace camera {

Vec2 Undistortion3DEClassicLD::undistortNormalized(const Vec2& p) const
{
    const double & c2 = _undistortionParams[0];
    const double & sq = _undistortionParams[1];
    const double & cx = _undistortionParams[2];
    const double & cy = _undistortionParams[3];
    const double & c4 = _undistortionParams[4];

    const double cxx = c2 / sq;
    const double cxy = (c2 + cx) / sq;
    const double cxxx = c4 / sq;
    const double cxxy = 2.0 * c4 / sq;
    const double cxyy = c4 / sq;
    const double cyx = c2 + cy;
    const double cyy = c2;
    const double cyxx = c4;
    const double cyyx = 2.0 * c4;
    const double cyyy = c4;

    const double x = p.x();
    const double y = p.y();
    const double x2 = x * x;
    const double y2 = y * y;
    const double x4 = x2 * x2;
    const double y4 = y2 * y2;
    const double x2y2 = x2 * y2;

    Vec2 np;

    np.x() = x * (1.0 + cxx * x2 + cxy * y2 + cxxx * x4 + cxxy * x2y2 + cxyy * y4);
    np.y() = y * (1.0 + cyx * x2 + cyy * y2 + cyxx * x4 + cyyx * x2y2 + cyyy * y4);

    return np;
}


Eigen::Matrix<double, 2, 2> Undistortion3DEClassicLD::getDerivativeUndistortNormalizedwrtPoint(const Vec2& p) const
{
    const double & c2 = _undistortionParams[0];
    const double & sq = _undistortionParams[1];
    const double & cx = _undistortionParams[2];
    const double & cy = _undistortionParams[3];
    const double & c4 = _undistortionParams[4];

    const double cxx = c2 / sq;
    const double cxy = (c2 + cx) / sq;
    const double cxxx = c4 / sq;
    const double cxxy = 2.0 * c4 / sq;
    const double cxyy = c4 / sq;
    const double cyx = c2 + cy;
    const double cyy = c2;
    const double cyxx = c4;
    const double cyyx = 2.0 * c4;
    const double cyyy = c4;

    const double x = p.x();
    const double y = p.y();
    const double x2 = x * x;
    const double y2 = y * y;
    const double x3 = x2 * x;
    const double y3 = y2 * y;
    const double x4 = x2 * x2;
    const double y4 = y2 * y2;
    const double x2y2 = x2 * y2;

    Vec2 np;

    const double d1 = (1.0 + cxx * x2 + cxy * y2 + cxxx * x4 + cxxy * x2y2 + cxyy * y4);
    const double d2 = (1.0 + cyx * x2 + cyy * y2 + cyxx * x4 + cyyx * x2y2 + cyyy * y4);
    
    np.x() = x * d1;
    np.y() = y * d2;

    const double d_d1_d_x = cxx * 2.0 * x + cxxx * 4.0 * x3 + cxxy * 2.0 * x * y2;
    const double d_d1_d_y = cxy * 2.0 * y + cxxy * 2.0 * y * x2 + cxyy * 4.0 * y3;

    const double d_d2_d_x = cyx * 2.0 * x + cyxx * 4.0 * x3 + cyyx * 2.0 * x * y2;
    const double d_d2_d_y = cyy * 2.0 * y + cyyx * 2.0 * y * x2 + cyyy * 4.0 * y3;

    Eigen::Matrix<double, 2, 2> J;
    J(0, 0) = 1.0 * d1 + x * d_d1_d_x;
    J(0, 1) = x * d_d1_d_y;
    J(1, 0) = y * d_d2_d_x;
    J(1, 1) = 1.0 * d2 + y * d_d2_d_y;

    return J;
}

Eigen::Matrix<double, 2, Eigen::Dynamic> Undistortion3DEClassicLD::getDerivativeUndistortNormalizedwrtParameters(const Vec2& p) const
{   
    const double & c2 = _undistortionParams[0];
    const double & sq = _undistortionParams[1];
    const double & cx = _undistortionParams[2];
    const double & cy = _undistortionParams[3];
    const double & c4 = _undistortionParams[4];

    const double cxx = c2 / sq;
    const double cxy = (c2 + cx) / sq;
    const double cxxx = c4 / sq;
    const double cxxy = 2.0 * c4 / sq;
    const double cxyy = c4 / sq;
    const double cyx = c2 + cy;
    const double cyy = c2;
    const double cyxx = c4;
    const double cyyx = 2.0 * c4;
    const double cyyy = c4;

    const double sq2 = sq * sq;

    Eigen::Matrix<double, 10, 5> d_intermediate_d_params = Eigen::Matrix<double, 10, 5>::Zero();
    
    d_intermediate_d_params(0, 0) = 1.0 / sq;
    d_intermediate_d_params(0, 1) = - c2 / sq2;
    d_intermediate_d_params(1, 0) = 1.0 / sq;
    d_intermediate_d_params(1, 1) = - (c2 + cx) / sq2;
    d_intermediate_d_params(1, 2) = 1.0 / sq;
    d_intermediate_d_params(2, 1) = - c4 / sq2;
    d_intermediate_d_params(2, 4) = 1.0 / sq;
    d_intermediate_d_params(3, 1) = -2.0 * c4 / sq2;
    d_intermediate_d_params(3, 4) = 2.0 / sq;
    d_intermediate_d_params(4, 1) = - c4 / sq2;
    d_intermediate_d_params(4, 4) = 1.0 / sq;
    
    d_intermediate_d_params(5, 0) = 1.0;
    d_intermediate_d_params(5, 3) = 1.0;
    d_intermediate_d_params(6, 0) = 1.0;
    d_intermediate_d_params(7, 4) = 1.0;
    d_intermediate_d_params(8, 4) = 2.0;
    d_intermediate_d_params(9, 4) = 1.0;

    

    const double x = p.x();
    const double y = p.y();
    const double x2 = x * x;
    const double y2 = y * y;
    const double x3 = x2 * x;
    const double y3 = y2 * y;
    const double x4 = x2 * x2;
    const double y4 = y2 * y2;
    const double x2y2 = x2 * y2;

    Vec2 np;
    np.x() = x * (1.0 + cxx * x2 + cxy * y2 + cxxx * x4 + cxxy * x2y2 + cxyy * y4);
    np.y() = y * (1.0 + cyx * x2 + cyy * y2 + cyxx * x4 + cyyx * x2y2 + cyyy * y4);

    Eigen::Matrix<double, 2, 10> d_np_d_intermediate = Eigen::Matrix<double, 2, 10>::Zero();
    
    d_np_d_intermediate(0, 0) = x * x2;
    d_np_d_intermediate(0, 1) = x * y2;
    d_np_d_intermediate(0, 2) = x * x4;
    d_np_d_intermediate(0, 3) = x * x2y2;
    d_np_d_intermediate(0, 4) = x * y4;

    d_np_d_intermediate(1, 5) = y * x2;
    d_np_d_intermediate(1, 6) = y * y2;
    d_np_d_intermediate(1, 7) = y * x4;
    d_np_d_intermediate(1, 8) = y * x2y2;
    d_np_d_intermediate(1, 9) = y * y4;

    return d_np_d_intermediate * d_intermediate_d_params;
}

Vec2 Undistortion3DEClassicLD::inverseNormalized(const Vec2& p) const
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
