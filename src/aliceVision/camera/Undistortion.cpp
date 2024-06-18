// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Undistortion.hpp"
#include <aliceVision/system/Logger.hpp>

namespace aliceVision {
namespace camera {

Vec2 Undistortion::undistort(const Vec2& p) const
{
    Vec2 centered;

    centered(0) = p(0) - _center(0) - _offset(0);
    centered(1) = p(1) - _center(1) - _offset(1);

    Vec2 normalized;
    normalized(0) = centered(0) / _diagonal;
    normalized(1) = centered(1) / (_pixelAspectRatio * _diagonal);

    Vec2 undistorted = undistortNormalized(normalized);
    Vec2 unnormalized;

    unnormalized(0) = undistorted(0) * _diagonal + _center(0) + _offset(0);
    unnormalized(1) = undistorted(1) * _diagonal * _pixelAspectRatio + _center(1) + _offset(1);

    return unnormalized;
}

Eigen::Matrix<double, 2, Eigen::Dynamic> Undistortion::getDerivativeUndistortWrtParameters(const Vec2& p)
{
    Vec2 centered;

    centered(0) = p(0) - _center(0) - _offset(0);
    centered(1) = p(1) - _center(1) - _offset(1);

    Vec2 normalized;
    normalized(0) = centered(0) / _diagonal;
    normalized(1) = centered(1) / (_pixelAspectRatio * _diagonal);

    Vec2 undistorted = undistortNormalized(normalized);
    Vec2 unnormalized;

    unnormalized(0) = undistorted(0) * _diagonal + _center(0) + _offset(0);
    unnormalized(1) = undistorted(1) * _diagonal * _pixelAspectRatio + _center(1) + _offset(1);

    Eigen::Matrix<double, 2, 2> d_unnormalized_d_undistorted;
    d_unnormalized_d_undistorted(0, 0) = _diagonal;
    d_unnormalized_d_undistorted(0, 1) = 0;
    d_unnormalized_d_undistorted(1, 0) = 0;
    d_unnormalized_d_undistorted(1, 1) = _diagonal * _pixelAspectRatio;

    return d_unnormalized_d_undistorted * getDerivativeUndistortNormalizedwrtParameters(normalized);
}

Eigen::Matrix<double, 2, 2> Undistortion::getDerivativeUndistortWrtOffset(const Vec2& p)
{
    Vec2 centered;

    centered(0) = p(0) - _center(0) - _offset(0);
    centered(1) = p(1) - _center(1) - _offset(1);

    Vec2 normalized;
    normalized(0) = centered(0) / _diagonal;
    normalized(1) = centered(1) / (_pixelAspectRatio * _diagonal);

    Vec2 undistorted = undistortNormalized(normalized);
    Vec2 unnormalized;

    unnormalized(0) = undistorted(0) * _diagonal + _center(0) + _offset(0);
    unnormalized(1) = undistorted(1) * _diagonal * _pixelAspectRatio + _center(1) + _offset(1);

    Eigen::Matrix<double, 2, 2> d_unnormalized_d_undistorted;
    d_unnormalized_d_undistorted(0, 0) = _diagonal;
    d_unnormalized_d_undistorted(0, 1) = 0;
    d_unnormalized_d_undistorted(1, 0) = 0;
    d_unnormalized_d_undistorted(1, 1) = _diagonal * _pixelAspectRatio;

    Eigen::Matrix<double, 2, 2> d_normalized_d_centered;
    d_normalized_d_centered(0, 0) = 1.0 / _diagonal;
    d_normalized_d_centered(0, 1) = 0;
    d_normalized_d_centered(1, 0) = 0;
    d_normalized_d_centered(1, 1) = 1.0 / (_diagonal * _pixelAspectRatio);

    return Eigen::Matrix2d::Identity() +
           d_unnormalized_d_undistorted * getDerivativeUndistortNormalizedwrtPoint(normalized) * d_normalized_d_centered * -1.0;
}

Vec2 Undistortion::inverse(const Vec2& p) const
{
    Vec2 centered;

    centered(0) = p(0) - _center(0) - _offset(0);
    centered(1) = p(1) - _center(1) - _offset(1);

    Vec2 normalized;
    normalized(0) = centered(0) / _diagonal;
    normalized(1) = centered(1) / (_diagonal * _pixelAspectRatio);

    Vec2 distorted = inverseNormalized(normalized);
    Vec2 unnormalized;


    unnormalized(0) = distorted(0) * _diagonal + _center(0) + _offset(0);
    unnormalized(1) = distorted(1) * _diagonal * _pixelAspectRatio + _center(1) + _offset(1);

    return unnormalized;
}

}  // namespace camera
}  // namespace aliceVision
