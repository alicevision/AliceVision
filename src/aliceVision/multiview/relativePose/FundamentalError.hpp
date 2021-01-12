// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// Copyright (c) 2010 libmv contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/robustEstimation/ISolver.hpp>
#include <aliceVision/multiview/relativePose/ISolverErrorRelativePose.hpp>

namespace aliceVision {
namespace multiview {
namespace relativePose {

/**
 * @brief Compute FundamentalSampsonError related to the Fundamental matrix and 2 correspondences
 */
struct FundamentalSampsonError : public ISolverErrorRelativePose<robustEstimation::Mat3Model>
{
  double error(const robustEstimation::Mat3Model& F, const Vec2& x1, const Vec2& x2) const override
  {
    const Vec3 x(x1(0), x1(1), 1.0);
    const Vec3 y(x2(0), x2(1), 1.0);

    // @see page 287 equation (11.9) of HZ.
    const Vec3 F_x = F.getMatrix() * x;
    const Vec3 Ft_y = F.getMatrix().transpose() * y;

    return Square(y.dot(F_x)) / (  F_x.head<2>().squaredNorm() + Ft_y.head<2>().squaredNorm());
  }
};

struct FundamentalSymmetricEpipolarDistanceError: public ISolverErrorRelativePose<robustEstimation::Mat3Model>
{
  double error(const robustEstimation::Mat3Model& F, const Vec2& x1, const Vec2& x2) const override
  {
    const Vec3 x(x1(0), x1(1), 1.0);
    const Vec3 y(x2(0), x2(1), 1.0);

    // @see page 288 equation (11.10) of HZ.
    const Vec3 F_x = F.getMatrix() * x;
    const Vec3 Ft_y = F.getMatrix().transpose() * y;

    // @note the divide by 4 is to make this match the Sampson distance.
    return Square(y.dot(F_x)) * ( 1.0 / F_x.head<2>().squaredNorm() + 1.0 / Ft_y.head<2>().squaredNorm()) / 4.0;
  }
};

struct FundamentalEpipolarDistanceError : public ISolverErrorRelativePose<robustEstimation::Mat3Model>
{
  double error(const robustEstimation::Mat3Model& F, const Vec2& x1, const Vec2& x2) const override
  {
    // transfer error in image 2
    // @see page 287 equation (11.9) of HZ.
    const Vec3 x(x1(0), x1(1), 1.0);
    const Vec3 y(x2(0), x2(1), 1.0);
    const Vec3 F_x = F.getMatrix() * x;

    return Square(F_x.dot(y)) /  F_x.head<2>().squaredNorm();
  }
};


struct EpipolarSphericalDistanceError
{
    double error(const robustEstimation::Mat3Model& F, const Vec3& x, const Vec3& y) const // override
    {
        // Transfer error in image 2
        // See page 287 equation (11.9) of HZ.

        Vec3 F_x = F.getMatrix() * x;
        return Square(F_x.dot(y));
    }
};


}  // namespace relativePose
}  // namespace multiview
}  // namespace aliceVision
