// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// Copyright (c) 2010 libmv contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/numeric/projection.hpp>
#include <aliceVision/robustEstimation/ISolver.hpp>
#include <aliceVision/multiview/relativePose/ISolverErrorRelativePose.hpp>

namespace aliceVision {
namespace multiview {
namespace relativePose {

/**
 * @brief The HomographyAsymmetricError struct
 * @note Should be distributed as Chi-squared with k = 2.
 */
struct HomographyAsymmetricError : ISolverErrorRelativePose<robustEstimation::Mat3Model>
{
    inline double error(const robustEstimation::Mat3Model& H, const Vec2& x1, const Vec2& x2) const override
    {
        const Vec3 x2h_est = H.getMatrix() * euclideanToHomogeneous(x1);
        const Vec2 x2_est = x2h_est.head<2>() / x2h_est[2];
        return (x2 - x2_est).squaredNorm();
    }
};

}  // namespace relativePose
}  // namespace multiview
}  // namespace aliceVision
