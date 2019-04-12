// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// Copyright (c) 2010 libmv contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/multiview/ISolver.hpp>

namespace aliceVision {
namespace multiview {
namespace relativePose {

/**
 * @brief Compute Essential error with a Fundamental error
 */
/*
template<typename FundamentalErrorT>
struct EssentialErrorFromFundamental : public ISolverErrorRelativePose<Mat3Model>
{
  inline double error(const Mat3Model& model, const Vec2& x1, const Vec2& x2) const override
  {
    Mat3 F;
    fundamentalFromEssential(model.getMatrix(), _K1, _K2, F);
    FundamentalErrorT errorEstimator;
    return errorEstimator.error(F, x1, x2);
  }
};
*/
}  // namespace relativePose
}  // namespace multiview
}  // namespace aliceVision
