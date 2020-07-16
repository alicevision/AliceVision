// This file is part of the AliceVision project.
// Copyright (c) 2019 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/numeric/numeric.hpp>


namespace aliceVision {
namespace multiview {
namespace relativePose {

/**
 * @brief Relative pose solver error interface.
 */
template <typename ModelT>
struct ISolverErrorRelativePose
{
  virtual double error(const ModelT& model, const Vec2& x1, const Vec2& x2) const = 0;
};

}  // namespace relativePose
}  // namespace multiview
}  // namespace aliceVision
