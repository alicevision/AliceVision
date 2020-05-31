// This file is part of the AliceVision project.
// Copyright (c) 2019 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

namespace aliceVision {
namespace multiview {
namespace resection {

/**
 * @brief Resection solver error interface.
 */
template <typename ModelT>
struct ISolverErrorResection
{
  virtual double error(const ModelT& model, const Vec2& x2d, const Vec3& x3d) const = 0;
};

}  // namespace resection
}  // namespace multiview
}  // namespace aliceVision
