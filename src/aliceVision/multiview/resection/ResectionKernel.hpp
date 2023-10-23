// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/robustEstimation/PointFittingKernel.hpp>
#include <aliceVision/multiview/resection/Resection6PSolver.hpp>
#include <aliceVision/multiview/resection/ProjectionDistanceError.hpp>

namespace aliceVision {
namespace multiview {
namespace resection {

/**
 * @brief Usable solver for the 6pt Resection estimation
 */
typedef robustEstimation::PointFittingKernel<Resection6PSolver, ProjectionDistanceError, robustEstimation::Mat34Model> Resection6PKernel;

}  // namespace resection
}  // namespace multiview
}  // namespace aliceVision
