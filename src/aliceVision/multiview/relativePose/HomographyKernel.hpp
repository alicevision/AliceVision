// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// Copyright (c) 2010 libmv contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/multiview/TwoViewKernel.hpp>
#include <aliceVision/multiview/relativePose/Homography4PSolver.hpp>
#include <aliceVision/multiview/relativePose/HomographyError.hpp>

namespace aliceVision {
namespace multiview {
namespace relativePose {

/**
 * @brief  Kernel that works on original data point
 */
typedef TwoViewKernel<Homography4PSolver, HomographyAsymmetricError, Mat3Model> Homography4PKernel;

/**
 * @brief By default use the normalized version for increased robustness.
 */
typedef NormalizedTwoViewKernel<Homography4PSolver, HomographyAsymmetricError, UnnormalizerI, Mat3Model> NormalizedHomography4PKernel;

}  // namespace relativePose
}  // namespace multiview
}  // namespace aliceVision
