// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// Copyright (c) 2010 libmv contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/robustEstimation/PointFittingKernel.hpp>
#include <aliceVision/multiview/relativePose/Fundamental7PSolver.hpp>
#include <aliceVision/multiview/relativePose/Fundamental8PSolver.hpp>
#include <aliceVision/multiview/relativePose/FundamentalError.hpp>
#include <aliceVision/multiview/Unnormalizer.hpp>

namespace aliceVision {
namespace multiview {
namespace relativePose {

/**
 * @brief Kernel solver for the 8pt Fundamental Matrix Estimation
 */
using Fundamental7PKernel = robustEstimation::PointFittingKernel<Fundamental7PSolver, FundamentalSampsonError, robustEstimation::Mat3Model>;

/**
 * @brief Kernel solver for the 8pt Fundamental Matrix Estimation
 */
using Fundamental8PKernel = robustEstimation::PointFittingKernel<Fundamental8PSolver, FundamentalSampsonError, robustEstimation::Mat3Model>;

/**
 * @brief Normalized 7pt kernel
 * @see conditioning from HZ (Algo 11.1) pag 282
 */
using NormalizedFundamental7PKernel = robustEstimation::NormalizedPointFittingKernel<Fundamental7PSolver, FundamentalSampsonError, UnnormalizerT, robustEstimation::Mat3Model>;

/**
 * @brief Normalized 8pt kernel
 * @see conditioning from HZ (Algo 11.1) pag 282
 */
using NormalizedFundamental8PKernel = robustEstimation::NormalizedPointFittingKernel<Fundamental8PSolver, FundamentalSampsonError, UnnormalizerT, robustEstimation::Mat3Model>;

}  // namespace relativePose
}  // namespace multiview
}  // namespace aliceVision
