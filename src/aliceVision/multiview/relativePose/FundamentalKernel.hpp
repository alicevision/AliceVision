// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// Copyright (c) 2010 libmv contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/multiview/TwoViewKernel.hpp>
#include <aliceVision/multiview/relativePose/Fundamental7PSolver.hpp>
#include <aliceVision/multiview/relativePose/Fundamental8PSolver.hpp>
#include <aliceVision/multiview/relativePose/FundamentalError.hpp>

namespace aliceVision {
namespace multiview {
namespace relativePose {

/**
 * @brief Kernel solver for the 8pt Fundamental Matrix Estimation
 */
typedef TwoViewKernel<Fundamental7PSolver, FundamentalSampsonError, Mat3Model> Fundamental7PKernel;

/**
 * @brief Kernel solver for the 8pt Fundamental Matrix Estimation
 */
typedef TwoViewKernel<Fundamental8PSolver, FundamentalSampsonError, Mat3Model> Fundamental8PKernel;

/**
 * @brief Normalized 7pt kernel
 * @see conditioning from HZ (Algo 11.1) pag 282
 */
typedef NormalizedTwoViewKernel<Fundamental7PSolver, FundamentalSampsonError, UnnormalizerT, Mat3Model> NormalizedFundamental7PKernel;

/**
 * @brief Normalized 8pt kernel
 * @see conditioning from HZ (Algo 11.1) pag 282
 */
typedef NormalizedTwoViewKernel<Fundamental8PSolver, FundamentalSampsonError, UnnormalizerT, Mat3Model> NormalizedFundamental8PKernel;

}  // namespace relativePose
}  // namespace multiview
}  // namespace aliceVision
