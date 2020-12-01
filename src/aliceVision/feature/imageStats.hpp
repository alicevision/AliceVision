// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/image/Image.hpp>


namespace aliceVision {
namespace feature {

/**
 * @brief Estimate a contrast factor using the percentile of the image gradients.
 *
 * @param[in] image Input image for the given octave
 * @param[in] percentile
 * @return contrastFactor
 */
float computeAutomaticContrastFactor(const image::Image<float>& image, const float percentile);

}  // namespace feature
}  // namespace aliceVision
