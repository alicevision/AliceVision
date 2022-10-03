// This file is part of the AliceVision project.
// Copyright (c) 2019 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/image/Image.hpp>
#include <aliceVision/image/pixelTypes.hpp>

namespace aliceVision {

/**
* @brief Calculate the difference between images of different sizes
* @param [inImgDownscaled] the smaller image
* @param [outImg] the difference
* @param [downscale] the downscale coefficient between image sizes
*/
void imageDiff(const image::Image<image::RGBfColor>& inImg,
               const image::Image<image::RGBfColor>& inImgDownscaled,
               image::Image<image::RGBfColor>& outImg, unsigned int downscale);

/**
* @brief Calculate the laplacian pyramid of a given image,
*        ie. its decomposition in frequency bands
* @param [out_pyramidL] the laplacian pyramid
* @param [nbBand] the number of frequency bands
* @param [downscale] the downscale coefficient between floors of the pyramid
*/
void laplacianPyramid(std::vector<image::Image<image::RGBfColor>>& out_pyramidL,
                      const image::Image<image::RGBfColor>& image, int nbBand, unsigned int downscale);

}

