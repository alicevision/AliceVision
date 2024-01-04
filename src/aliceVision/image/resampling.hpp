// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "io.hpp"

#include <aliceVision/image/Sampler.hpp>
#include <aliceVision/system/Logger.hpp>

#include <OpenImageIO/imageio.h>
#include <OpenImageIO/imagebuf.h>
#include <OpenImageIO/imagebufalgo.h>

namespace oiio = OIIO;

namespace aliceVision {
namespace image {

/**
 * @brief Downscale an image using a given type of sampler.
 * @param[in] src source image to downscale
 * @param[out] out image to store the downscaled result
 * @param[in] downscale downscale value
 */
template<typename SamplerType, typename Image>
void downscaleImage(const Image& src, Image& out, int downscale)
{
    const int newWidth = src.width() / downscale;
    const int newHeight = src.height() / downscale;

    out.resize(newWidth, newHeight);

    const Sampler2d<SamplerType> sampler;
    const float downscalef = downscale;
    for (int i = 0; i < newHeight; ++i)
    {
        for (int j = 0; j < newWidth; ++j)
        {
            // Use .5f offset to ensure mid pixel and correct sampling
            out(i, j) = sampler(src, downscalef * (i + .5f), downscalef * (j + .5f));
        }
    }
}

/**
 ** Half sample an image (ie reduce its size by a factor 2) using bilinear interpolation
 ** @param[in] src input image
 ** @param[out] out output image
 **/
template<typename Image>
void imageHalfSample(const Image& src, Image& out)
{
    downscaleImage<SamplerLinear, Image>(src, out, 2);
}

/**
 ** @brief Resample an image using given sampling positions
 ** @param src Input image
 ** @param samplingPos A list of coordinates where the image needs to be resampled (samples are (Y,X) )
 ** @param outputWidth Width of the output image.
 ** @param outputHeight Height of the output image
 ** @param samplingFunc Resampling functor used to sample the Input image
 ** @param[out] out Output image
 ** @note samplingPos.size() must be equal to output_width * output_height
 **/
template<typename Image, typename ResamplingFunctor>
void genericResample(const Image& src,
                     const std::vector<std::pair<float, float>>& samplingPos,
                     const int outputWidth,
                     const int outputHeight,
                     const ResamplingFunctor& samplingFunc,
                     Image& out)
{
    assert(samplingPos.size() == outputWidth * outputHeight);

    out.resize(outputWidth, outputHeight);

    std::vector<std::pair<float, float>>::const_iterator itPos = samplingPos.begin();

    for (int i = 0; i < outputHeight; ++i)
    {
        for (int j = 0; j < outputWidth; ++j, ++itPos)
        {
            const float inputX = itPos->second;
            const float inputY = itPos->first;

            out(i, j) = samplingFunc(src, inputY, inputX);
        }
    }
}

}  // namespace image
}  // namespace aliceVision
