// This file is part of the AliceVision project.
// Copyright (c) 2020 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "gaussian.hpp"

#include <OpenImageIO/imagebufalgo.h>

namespace aliceVision {

GaussianPyramidNoMask::GaussianPyramidNoMask(const size_t width_base, const size_t height_base, const size_t limit_scales)
  : _width_base(width_base),
    _height_base(height_base)
{
    /**
     * Compute optimal scale
     * The smallest level will be at least of size min_size
     */
    size_t min_dim = std::min(_width_base, _height_base);
    size_t min_size = 32;
    _scales = std::min(limit_scales, static_cast<size_t>(floor(log2(double(min_dim) / float(min_size)))));

    /**
     * Create pyramid
     **/
    size_t new_width = _width_base;
    size_t new_height = _height_base;
    for (int i = 0; i < _scales; i++)
    {
        _pyramid_color.push_back(image::Image<image::RGBfColor>(new_width, new_height, true, image::RGBfColor(0)));
        _filter_buffer.push_back(image::Image<image::RGBfColor>(new_width, new_height, true, image::RGBfColor(0)));
        new_height /= 2;
        new_width /= 2;
    }
}

bool GaussianPyramidNoMask::process(const image::Image<image::RGBfColor>& input)
{
    if (input.height() != _pyramid_color[0].height())
        return false;
    if (input.width() != _pyramid_color[0].width())
        return false;

    /**
     * Build pyramid
     */
    _pyramid_color[0] = input;
    for (int lvl = 0; lvl < _scales - 1; lvl++)
    {
        const image::Image<image::RGBfColor>& source = _pyramid_color[lvl];
        image::Image<image::RGBfColor>& dst = _filter_buffer[lvl];

        convolveGaussian5x5(dst, source);
        downscale(_pyramid_color[lvl + 1], _filter_buffer[lvl]);
    }

    return true;
}

bool GaussianPyramidNoMask::downscale(image::Image<image::RGBfColor>& output, const image::Image<image::RGBfColor>& input)
{
    for (int i = 0; i < output.height(); i++)
    {
        int ui = i * 2;

        for (int j = 0; j < output.width(); j++)
        {
            int uj = j * 2;

            output(i, j) = input(ui, uj);
        }
    }

    return true;
}

}  // namespace aliceVision
