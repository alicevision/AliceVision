// This file is part of the AliceVision project.
// Copyright (c) 2020 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "warper.hpp"
#include <aliceVision/half.hpp>

namespace aliceVision {

bool Warper::warp(const CoordinatesMap& map, const aliceVision::image::Image<image::RGBfColor>& source)
{
    /**
     * Copy additional info from map
     */
    _offset_x = map.getOffsetX();
    _offset_y = map.getOffsetY();
    _mask = map.getMask();

    const image::Sampler2d<image::SamplerLinear> sampler;
    const aliceVision::image::Image<Eigen::Vector2f>& coordinates = map.getCoordinates();

    /**
     * Create buffer
     * No longer need to keep a 2**x size
     */
    _color = aliceVision::image::Image<image::RGBfColor>(coordinates.width(), coordinates.height());

    /**
     * Simple warp
     */
    for (size_t i = 0; i < _color.height(); i++)
    {
        for (size_t j = 0; j < _color.width(); j++)
        {
            bool valid = _mask(i, j);
            if (!valid)
            {
                continue;
            }

            const Eigen::Vector2f& coord = coordinates(i, j);
            const image::RGBfColor pixel = sampler(source, coord(1), coord(0));

            _color(i, j) = pixel;
        }
    }

    return true;
}

bool GaussianWarper::warp(const CoordinatesMap& map, const GaussianPyramidNoMask& pyramid, bool clamp)
{
    /**
     * Copy additional info from map
     */
    _offset_x = map.getOffsetX();
    _offset_y = map.getOffsetY();
    _mask = map.getMask();

    const image::Sampler2d<image::SamplerLinear> sampler;
    const aliceVision::image::Image<Eigen::Vector2f>& coordinates = map.getCoordinates();

    /**
     * Create a pyramid for input
     */
    const std::vector<image::Image<image::RGBfColor>>& mlsource = pyramid.getPyramidColor();
    int max_level = pyramid.getScalesCount() - 1;

    /**
     * Create buffer
     */
    _color = aliceVision::image::Image<image::RGBfColor>(coordinates.width(), coordinates.height(), true, image::RGBfColor(1.0, 0.0, 0.0));

    /**
     * Multi level warp
     */
    for (size_t i = 0; i < _color.height(); i++)
    {
        int next_i = i + 1;

        if (i == _color.height() - 1)
        {
            next_i = i - 1;
        }

        for (size_t j = 0; j < _color.width(); j++)
        {
            bool valid = _mask(i, j);
            if (!valid)
            {
                continue;
            }

            int next_j = j + 1;

            if (j == _color.width() - 1)
            {
                next_j = j - 1;
            }

            if (!_mask(next_i, j) || !_mask(i, next_j))
            {
                const Eigen::Vector2f& coord = coordinates(i, j);
                const image::RGBfColor pixel = sampler(mlsource[0], coord(1), coord(0));
                _color(i, j) = pixel;

                continue;
            }

            const Eigen::Vector2f& coord_mm = coordinates(i, j);
            const Eigen::Vector2f& coord_mp = coordinates(i, next_j);
            const Eigen::Vector2f& coord_pm = coordinates(next_i, j);

            float dxx = coord_pm(0) - coord_mm(0);
            float dxy = coord_mp(0) - coord_mm(0);
            float dyx = coord_pm(1) - coord_mm(1);
            float dyy = coord_mp(1) - coord_mm(1);
            float det = std::abs(dxx * dyy - dxy * dyx);
            float scale = /*sqrtf*/ (det);  // logsqrt = 0.5log

            float flevel = std::max(0.0f, 0.5f * log2f(scale));
            int blevel = std::min(max_level, int(floor(flevel)));

            float dscale, x, y;
            dscale = pow(2.0, -blevel);
            x = coord_mm(0) * dscale;
            y = coord_mm(1) * dscale;

            /*Fallback to first level if outside*/
            if (x >= mlsource[blevel].width() - 1 || y >= mlsource[blevel].height() - 1)
            {
                _color(i, j) = sampler(mlsource[0], coord_mm(1), coord_mm(0));
                continue;
            }

            _color(i, j) = sampler(mlsource[blevel], y, x);

            if (clamp)
            {
                if (_color(i, j).r() > HALF_MAX)
                    _color(i, j).r() = HALF_MAX;
                if (_color(i, j).g() > HALF_MAX)
                    _color(i, j).g() = HALF_MAX;
                if (_color(i, j).b() > HALF_MAX)
                    _color(i, j).b() = HALF_MAX;
            }
        }
    }

    return true;
}

}  // namespace aliceVision
