// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/image/Image.hpp>
#include <aliceVision/image/Sampler.hpp>
#include <aliceVision/system/Logger.hpp>
#include <type_traits>

namespace aliceVision {
namespace image {

/**
 * @brief Fill pixels of an image such that dest(i, j) = input(map(i, j).y, map(i, j).x)
 * @param[in] src source image (of size H1,W1) to remap
 * @param[in] map image (of size H2, W2) containing coordinates in the source image
 * @param[in] fillColor the default pixel value if not possible to remap
 * @param[out] output image to store the result (will be resized to size H2, W2)
 */
template <typename P>
void remap(const Image<P> & source, const Image<Vec2> & map, const P & fillColor, Image<P> & output)
{
    if (output.width() != map.width() || output.height() != map.height())
    {
        output.resize(map.width(), map.height());
    }

    const image::Sampler2d<image::SamplerLinear> sampler;

    for (int i = 0; i < map.height(); i++)
    {
        for (int j = 0; j < map.width(); j++)
        {
            const Vec2 & coord = map(i, j);

            const double & x = coord.x();
            const double & y = coord.y();

            // pick pixel if it is in the image domain
            if (source.contains(y, x))
            {
                output(i, j) = sampler(source, y, x);
            }
            else 
            {
                output(i, j) = fillColor;
            }
        }
    }
}

/**
 * @brief Fill pixels of an image such that dest(i, j) = input(map(i, j).y, map(i, j).x)
 * @param[in] src source image (of size H1,W1) to remap
 * @param[in] map image (of size H2, W2) containing coordinates in the source image
 * @param[in] fillColor the default pixel value if not possible to remap
 * @param[out] output image to store the result (will be resized to size H2, W2)
 */
template <typename P>
void remapInter(const Image<P> & source, const Image<Vec2> & map, const P& fillColor, Image<P> & output)
{
    if (output.width() != map.width() || output.height() != map.height())
    {
        output.resize(map.width(), map.height());
    }

    int srcWidth = source.width();
    int srcHeight = source.height();

    const image::Sampler2d<image::SamplerLinear> sampler;

    for (int i = 0; i < map.height(); i++)
    {
        for (int j = 0; j < map.width(); j++)
        {
            const Vec2 & coord = map(i, j);

            const double & x = coord.x();
            const double & y = coord.y();
            
            double scaleX = 1.0;
            if (j < map.width() - 1 && j > 0) 
            {
                scaleX = std::abs(map(i, j + 1).x() - map(i, j - 1).x()) * 0.5;
            } 
            else if (j < map.width() - 1) 
            {
                scaleX = std::abs(map(i, j + 1).x() - x);
            } 
            else if (j > 0) 
            {
                scaleX = std::abs(x - map(i, j - 1).x());
            }
            
            double scaleY = 1.0;
            if (i < map.height() - 1 && i > 0) 
            {
                scaleY = std::abs(map(i + 1, j).y() - map(i - 1, j).y()) * 0.5;
            } 
            else if (i < map.height() - 1) 
            {
                scaleY = std::abs(map(i + 1, j).y() - y);
            } 
            else if (i > 0) 
            {
                scaleY = std::abs(y - map(i - 1, j).y());
            }
        
            const double max_scale = std::max(scaleX, scaleY);
            if (scaleX > 1.0 && scaleY > 1.0)
            {
                //We are doing downsampling here.
                //Use area interpolation to avoid aliasing

                double halfWidth = scaleX * 0.5;
                double halfHeight = scaleY * 0.5;

                double x1 = coord.x() - halfWidth;
                double x2 = coord.x() + halfWidth;
                double y1 = coord.y() - halfHeight;
                double y2 = coord.y() + halfHeight;

                int ix1 = static_cast<int>(x1);
                int ix2 = static_cast<int>(std::ceil(x2));
                int iy1 = static_cast<int>(y1);
                int iy2 = static_cast<int>(std::ceil(y2));

                //Not using sampler as the neigborhood size is dynamic

                P sum;
                sum.fill(0.0);

                double wsum = 0.0;

                for (int by = iy1; by < iy2; by++)
                {
                    if (by < 0 || by >= srcHeight)
                    {
                        continue;
                    }

                    //Compute vertical occupancy
                    // (by + 1) - y1
                    // or y2 - by
                    double yocc = std::max(0.0, std::min(double(by + 1), y2) - std::max(double(by), y1));

                    for (int bx = ix1; bx < ix2; bx++)
                    {
                        if (bx < 0 || bx >= srcWidth)
                        {
                            continue;
                        }

                        //Compute horizontal occupancy
                        // (bx + 1) - x1
                        // or x2 - bx
                        double xocc = std::max(0.0, std::min(double(bx + 1), x2) - std::max(double(bx), x1));
                        
                        const P & pix = source(by, bx);
                        double weight = yocc * xocc;
                        
                        if constexpr (std::is_same<P, RGBAfColor>())
                        {   
                            weight *= pix.a();
                        }

                        sum += pix * weight;
                        wsum += weight;
                    }
                }

                //Output pixel is the weighted mean of the 
                //Source pixels in the area covered by the output pixel.
                if (wsum > 0.0)
                {
                    sum /= wsum;
                }

                output(i, j) = sum;
            }
            else 
            {
                // Not downsampling, do bilinear interpolation

                // pick pixel if it is in the image domain
                if (source.contains(y, x))
                {
                    output(i, j) = sampler(source, y, x);
                }
                else 
                {
                    output(i, j) = fillColor;
                }
            }
        }
    }
}

}  // namespace image
}  // namespace aliceVision
