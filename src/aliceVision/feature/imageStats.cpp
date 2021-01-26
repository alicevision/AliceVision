// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "imageStats.hpp"

#include <aliceVision/image/all.hpp>

namespace aliceVision {
namespace feature {

float computeAutomaticContrastFactor(const image::Image<float>& image, const float percentile)
{
    const size_t nbBins = 300;
    const int height = image.Height();
    const int width = image.Width();

    // smooth the image
    image::Image<float> smoothed;
    image::ImageGaussianFilter(image, 1.f, smoothed, 0, 0);

    // compute gradient
    image::Image<float> Lx, Ly;
    image::ImageScharrXDerivative(smoothed, Lx, false);
    image::ImageScharrYDerivative(smoothed, Ly, false);

    // reuse smoothed to avoid new allocation
    image::Image<float>& grad = smoothed;
    // grad = sqrt(Lx^2 + Ly^2)
    grad.array() = (Lx.array().square() + Ly.array().square()).sqrt();

    const float gradMax = grad.maxCoeff();

    // compute histogram
    std::vector<std::size_t> histo(nbBins, 0);

    int nbValues = 0;

    for(int i = 1; i < height - 1; ++i)
    {
        for(int j = 1; j < width - 1; ++j)
        {
            const float val = grad(i, j);

            if(val > 0)
            {
                int binId = floor((val / gradMax) * static_cast<float>(nbBins));

                // handle overflow (need to do it in a cleaner way)
                if(binId == nbBins)
                    --binId;

                // accumulate
                ++histo[binId];
                ++nbValues;
            }
        }
    }

    const std::size_t searchId = percentile * static_cast<float>(nbValues);

    int binId = 0;
    std::size_t acc = 0;

    while(acc < searchId && binId < nbBins)
    {
        acc += histo[binId];
        ++binId;
    }

    // handle 0 bin search
    if(acc < searchId)
        return 0.03f; // only empiric value

    return gradMax * static_cast<float>(binId) / static_cast<float>(nbBins);
}

}  // namespace feature
}  // namespace aliceVision
