// This file is part of the AliceVision project.
// Copyright (c) 2020 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "imageOps.hpp"
#include "gaussian.hpp"
#include "feathering.hpp"

namespace aliceVision {

void removeNegativeValues(image::Image<image::RGBfColor>& img)
{
    for (int i = 0; i < img.height(); i++)
    {
        for (int j = 0; j < img.width(); j++)
        {
            image::RGBfColor rpix;
            image::RGBfColor ret = img(i, j);

            rpix.r() = std::exp(ret.r());
            rpix.g() = std::exp(ret.g());
            rpix.b() = std::exp(ret.b());

            if (rpix.r() < 0.0)
            {
                ret.r() = 0.0;
            }

            if (rpix.g() < 0.0)
            {
                ret.g() = 0.0;
            }

            if (rpix.b() < 0.0)
            {
                ret.b() = 0.0;
            }

            img(i, j) = ret;
        }
    }
}

}  // namespace aliceVision