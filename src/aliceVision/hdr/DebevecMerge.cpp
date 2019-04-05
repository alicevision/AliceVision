// This file is part of the AliceVision project.
// Copyright (c) 2019 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "DebevecMerge.hpp"
#include <cassert>
#include <cmath>
#include <limits>
#include <iostream>
#include <aliceVision/alicevision_omp.hpp>

namespace aliceVision {
namespace hdr {

void DebevecMerge::process(const std::vector< image::Image<image::RGBfColor> > &images,
                              const std::vector<float> &times,
                              const rgbCurve &weight,
                              const rgbCurve &response,
                              image::Image<image::RGBfColor> &radiance,
                              float targetTime)
{

  std::cout << "Debevec merge" << std::endl;

  //checks
  assert(!response.isEmpty());
  assert(!images.empty());
  assert(images.size() == times.size());

  //reset radiance image
  radiance.fill(image::RGBfColor(0.f, 0.f, 0.f));

  //get images width, height
  const std::size_t width = images.front().Width();
  const std::size_t height = images.front().Height();

  #pragma omp parallel for
  for(std::size_t y = 0; y < height; ++y)
  {
    for(std::size_t x = 0; x < width; ++x)
    {
      //for each pixels
      image::RGBfColor &radianceColor = radiance(y, x);

      for(std::size_t channel = 0; channel < 3; ++channel)
      {
        double wsum = 0.0f;
        double wdiv = 0.0f;

        for(std::size_t i = 0; i < images.size(); ++i)
        {
          //for each images
          const double value = images[i](y, x)(channel);
          const double time = times[i];
          const double w = weight(value, channel) + 0.001;
          const float r = std::exp(response(value, channel));

//          const double time = std::log(times[i]);
//          const float r = response(value, channel);

          wsum += w * r / time;
          wdiv += w;

//          wsum += w * (r - time);

        }

        if(wdiv > 0.0001f)
        {
            radianceColor(channel) = (wsum / wdiv)*targetTime;

//          radianceColor(channel) = std::exp(wsum / wdiv)*targetTime;
        }
        else
        {
          radianceColor(channel) = 1.0f;
        }
      }
    }
  }
}

} // namespace hdr
} // namespace aliceVision
