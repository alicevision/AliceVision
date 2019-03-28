// This file is part of the AliceVision project.
// Copyright (c) 2019 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "RobertsonMerge.hpp"
#include <cassert>
#include <cmath>
#include <limits>
#include <iostream>
#include <aliceVision/alicevision_omp.hpp>


namespace aliceVision {
namespace hdr {
  
void RobertsonMerge::process(const std::vector< image::Image<image::RGBfColor> > &images,
                              const std::vector<float> &times,
                              const rgbCurve &weight,
                              const rgbCurve &response,
                              image::Image<image::RGBfColor> &radiance,
                              float targetTime)
{
  //checks
  assert(!response.isEmpty());
  assert(!images.empty());
  assert(images.size() == times.size());
  
  //reset radiance image
  radiance.fill(image::RGBfColor(0.f, 0.f, 0.f));

  //get images width, height
  const std::size_t width = images.front().Width();
  const std::size_t height = images.front().Height();
  
  //min and max trusted values
  const float minTrustedValue = 0.0f - std::numeric_limits<float>::epsilon();
  const float maxTrustedValue = 1.0f + std::numeric_limits<float>::epsilon();
  
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
//        float minTimeSaturation = std::numeric_limits<float>::max();
//        float maxTimeSaturation = std::numeric_limits<float>::min();

        for(std::size_t i = 0; i < images.size(); ++i) 
        {
          //for each images
          const double value = images[i](y, x)(channel);
          const double time = times[i];
          const double vt = value / time;
          const double w = weight(value, channel) + 0.001;
          const float r = response(value, channel);

//          wsum += w * time * r;
//          wdiv += w * time * time;
          wsum += w * r / time;
          wdiv += w;
//          wsum += w * vt;
//          wdiv += w;

//          //saturation detection
//          if(value > maxTrustedValue) 
//          {
//            minTimeSaturation = std::min(minTimeSaturation, time);
//          }
//          
//          if(value < minTrustedValue) 
//          {
//            maxTimeSaturation = std::max(maxTimeSaturation, time);
//          }
        }

//        //saturation correction
//        if((wdiv == 0.0f) && 
//               (maxTimeSaturation > std::numeric_limits<float>::min())) 
//        {
//          wsum = minTrustedValue;
//          wdiv = maxTimeSaturation;
//        }
//        
//        if((wdiv == 0.0f) && 
//               (minTimeSaturation < std::numeric_limits<float>::max())) 
//        {
//          wsum = maxTrustedValue;
//          wdiv = minTimeSaturation;
//        }

        if(wdiv > 0.0001f) 
        {
          radianceColor(channel) = (wsum / wdiv) * targetTime;
        } 
        else
        {
          radianceColor(channel) = 0.0f;
        }
        
      } 
    }
  }
}


} // namespace hdr
} // namespace aliceVision
