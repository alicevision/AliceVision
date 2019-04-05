// This file is part of the AliceVision project.
// Copyright (c) 2019 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "RobertsonCalibrate.hpp"
#include <iostream>
#include <fstream>
#include <cassert>
#include <aliceVision/alicevision_omp.hpp>

namespace aliceVision {
namespace hdr {

void RobertsonCalibrate::process(const std::vector< std::vector< image::Image<image::RGBfColor> > > &ldrImageGroups,
                                 const std::vector< std::vector<float> > &times,
                                 const rgbCurve &weight,
                                 rgbCurve &response,
                                 float targetTime)
{
  //checks
  for (int g = 0; g < ldrImageGroups.size(); ++g)
  {
    assert(ldrImageGroups[g].size() == times[g].size());
  }

  //set channels count always RGB
  static const std::size_t channels = 3;

  //get channels quantization
  const std::size_t channelQuantization = std::pow(2, 12); //RAW 12 bit precision, 2^12 values between black and white point

  //create radiance vector of image
  _radiance = std::vector< image::Image<image::RGBfColor> >(ldrImageGroups.size());
  for(auto& radianceImg: _radiance)
  {
    radianceImg.resize(ldrImageGroups[0][0].Width(), ldrImageGroups[0][0].Height(), false);
  }

  //initialize response
  response = rgbCurve(channelQuantization);
  response.setLinear();
  response.normalize();

  //initialize cardinality
  rgbCurve card(channelQuantization);
  card.setZero();

  //compute cardinal curve
  for(unsigned int g = 0; g < ldrImageGroups.size(); ++g)
  {
    const std::vector< image::Image<image::RGBfColor> > &ldrImagesGroup = ldrImageGroups[g];

    for(unsigned int i = 0; i < ldrImagesGroup.size(); ++i) 
    {
      const image::Image<image::RGBfColor> &image = ldrImagesGroup[i];

      //for each pixel
      for(std::size_t y = 0; y < image.Height(); ++y)
      {
        for(std::size_t x = 0; x < image.Width(); ++x)
        {
          const image::RGBfColor &pixelValue = image(y, x);
          
          for(std::size_t channel = 0; channel < channels; ++channel) 
          {
            //number of pixel with the same value 
            card(pixelValue(channel), channel) += 1;
            
          }
        }
      }
    }
  }
  card.interpolateMissingValues();
  
  //inverse cardinal curve value (for optimized division in the loop)
  card.inverseAllValues();

  //create merge operator
  RobertsonMerge merge;

  for(std::size_t iter = 0; iter < _maxIteration; ++iter) 
  {
    std::cout << "--> iteration : "<< iter << std::endl;

    std::cout << "1) compute radiance "<< std::endl;
    //initialize radiance
    for(std::size_t g = 0; g < ldrImageGroups.size(); ++g)
    {
      merge.process(ldrImageGroups[g], times[g], weight, response, _radiance[g], targetTime);
    }

    std::cout << "2) initialization new response "<< std::endl;
    //initialize new response
    rgbCurve newResponse = rgbCurve(channelQuantization);
    newResponse.setZero();

    std::cout << "3) compute new response "<< std::endl;
    //compute new response
    for(unsigned int g = 0; g < ldrImageGroups.size(); ++g)
    {
      const std::vector< image::Image<image::RGBfColor> > &ldrImagesGroup = ldrImageGroups[g];
      const image::Image<image::RGBfColor> &radiance = _radiance[g];

      for(unsigned int i = 0; i < ldrImagesGroup.size(); ++i) 
      {
        #pragma omp parallel for
        for(std::size_t y = 0; y < ldrImagesGroup[i].Height(); ++y)
        {
          for(std::size_t x = 0; x < ldrImagesGroup[i].Width(); ++x)
          {
            //for each pixels
            const image::RGBfColor &pixelValue = ldrImagesGroup[i](y, x);
            const image::RGBfColor &radianceValue = radiance(y, x);

            for(std::size_t channel = 0; channel < channels; ++channel)
            {
                newResponse(pixelValue(channel), channel) += times[g][i] * (radianceValue(channel));
            }
          }
        }
      }
    }
    newResponse.interpolateMissingValues();
    //dividing the response by the cardinal curve
    newResponse *= card;
    
    std::cout << "4) normalize response"<< std::endl;
    //normalization
    newResponse.normalize();
    
    std::cout << "5) compute difference"<< std::endl;    
    //calculate difference between the old response and the new one
    rgbCurve responseDiff = newResponse - response;
    responseDiff.setAllAbsolute();

    double diff = rgbCurve::sumAll(responseDiff) / channels;

    //update the response
    response = newResponse;
    
    std::cout << "6) check end condition"<< std::endl; 
    //check end condition
    if(diff < _threshold) 
    {
      std::cout << "[BREAK] difference < threshold " << std::endl;
      break;
    }
    std::cout << "-> difference is " << diff << std::endl;
  }
}

} // namespace hdr
} // namespace aliceVision
