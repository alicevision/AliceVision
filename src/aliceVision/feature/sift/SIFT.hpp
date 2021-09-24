// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/feature/Descriptor.hpp>
#include <aliceVision/feature/ImageDescriber.hpp>
#include <aliceVision/feature/regionsFactory.hpp>
#include <aliceVision/config.hpp>
#include <aliceVision/system/Logger.hpp>

#include <aliceVision/feature/imageStats.hpp>

extern "C" {
#include "nonFree/sift/vl/sift.h"
}

#include <iostream>
#include <numeric>
#include <stdexcept>

namespace aliceVision {
namespace feature {

/**
 * Bibliography:
 * [1] R. Arandjelović, A. Zisserman.
 * Three things everyone should know to improve object retrieval. CVPR2012.
 */
struct SiftParams
{
  /// Use original image, or perform an upscale if == -1
  int _firstOctave = 0;
  /// Scales per octave
  int _numScales = 3;
  /// Max ratio of Hessian eigenvalues
  float _edgeThreshold = 10.0f;
  /// Min contrast
  float _peakThreshold = 0.005f;
  /// Min contrast (relative to variance median)
  float _relativePeakThreshold = 0.01f;
  EFeatureConstrastFiltering _contrastFiltering = EFeatureConstrastFiltering::GridSort;

  std::size_t _gridSize = 4;
  std::size_t _maxTotalKeypoints = 10000;
  /// see [1]
  bool _rootSift = true;
  
  virtual void setPreset(ConfigurationPreset preset);

  int getImageFirstOctave(int w, int h) const
  {
    return _firstOctave - (w * h <= 3000 * 2000 ? 1 : 0); // -1 to upscale for small resolutions
  }
};

// VLFeat Instance management
class VLFeatInstance
{
public:

  static void initialize();

  static void destroy();

private:
  static int nbInstances;
};

//convertSIFT
//////////////////////////////
template < typename TOut > 
inline void convertSIFT(
  const vl_sift_pix* descr,
  Descriptor<TOut,128> & descriptor,
  bool rootSift
  );

template <> 
inline void convertSIFT<float>(
  const vl_sift_pix* descr,
  Descriptor<float,128> &descriptor,
  bool rootSift)
{
  if(rootSift)
  {
    const float sum = std::accumulate(descr, descr + 128, 0.0f);
    for(int k = 0; k < 128; ++k)
      descriptor[k] = std::floor(512.f*sqrt(descr[k] / sum));
  }
  else
  {
    for(int k = 0; k < 128; ++k)
      descriptor[k] = std::floor(512.f*descr[k]);
  }
}

template <> 
inline void convertSIFT<unsigned char>(
  const vl_sift_pix* descr,
  Descriptor<unsigned char,128> & descriptor,
  bool rootSift)
{
  if (rootSift)
  {
    // rootsift = sqrt( sift / sum(sift) );
    const float sum = std::accumulate(descr, descr+128, 0.0f);
    for (int k=0;k<128;++k)
      descriptor[k] = static_cast<unsigned char>(512.f*sqrt(descr[k]/sum));
  }
  else
  {
    for (int k=0;k<128;++k)
      descriptor[k] = static_cast<unsigned char>(512.f*descr[k]);
  }
}

/**
 * @brief Get the total amount of RAM needed for a
 * feature extraction of an image of the given dimension.
 * @param[in] width The image width
 * @param[in] height The image height
 * @return total amount of memory needed
 */
std::size_t getMemoryConsumptionVLFeat(std::size_t width, std::size_t height, const SiftParams& params);

/**
 * @brief Extract SIFT regions (in float or unsigned char).
 *
 * @param image
 * @param regions
 * @param params
 * @param orientation
 * @param mask
 * @return
 */
template <typename T>
bool extractSIFT(const image::Image<float>& image,
    std::unique_ptr<Regions>& regions,
    const SiftParams& params,
    bool orientation,
    const image::Image<unsigned char>* mask);

} //namespace feature
} //namespace aliceVision
