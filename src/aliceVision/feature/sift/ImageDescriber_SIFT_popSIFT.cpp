// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "ImageDescriber_SIFT_popSIFT.hpp"
#include <aliceVision/system/Logger.hpp>

// PopSIFT includes
#include <popsift/popsift.h>
#include <popsift/sift_pyramid.h>
#include <popsift/sift_octave.h>
#include <popsift/common/device_prop.h>

#include <atomic>

namespace aliceVision {
namespace feature {

std::unique_ptr<PopSift> ImageDescriber_SIFT_popSIFT::_popSift{nullptr};
std::atomic<int> ImageDescriber_SIFT_popSIFT::_instanceCounter{0};

void ImageDescriber_SIFT_popSIFT::setConfigurationPreset(ConfigurationPreset preset)
{
    _params.setPreset(preset);
    _popSift.reset(nullptr); // reset by describe method
}

bool ImageDescriber_SIFT_popSIFT::describe(const image::Image<float>& image,
                                      std::unique_ptr<Regions>& regions,
                                      const image::Image<unsigned char>* mask)
{
  if(_popSift == nullptr)
    resetConfiguration();

  std::unique_ptr<SiftJob> job(_popSift->enqueue(image.Width(), image.Height(), &image(0,0)));
  std::unique_ptr<popsift::Features> popFeatures(job->get());

  allocate(regions);

  // Build alias to cached data
  SIFT_Regions * regionsCasted = dynamic_cast<SIFT_Regions*>(regions.get());
  regionsCasted->Features().reserve(popFeatures->getDescriptorCount());
  regionsCasted->Descriptors().reserve(popFeatures->getDescriptorCount());

  ALICEVISION_LOG_TRACE("PopSIFT features count: " << popFeatures->getFeatureCount() << ", descriptors count: " << popFeatures->getDescriptorCount() << std::endl);

  for(const auto& popFeat: *popFeatures)
  {
    for(int orientationIndex = 0; orientationIndex < popFeat.num_ori; ++orientationIndex)
    {
      const popsift::Descriptor* popDesc = popFeat.desc[orientationIndex];
      Descriptor<unsigned char, 128> desc;

      //convertSIFT<unsigned char>(*popDesc, desc, _params._root_sift);
      // root sift is done inside popsift, so we only need to cast the result
      for (std::size_t k = 0; k < 128; ++k)
        desc[k] = static_cast<unsigned char>(popDesc->features[k]);

      regionsCasted->Features().emplace_back(
        popFeat.xpos,
        popFeat.ypos,
        popFeat.sigma,
        popFeat.orientation[orientationIndex]);

      regionsCasted->Descriptors().emplace_back(desc);
    }
  }

  ALICEVISION_LOG_TRACE("aliceVision PopSIFT feature count : " << regionsCasted->RegionCount() << std::endl);

  return true;
}

void ImageDescriber_SIFT_popSIFT::resetConfiguration()
{
  // destroy all allocations and reset all state
  // on the current device in the current process
  cudaDeviceReset();

  popsift::cuda::device_prop_t deviceInfo;
  deviceInfo.set(0, true); // use only the first device & print information

  // reset configuration
  popsift::Config config;
  config.setLevels(_params._numScales);
  config.setDownsampling(_params._firstOctave);
  config.setThreshold(_params._peakThreshold);
  config.setEdgeLimit(_params._edgeThreshold);
  config.setNormalizationMultiplier(9); // 2^9 = 512
  config.setNormMode(_params._rootSift ? popsift::Config::RootSift : popsift::Config::Classic);
  config.setFilterMaxExtrema(_params._maxTotalKeypoints);
  config.setFilterSorting(popsift::Config::LargestScaleFirst);

  _popSift.reset(new PopSift(config, popsift::Config::ExtractingMode, PopSift::FloatImages));
}

ImageDescriber_SIFT_popSIFT::ImageDescriber_SIFT_popSIFT(const SiftParams& params, bool isOriented)
    : ImageDescriber()
    , _params(params)
    , _isOriented(isOriented)
{
    _instanceCounter++;
}

ImageDescriber_SIFT_popSIFT::~ImageDescriber_SIFT_popSIFT()
{
    _instanceCounter--;

    if(_instanceCounter.load() == 0)
    {
        _popSift.reset(nullptr);
    }
}

} // namespace feature
} // namespace aliceVision
