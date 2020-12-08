// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "ImageDescriber_CCTAG.hpp"
#include <aliceVision/gpu/gpu.hpp>
#include <aliceVision/system/Logger.hpp>

#include <cctag/ICCTag.hpp>
#include <cctag/utils/LogTime.hpp>
//#define CPU_ADAPT_OF_GPU_PART //todo: #ifdef depreciated
#ifdef CPU_ADAPT_OF_GPU_PART    
  #include "cctag/progBase/MemoryPool.hpp"
#endif

namespace aliceVision {
namespace feature {

ImageDescriber_CCTAG::CCTagParameters::CCTagParameters(size_t nRings)
  : _internalParams(new cctag::Parameters(nRings))
{
#ifdef CCTAG_WITH_CUDA // CCTAG_WITH_CUDA
  _internalParams->_useCuda = gpu::gpuSupportCUDA(3,5);
#else
  _internalParams->_useCuda = false;
#endif
}

bool ImageDescriber_CCTAG::CCTagParameters::setPreset(EImageDescriberPreset preset)
{
  switch(preset)
  {
  // Normal lighting conditions: normal contrast
  case EImageDescriberPreset::LOW:
  case EImageDescriberPreset::MEDIUM:
  case EImageDescriberPreset::NORMAL:
    _cannyThrLow = 0.01f;
    _cannyThrHigh = 0.04f;
  break;
  // Low lighting conditions: very low contrast
  case EImageDescriberPreset::HIGH:
  case EImageDescriberPreset::ULTRA:
    _cannyThrLow = 0.002f;
    _cannyThrHigh = 0.01f;
  break;
  }
  return true;
}


ImageDescriber_CCTAG::ImageDescriber_CCTAG(const std::size_t nRings)
  : ImageDescriber()
  , _params(nRings)
  {}


bool ImageDescriber_CCTAG::useCuda() const
{
  return _params._internalParams->_useCuda;
}

void ImageDescriber_CCTAG::allocate(std::unique_ptr<Regions> &regions) const
{
  regions.reset( new CCTAG_Regions );
}

void ImageDescriber_CCTAG::setConfigurationPreset(ConfigurationPreset preset)
{
  _params.setPreset(preset.descPreset);
}

EImageDescriberType ImageDescriber_CCTAG::getDescriberType() const
{
  if(_params._internalParams->_nCrowns == 3)
    return EImageDescriberType::CCTAG3;
  if(_params._internalParams->_nCrowns == 4)
    return EImageDescriberType::CCTAG4;
  throw std::out_of_range("Invalid number of rings, can't find CCTag imageDescriber type !");
}

void ImageDescriber_CCTAG::setUseCuda(bool useCuda)
{
  _params._internalParams->_useCuda = useCuda;
}

bool ImageDescriber_CCTAG::describe(const image::Image<unsigned char>& image,
    std::unique_ptr<Regions> &regions,
    const image::Image<unsigned char> * mask)
{

  if ( !_doAppend )
    allocate(regions);
  // else regions are added to the current vector of features/descriptors

  // Build alias to cached data
  CCTAG_Regions * regionsCasted = dynamic_cast<CCTAG_Regions*>(regions.get());
  // reserve some memory for faster keypoint saving

  regionsCasted->Features().reserve(regionsCasted->Features().size() + 50);
  regionsCasted->Descriptors().reserve(regionsCasted->Descriptors().size() + 50);

  boost::ptr_list<cctag::ICCTag> cctags;
  cctag::logtime::Mgmt* durations = new cctag::logtime::Mgmt( 25 );
  // cctag::CCTagMarkersBank bank(_params._nCrowns);

#ifndef CPU_ADAPT_OF_GPU_PART
  const cv::Mat graySrc(cv::Size(image.Width(), image.Height()), CV_8UC1, (unsigned char *) image.data(), cv::Mat::AUTO_STEP);
  //// Invert the image
  //cv::Mat invertImg;
  //cv::bitwise_not(graySrc,invertImg);
  cctag::cctagDetection(cctags, _cudaPipe, 1,graySrc, *_params._internalParams, durations);
#else //todo: #ifdef depreciated
  cctag::MemoryPool::instance().updateMemoryAuthorizedWithRAM();
  cctag::View cctagView((const unsigned char *) image.data(), image.Width(), image.Height(), image.Depth()*image.Width());
  cctag::cctagDetection(cctags, _cudaPipe, 1 ,cctagView._grayView ,*_params._internalParams, durations );
#endif
  durations->print( std::cerr );

  // There is no notion of orientation in CCTag
  const float orientation = 0.0f;

  for (const auto & cctag : cctags)
  {
    if ( cctag.getStatus() > 0 )
    {
      ALICEVISION_LOG_DEBUG(" New CCTag: Id" << cctag.id() << " ; Location ( " << cctag.x() << " , " << cctag.y() << " ) ");

      // Add its associated descriptor
      Descriptor<unsigned char,128> desc;
      for(std::size_t i=0; i< desc.size(); ++i)
      {
        desc[i] = (unsigned char) 0;
      }
      desc[cctag.id()] = (unsigned char) 255;
      regionsCasted->Descriptors().push_back(desc);
      const float scale = std::max(cctag.rescaledOuterEllipse().a(), cctag.rescaledOuterEllipse().b());
      regionsCasted->Features().push_back(PointFeature(cctag.x(), cctag.y(), scale, orientation));
    }
  }

  cctags.clear();

  return true;
}

} // namespace feature
} // namespace aliceVision

