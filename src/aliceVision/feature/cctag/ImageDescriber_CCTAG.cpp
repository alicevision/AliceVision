// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#include "ImageDescriber_CCTAG.hpp"

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
}

ImageDescriber_CCTAG::CCTagParameters::~CCTagParameters()
{
}

bool ImageDescriber_CCTAG::CCTagParameters::setPreset(EDESCRIBER_PRESET preset)
{
  switch(preset)
  {
  // Normal lighting conditions: normal contrast
  case LOW_PRESET:
  case MEDIUM_PRESET:
  case NORMAL_PRESET:
    _cannyThrLow = 0.01f;
    _cannyThrHigh = 0.04f;
  break;
  // Low lighting conditions: very low contrast
  case HIGH_PRESET:
  case ULTRA_PRESET:
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

ImageDescriber_CCTAG::~ImageDescriber_CCTAG()
{
}

void ImageDescriber_CCTAG::Allocate(std::unique_ptr<Regions> &regions) const
{
  regions.reset( new CCTAG_Regions );
}

bool ImageDescriber_CCTAG::Set_configuration_preset(EDESCRIBER_PRESET preset)
{
  return _params.setPreset(preset);
}

void ImageDescriber_CCTAG::setUseCuda(bool useCuda)
{
  _params._internalParams->_useCuda = useCuda;
}

bool ImageDescriber_CCTAG::Describe(const image::Image<unsigned char>& image,
    std::unique_ptr<Regions> &regions,
    const image::Image<unsigned char> * mask)
  {
    
    if ( !_doAppend )
      Allocate(regions);  
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
    boost::ptr_list<cctag::ICCTag> cctags;
    cctag::cctagDetection(cctags, _cudaPipe, 1 ,cctagView._grayView ,*_params._internalParams, durations );
#endif
    durations->print( std::cerr );
    
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
        regionsCasted->Features().push_back(SIOPointFeature(cctag.x(), cctag.y()));
      }
    }

    cctags.clear();

    return true;
  };

} // namespace feature
} // namespace aliceVision

