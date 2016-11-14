#include "CCTAG_describer.hpp"

#include <cctag/ICCTag.hpp>
#include <cctag/utils/LogTime.hpp>
//#define CPU_ADAPT_OF_GPU_PART //todo: #ifdef depreciated
#ifdef CPU_ADAPT_OF_GPU_PART    
  #include "cctag/progBase/MemoryPool.hpp"
#endif

namespace openMVG {
namespace features {

CCTAG_Image_describer::CCTagParameters::CCTagParameters(size_t nRings)
  : _internalParams(new cctag::Parameters(nRings))
{
}

CCTAG_Image_describer::CCTagParameters::~CCTagParameters()
{
}

bool CCTAG_Image_describer::CCTagParameters::setPreset(EDESCRIBER_PRESET preset)
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


CCTAG_Image_describer::CCTAG_Image_describer(const std::size_t nRings)
  : Image_describer()
  , _params(nRings)
  {}

CCTAG_Image_describer::~CCTAG_Image_describer() 
{
}

void CCTAG_Image_describer::Allocate(std::unique_ptr<Regions> &regions) const
{
  regions.reset( new CCTAG_Regions );
}

bool CCTAG_Image_describer::Set_configuration_preset(EDESCRIBER_PRESET preset)
{
  return _params.setPreset(preset);
}

void CCTAG_Image_describer::Set_use_cuda(bool use_cuda)
{
  _params._internalParams->_useCuda = use_cuda;
}

bool CCTAG_Image_describer::Describe(const image::Image<unsigned char>& image,
    std::unique_ptr<Regions> &regions,
    const image::Image<unsigned char> * mask)
  {
    const int w = image.Width(), h = image.Height();
    
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
        OPENMVG_LOG_DEBUG(" New CCTag: Id" << cctag.id() << " ; Location ( " << cctag.x() << " , " << cctag.y() << " ) ");

        // Add its associated descriptor
        Descriptor<unsigned char,128> desc;
        for(int i=0; i< desc.size(); ++i)
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

} // namespace features
} // namespace openMVG

