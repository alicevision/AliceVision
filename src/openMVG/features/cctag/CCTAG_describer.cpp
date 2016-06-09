#include "CCTAG_describer.hpp"

// #include <cctag/view.hpp>
#include <cctag/ICCTag.hpp>
// #include <cctag/logtime.hpp>
//#define CPU_ADAPT_OF_GPU_PART //todo: #ifdef depreciated
#ifdef CPU_ADAPT_OF_GPU_PART    
  #include "cctag/progBase/MemoryPool.hpp"
#endif

namespace openMVG {
namespace features {

struct CCTAG_Image_describer::CCTagParameters : public cctag::Parameters 
{
  CCTagParameters(size_t nRings) : cctag::Parameters(nRings) {}
  
  float _cannyThrLow;
  float _cannyThrHigh;
  
  bool setPreset(EDESCRIBER_PRESET preset)
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
};


CCTAG_Image_describer::CCTAG_Image_describer()
    :Image_describer(), _params(new CCTagParameters(3)), _doAppend(false), _frameCount(0)
{}
    
CCTAG_Image_describer::CCTAG_Image_describer(const std::size_t nRings, const bool doAppend)
    :Image_describer(), _params(new CCTagParameters(nRings)), _doAppend(doAppend), _frameCount(0)
{}   

CCTAG_Image_describer::~CCTAG_Image_describer() 
{
  delete _params;
}

void CCTAG_Image_describer::Allocate(std::unique_ptr<Regions> &regions) const
{
  regions.reset( new CCTAG_Regions );
}

bool CCTAG_Image_describer::Set_configuration_preset(EDESCRIBER_PRESET preset)
{
  return _params->setPreset(preset);
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
    // cctag::logtime::Mgmt* durations = new cctag::logtime::Mgmt( 25 );
    // cctag::CCTagMarkersBank bank(_params._nCrowns);
#ifndef CPU_ADAPT_OF_GPU_PART
    const cv::Mat graySrc(cv::Size(image.Width(), image.Height()), CV_8UC1, (unsigned char *) image.data(), cv::Mat::AUTO_STEP);
    //// Invert the image
    //cv::Mat invertImg;
    //cv::bitwise_not(graySrc,invertImg);
    cctag::cctagDetection(cctags,1,graySrc, *_params, 0 );
#else //todo: #ifdef depreciated
    cctag::MemoryPool::instance().updateMemoryAuthorizedWithRAM();
    cctag::View cctagView((const unsigned char *) image.data(), image.Width(), image.Height(), image.Depth()*image.Width());
    boost::ptr_list<cctag::ICCTag> cctags;
    cctag::cctagDetection(cctags, 1 ,cctagView._grayView ,*_params, 0 );
#endif
    // durations->print( std::cerr );

    const uint32_t tagbits = tagsToBitmask(cctags);
    for (const auto & cctag : cctags)
    {
      if ( cctag.getStatus() > 0 && cctag.id() < 32 && (tagbits & (1U << cctag.id())) )
      {
        std::cout << " New CCTag: Id" << cctag.id() << " ; Location ( " << cctag.x() << " , " << cctag.y() << " ) " << std::endl;

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

uint32_t CCTAG_Image_describer::updateLastSeen(const boost::ptr_list<cctag::ICCTag>& tags)
{
  uint32_t currTags = tagsToBitmask(tags);

  // No filtering if history is shorter than its size.
  if (_frameCount < _lastSeen.size()) {
    _lastSeen[_frameCount++] = currTags;
    return currTags;
  }

  for (size_t i = 1; i < _lastSeen.size(); ++i)
    _lastSeen[i-1] = _lastSeen[i];
  _lastSeen.back() = currTags;

  for (size_t i = 0; i < _lastSeen.size(); ++i)
    currTags &= _lastSeen[i];

  return currTags;
}

uint32_t CCTAG_Image_describer::tagsToBitmask(const boost::ptr_list<cctag::ICCTag>& tags)
{
  uint32_t bits = 0;
  for (const auto& tag: tags)
  if (tag.getStatus() > 0 && tag.id() < 32)
    bits |= 1U << tag.id();
  return bits;
}


} // namespace features
} // namespace openMVG

