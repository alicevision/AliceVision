
#ifndef SIFT_POPSIFT_DESCRIBER_HPP
#define	SIFT_POPSIFT_DESCRIBER_HPP

#include "SIFT_describer.hpp"
#include <openMVG/features/descriptor.hpp>
#include <openMVG/features/image_describer.hpp>
#include <openMVG/features/regions_factory.hpp>

// popsift include
#include <popsift/popsift.h>
#include <popsift/sift_pyramid.h>
#include <popsift/sift_octave.h>
#include <popsift/common/device_prop.h>

#include <cereal/cereal.hpp>

#include <iostream>
#include <numeric>


namespace openMVG {
namespace features {

class SIFT_popSIFT_describer : public Image_describer
{
public:
  SIFT_popSIFT_describer(const SiftParams & params = SiftParams(), bool bOrientationDummy = true )
    : Image_describer()
    , _params(params)
    /* , _bOrientation(bOrientation) */
    , _previousImageSize(0, 0)
  {
    std::cout << "SIFT_popSIFT_describer" << std::endl;

    // Process SIFT computation
    cudaDeviceReset();

    popsift::Config config;
    config.setOctaves(_params._num_octaves);
    config.setLevels(_params._num_scales);
    config.setDownsampling(_params._first_octave);
    config.setThreshold(  _params._peak_threshold);
    config.setEdgeLimit(  _params._edge_threshold);
    config.setUseRootSift(_params._root_sift);

    const bool print_dev_info = true;

    popsift::cuda::device_prop_t deviceInfo;
    deviceInfo.set( 0, print_dev_info );
    if( print_dev_info ) deviceInfo.print( );

    _popSift.reset( new PopSift(config) );
  }

  ~SIFT_popSIFT_describer()
  {
    if(_previousImageSize.first != 0)
      _popSift->uninit(0);
  }

  bool Set_configuration_preset(EDESCRIBER_PRESET preset)
  {
    switch(preset)
    {
    case LOW_PRESET:
      _params._peak_threshold = 0.04f;
      _params._first_octave = 2;
    break;
    case MEDIUM_PRESET:
      _params._peak_threshold = 0.04f;
      _params._first_octave = 1;
    break;
    case NORMAL_PRESET:
      _params._peak_threshold = 0.04f;
    break;
    case HIGH_PRESET:
      _params._peak_threshold = 0.01f;
    break;
    case ULTRA_PRESET:
      _params._peak_threshold = 0.01f;
      _params._first_octave = -1;
    break;
    default:
      return false;
    }
    return true;
  }

  /**
  @brief Detect regions on the image and compute their attributes (description)
  @param image Image.
  @param regions The detected regions and attributes (the caller must delete the allocated data)
  @param mask 8-bit gray image for keypoint filtering (optional).
     Non-zero values depict the region of interest.
  */
  bool Describe(const image::Image<unsigned char>& image,
    std::unique_ptr<Regions> &regions,
    const image::Image<unsigned char> * mask = NULL)
  {
    const bool print_time_info = true;
    
    if(_previousImageSize.first != image.Width() ||
       _previousImageSize.second != image.Height())
    {
      if(_previousImageSize.first != 0)
      {
        // Only call uninit if we already called init()
        _popSift->uninit(0);
      }
      _popSift->init( 0, image.Width(), image.Height(), print_time_info );
      _previousImageSize.first = image.Width();
      _previousImageSize.second = image.Height();
    }

    std::unique_ptr<popsift::Features> popFeatures(_popSift->execute( 0, &image(0,0), print_time_info ));

    Allocate(regions); 

    // Build alias to cached data
    SIFT_Regions * regionsCasted = dynamic_cast<SIFT_Regions*>(regions.get()); 
    regionsCasted->Features().reserve(popFeatures->getFeatureCount());
    regionsCasted->Descriptors().reserve(popFeatures->getDescriptorCount());

    std::cout << "popSIFT featurse: " << popFeatures->getFeatureCount() << std::endl;
    std::cout << "popSIFT featurse: " << popFeatures->getDescriptorCount() << std::endl;

    for(const auto& popFeat: *popFeatures)
    {
      for(int orientationIndex = 0; orientationIndex < popFeat.num_descs; ++orientationIndex)
      {
        const popsift::Descriptor* popDesc = popFeat.desc[orientationIndex];
        Descriptor<unsigned char, 128> desc;
        // convertSIFT<unsigned char>(*popDesc, desc);
        for (int k=0;k<128;++k)
          desc[k] = static_cast<unsigned char>(popDesc->features[k]);

        regionsCasted->Features().emplace_back(
          popFeat.xpos,
          popFeat.ypos,
          popFeat.sigma,
          popFeat.orientation[orientationIndex]);

        regionsCasted->Descriptors().emplace_back(desc);
      }
    }


    return true;
  }

  /// Allocate Regions type depending of the Image_describer
  void Allocate(std::unique_ptr<Regions> &regions) const
  {
      regions.reset( new SIFT_Regions );
  }

  template<class Archive>
  void serialize( Archive & ar )
  {
    ar(
     cereal::make_nvp("params", _params),
     cereal::make_nvp("bOrientation", true ));
  }

private:
  SiftParams _params;
  /* bool _bOrientation; */

  std::unique_ptr<PopSift> _popSift;
  std::pair<std::size_t, std::size_t> _previousImageSize;
};

} // namespace features
} // namespace openMVG

#include <cereal/types/polymorphic.hpp>
#include <cereal/archives/json.hpp>
CEREAL_REGISTER_TYPE_WITH_NAME(openMVG::features::SIFT_popSIFT_describer, "SIFT_popSIFT_describer");


#endif	/* SIFT_POPSIFT_DESCRIBER_HPP */
