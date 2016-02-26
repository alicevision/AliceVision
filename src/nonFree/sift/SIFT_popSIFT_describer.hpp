#ifndef SIFT_POPSIFT_DESCRIBER_HPP
#define	SIFT_POPSIFT_DESCRIBER_HPP

#include "SIFT_describer.hpp"
#include <openMVG/features/descriptor.hpp>
#include <openMVG/features/image_describer.hpp>
#include <openMVG/features/regions_factory.hpp>

// popsift include
#include <SIFT.h>


#include <cereal/cereal.hpp>

#include <iostream>
#include <numeric>

namespace openMVG {
namespace features {

class SIFT_popSIFT_describer : public Image_describer
{
public:
  SIFT_popSIFT_describer(const SiftParams & params = SiftParams(), bool bOrientation = true)
    :Image_describer(), _params(params), _bOrientation(bOrientation) {}

  ~SIFT_popSIFT_describer() {}

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
    const int w = image.Width(), h = image.Height();

    //Convert to float
    const image::Image<float> If(image.GetMat().cast<float>()); // <== 

    Descriptor<float, 128> descr; // <== an openMVG descriptor

    // Process SIFT computation
    cudaDeviceReset();

    imgStream inp;

    /* Parse user input */
    // => how the image is packed ? read_gray(inputFilename, inp);

    const double sigma = 1.6;
    PopSift PopSift( _params._num_octaves,
                    _params._num_scales,
                    1,
                    _params._peak_threshold, // DoG threshold, FIXME: verify this is actually _peak_threshold
                    _params._edge_threshold,
                    sigma );

    PopSift.init( w, h );
    PopSift.execute(inp);
    PopSift.uninit( );

    Allocate(regions); // <== regions allocation

    // Build alias to cached data
    SIFT_Float_Regions * regionsCasted = dynamic_cast<SIFT_Float_Regions*>(regions.get()); // <== This is the container where the final result of the extraction is stored.
                // it consists in the aggregation of a vector of the extracted keypoints (a keypoint type named SIOPointFeature)
                // and a vector of their associated descriptors (a descriptor type named Descriptor)

    // reserve some memory for faster keypoint saving
    regionsCasted->Features().reserve(2000); // <== 
    regionsCasted->Descriptors().reserve(2000); // <==

    return true;  // <==
  };

  /// Allocate Regions type depending of the Image_describer
  void Allocate(std::unique_ptr<Regions> &regions) const
  {
      regions.reset( new SIFT_Float_Regions );
  }

  template<class Archive>
  void serialize( Archive & ar )
  {
    ar(
     cereal::make_nvp("params", _params),
     cereal::make_nvp("bOrientation", _bOrientation));
  }

private:
  SiftParams _params;
  bool _bOrientation;
};

} // namespace features
} // namespace openMVG

#include <cereal/types/polymorphic.hpp>
#include <cereal/archives/json.hpp>
CEREAL_REGISTER_TYPE_WITH_NAME(openMVG::features::SIFT_popSIFT_describer, "SIFT_popSIFT_describer");


#endif	/* SIFT_POPSIFT_DESCRIBER_HPP */

