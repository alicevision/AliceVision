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
    
    // Process SIFT computation
    cudaDeviceReset();

//struct imgStream {
//    int width;
//    int height;
//    pixel_uc *data_r;
//    pixel_uc *data_g;
//    pixel_uc *data_b;
//};
    imgStream inp;
    inp.width = w;
    inp.height = h;
    inp.data_r = new unsigned char [w*h];
    inp.data_g = nullptr; // inp.data_r; 
    inp.data_b = nullptr; //inp.data_r;;
    
    unsigned char *pixel = inp.data_r;
    for(int j=0; j<h; j++)
    {
      for(int i=0; i<w; i++)
      {
        *pixel++ = image(j, i);
      }
    }
    /* Parse user input */
    // TODO => how the image is packed ? read_gray(inputFilename, inp);

    const double sigma = 1.6;
    PopSift PopSift( _params._num_octaves,
                    _params._num_scales,
                    1,
                    _params._peak_threshold, // DoG threshold, FIXME: verify this is actually _peak_threshold we want
                    _params._edge_threshold,
                    sigma );

    PopSift.init( w, h );
    PopSift.execute(inp);

    Allocate(regions); 

    // Build alias to cached data
    SIFT_Float_Regions * regionsCasted = dynamic_cast<SIFT_Float_Regions*>(regions.get()); 
    regionsCasted->Features().reserve(2000);
    regionsCasted->Descriptors().reserve(2000);

    popart::Pyramid &pyramid = PopSift.pyramid();
    for( int o=0; o<_params._num_octaves; o++ )
    {
      popart::Pyramid::Octave &octave = pyramid.octave(o);
      // GPU to CPU memory
      octave.downloadDescriptor();

      std::vector<popart::ExtremumCandidate> candidates;
      std::vector<popart::Descriptor> descriptors;
      for( int s=0; s<_params._num_scales+3; s++ ) 
      {
          size_t count = octave.getExtremaCount(s);
          candidates.resize(count);
          descriptors.resize(count);
          octave.downloadToVector(s, candidates, descriptors); // This also does GPU to CPU memory transfert

          // position scale orientation
          for(auto &feat : candidates)
          {
            regionsCasted->Features().emplace_back(feat.xpos, feat.ypos, feat.sigma, feat.angle_from_bemap);
          }

          // Descriptor values
          for(auto &descIt : descriptors)
          {
            Descriptor<float, 128> desc;
            for(int j = 0; j < 128; j++)
            {
              desc[j] = descIt.features[j]; // FIXME: write correct convertion
            }
            regionsCasted->Descriptors().emplace_back(desc);
          }
        }
    }

    PopSift.uninit( );
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

