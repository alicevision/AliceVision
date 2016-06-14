
#ifndef SIFT_POPSIFT_DESCRIBER_HPP
#define	SIFT_POPSIFT_DESCRIBER_HPP

#include "SIFT_describer.hpp"
#include <openMVG/features/descriptor.hpp>
#include <openMVG/features/image_describer.hpp>
#include <openMVG/features/regions_factory.hpp>

// popsift include
#include <popsift/c_util_img.h>
#include <popsift/popsift.h>
#include <popsift/sift_pyramid.h>
#include <popsift/sift_octave.h>

#include <cereal/cereal.hpp>

#include <iostream>
#include <numeric>


extern char *debug_file_name; // FIXME: get rid of debug_file_name, it is used in popsift and
                              // openMVG_Samples/main_repeatability_dataset.cpp for quick 
                              // debugging puposes 
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

    imgStream inp;
    inp.width = w;
    inp.height = h;
    inp.data_r = new unsigned char [w*h];
    inp.data_g = nullptr;
    inp.data_b = nullptr;
    
    unsigned char *pixel = inp.data_r;
    for(int j=0; j<h; j++)
    {
      for(int i=0; i<w; i++)
      {
        *pixel++ = image(j, i);
      }
    }

    const double sigma = 1.6;
    
    popart::Config config;
    config.setOctaves(_params._num_octaves);
    config.setLevels(_params._num_scales);
    config.setDownsampling(-1);
    config.setThreshold( 1.0f ); // _params._peak_threshold);
    config.setEdgeLimit( 0.8f ); // _params._edge_threshold);
    config.setSigma(sigma);

    
    PopSift PopSift(config);
    

    PopSift.init( 0, w, h );
    PopSift.execute(0, &inp);

    Allocate(regions); 

    // Build alias to cached data
    SIFT_Float_Regions * regionsCasted = dynamic_cast<SIFT_Float_Regions*>(regions.get()); 
    regionsCasted->Features().reserve(10000);
    regionsCasted->Descriptors().reserve(10000);

    popart::Pyramid &pyramid = PopSift.pyramid(0);
    for( int o=0; o<_params._num_octaves; o++ )
    {
      popart::Octave &octave = pyramid.octave(o);
      // GPU to CPU memory
      // TODO: the download GPU -> CPU is also done in downloadToVector,
      // check that we can safely remove downloadDescriptors
      octave.downloadDescriptor();

      const float imageScale = static_cast<float>(1 << o);
      std::vector<popart::Extremum> candidates;
      std::vector<popart::Descriptor> descriptors;
      for( int s=0; s<_params._num_scales+3; s++ ) 
      {
          
          size_t count = octave.getExtremaCount(s);
          std::cerr << "Octave " << o << " level " << s
                    << " #extremas " << count << std::endl;
          
          candidates.resize(count);
          descriptors.resize(count);
          octave.downloadToVector(s, candidates, descriptors); // This also does GPU to CPU memory transfert

          // DEBUGGING:
          //octave.download_and_save_array(debug_file_name, o, s);

          // position scale orientation
          for(auto &feat : candidates)
          {
            // the position of the points we get from popsift are multiplied by two
            // the 0.5 value is used to move their coordinates to the correct image coordinates
            regionsCasted->Features().emplace_back(0.5*feat.xpos*imageScale, 0.5*feat.ypos*imageScale, feat.sigma, feat.orientation);
          }

          // Descriptor values
          for(auto &descIt : descriptors)
          {
            Descriptor<float, 128> desc;
            for(int j = 0; j < 128; j++)
            {
              desc[j] = descIt.features[j]; // FIXME: ensure the conversion is correct 
            }
            regionsCasted->Descriptors().emplace_back(desc);
          }
        }
    }
    // FIXME: do we have to deallocate the image allocated line 78 ?

    PopSift.uninit(0);
    // FIXME: remove following commented code once the debug is finished
    //regions->Save("/tmp/features_popsift.txt", "/tmp/descriptors_popsift.txt");    
    //_exit(0);
    return true;  // <==
  }

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
