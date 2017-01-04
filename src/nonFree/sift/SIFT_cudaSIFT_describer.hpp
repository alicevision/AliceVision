#ifndef SIFT_CUDASIFT_DESCRIBER_HPP
#define	SIFT_CUDASIFT_DESCRIBER_HPP

#include "SIFT_describer.hpp"
#include <openMVG/features/descriptor.hpp>
#include <openMVG/features/image_describer.hpp>
#include <openMVG/features/regions_factory.hpp>

/* Two exported header files of Celebrandil's CudaSift */
#include <cudaImage.h>
#include <cudaSift.h>

#include <cereal/cereal.hpp>

#include <iostream>
#include <numeric>


namespace openMVG {
namespace features {

class SIFT_cudaSIFT_describer : public Image_describer
{
public:
  SIFT_cudaSIFT_describer(const SiftParams & params = SiftParams(), bool bOrientationDummy = true )
    : Image_describer()
    , _params(params)
  {

    std::cout << "SIFT_cudaSIFT_describer" << std::endl;

    const int  devNum         = 0; // Choose the CUDA device. This should be a parameter.
    const bool print_dev_info = true;

    popsift::cuda::device_prop_t deviceInfo;
    deviceInfo.set( devNum, print_dev_info );
    if( print_dev_info ) deviceInfo.print( );

    InitCuda( devNum );
  }

  ~SIFT_cudaSIFT_describer()
  {
  }

  bool Set_configuration_preset(EDESCRIBER_PRESET preset)
  {
    switch(preset)
    {
    case LOW_PRESET:
      _params._peak_threshold = 0.04f;
      _params._first_octave = 2;
      return false; // not supported by CudaSift
    case MEDIUM_PRESET:
      _params._peak_threshold = 0.04f;
      _params._first_octave = 1;
      return false; // not supported by CudaSift
    case NORMAL_PRESET:
      _params._peak_threshold = 0.04f;
      _params._first_octave = 0;
      break;
    case HIGH_PRESET:
      _params._peak_threshold = 0.01f;
      _params._first_octave = 0;
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
    int w = image.Width();
    int h = image.Height();

    float* h_image = new float[w*h];
    for( int x=0; x<w; x++ )
        for( int y=0; y<h; y++ )
            h_image[y*w+h] = image(y,x) / 255.0f;

    CudaImage img;
    img.Allocate( w, h, iAlignUp(w, 128), false, 0, h_image );
    img.Download( );
    delete [] h_image;

    // int num_scales = 3,
    // float peak_threshold = 0.04f,
    // // std::size_t maxTotalKeypoints = 1000,

    SiftData siftData;
    float    initBlur = 1.0f;
    float    lowestScale = 0.0f;
    bool     scaleUp = ( _params._first_octave == -1 ?  true : false );
    bool     allocSiftPointsHost = true;
    bool     allocSiftPointsDev  = true;
    InitSiftData( siftData,
                  _params._maxTotalKeypoints,
                  allocSiftPointsHost,
                  allocSiftPointsDev );
    ExtractSift( siftData,
                 img,
                 _params._num_octaves,
                 initBlur,
                 _params._edge_threshold,
                 lowestScale,
                 scaleUp );

    // get info out

    Allocate(regions); 

    // Build alias to cached data
    SIFT_Regions * regionsCasted = dynamic_cast<SIFT_Regions*>(regions.get()); 
    regionsCasted->Features().reserve( siftData.numPts );
    regionsCasted->Descriptors().reserve( siftData.numPts );

    std::cout << "cudaSift features: " << siftData.numPts << std::endl;

    for( int i=0; i<siftData.numPts; i++ )
    {
        // root sift is unknown
        const bool usingRootSift = false;

        const SiftPoint& point = siftData.h_data[i];

        // descriptor from Celebrandil CudaSift consists of floats, vl_sift_pix is
        // also float, so this is compatible
        Descriptor<unsigned char, 128> desc;
        convertSIFT<unsigned char>( point.data, desc, usingRootSift );

        regionsCasted->Features().emplace_back(
          point.xpos,
          point.ypos,
          point.scale,
          point.orientation );

        regionsCasted->Descriptors().emplace_back(desc);
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
};

} // namespace features
} // namespace openMVG

#include <cereal/types/polymorphic.hpp>
#include <cereal/archives/json.hpp>
CEREAL_REGISTER_TYPE_WITH_NAME(openMVG::features::SIFT_cudaSIFT_describer, "SIFT_cudaSIFT_describer");

#endif	/* SIFT_CUDASIFT_DESCRIBER_HPP */

