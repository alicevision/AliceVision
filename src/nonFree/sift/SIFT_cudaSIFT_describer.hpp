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
    const bool print_dev_info = false;

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
    return _params.setPreset(preset);
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
        for( int y=0; y<h; y++ ) {
            h_image[y*w+x] = image(y,x);
        }

    CudaImage img;
    img.Allocate( w, h, iAlignUp(w, 128), false, 0, h_image );
    img.Download( );

    // int num_scales = 3,
    // float peak_threshold = 0.04f,
    // // std::size_t maxTotalKeypoints = 1000,

    SiftData siftData;
    int      numOctaves = _params._num_octaves; // default in CudaSift is 5
    // float    peak_threshold = 3.4f; // TEMP, initial 3.5. I put it to 3.4f because = 255 * 0.04 / 3 where 3 the number of levels/octave 
                                    // cf. l. 200, nonFree/sift/sift.hpp,
                                    // Same for openCV: ./xfeatures2d/src/sift.cpp: l. 383 + l. 456
    float    peak_threshold = 255.0f * _params._peak_threshold / 3.0f;
    float    initBlur = 0.5f * pow (2.0, -_params._first_octave); // -1: 1.0f, 0: 0.5f, 1:0.25f, etc. as not computed in the library

    float    lowestScale = 0.0f;
    // float    edgeLimit   = _params._edge_treshold; - ignore - this is always 10.0f in CudaSift
    bool     scaleUp = ( _params._first_octave == -1 ?  true : false );
    bool     allocSiftPointsHost = true;
    bool     allocSiftPointsDev  = true;
    InitSiftData( siftData,
                  _params._maxTotalKeypoints, // Should be custom
                  allocSiftPointsHost,
                  allocSiftPointsDev );
    ExtractSift( siftData,
                 img,
                 numOctaves,
                 initBlur,
                 peak_threshold,
                 lowestScale,
                 scaleUp );

    // get info out

    Allocate(regions); 

    // Build alias to cached data
    SIFT_Regions * regionsCasted = dynamic_cast<SIFT_Regions*>(regions.get()); 
    regionsCasted->Features().reserve( siftData.numPts );
    regionsCasted->Descriptors().reserve( siftData.numPts );

    // std::cout << "cudaSift features: " << siftData.numPts << std::endl;

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

    delete [] h_image;

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

