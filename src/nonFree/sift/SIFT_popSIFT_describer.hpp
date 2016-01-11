#ifndef SIFT_POPSIFT_DESCRIBER_HPP
#define	SIFT_POPSIFT_DESCRIBER_HPP

#include "SIFT_describer.hpp"
#include <openMVG/features/descriptor.hpp>
#include <openMVG/features/image_describer.hpp>
#include <openMVG/features/regions_factory.hpp>

#include <cereal/cereal.hpp>

#include <iostream>
#include <numeric>

extern "C" {                         // should be removed
#include "nonFree/sift/vl/sift.h"    // should be removed
}                                    // should be removed

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
    const int w = image.Width(), h = image.Height(); // <== 
    //Convert to float
    const image::Image<float> If(image.GetMat().cast<float>()); // <== 

    // Configure VLFeat
    vl_constructor();

    VlSiftFilt *filt = vl_sift_new(w, h,
      _params._num_octaves, _params._num_scales, _params._first_octave);
    if (_params._edge_threshold >= 0)
      vl_sift_set_edge_thresh(filt, _params._edge_threshold);
    if (_params._peak_threshold >= 0)
      vl_sift_set_peak_thresh(filt, 255*_params._peak_threshold/_params._num_scales);

    Descriptor<float, 128> descr; // <== an openMVG descriptor

    // Process SIFT computation
    vl_sift_process_first_octave(filt, If.data());

    Allocate(regions); // <== regions allocation

    // Build alias to cached data
    SIFT_Float_Regions * regionsCasted = dynamic_cast<SIFT_Float_Regions*>(regions.get()); // <== This is the container where the final result of the extraction is stored.
                // it consists in the aggregation of a vector of the extracted keypoints (a keypoint type named SIOPointFeature)
                // and a vector of their associated descriptors (a descriptor type named Descriptor)

    // reserve some memory for faster keypoint saving
    regionsCasted->Features().reserve(2000); // <== 
    regionsCasted->Descriptors().reserve(2000); // <==
    
    while (true) { // A loop over all the octaves until there isn't a next octave to process
      vl_sift_detect(filt); // Run the keypoint extraction (in popSIFT, the loop is inside the SIFT extraction function)

      VlSiftKeypoint const *keys  = vl_sift_get_keypoints(filt); // Get the address of the extracted keypoints
      const int nkeys = vl_sift_get_nkeypoints(filt); // and their number

      #ifdef OPENMVG_USE_OPENMP
      #pragma omp parallel for private(descr)
      #endif
      for (int i = 0; i < nkeys; ++i) {

        // (*) All keypoints/descriptors for all orientations of a same keypoint
        // are compute inside popSIFT and a single keypoint with N orientations 
        // lead to N keypoints (duplicated), each associated to an orientation.
        // In lcalvet opinion, this loop shouldn't appear here but in the feature 
        // extraction/description function.
        double angles [4] = {0.0, 0.0, 0.0, 0.0};
        int nangles = 1; // by default (1 upright feature)
        if (_bOrientation)
        { // compute from 1 to 4 orientations
          nangles = vl_sift_calc_keypoint_orientations(filt, angles, keys+i);
        }

        // Loop over all the retrieved orientations. In popSIFT, if lcalvet remember correctly, there is a maximum of 2 possible orientations.
        for (int q=0 ; q < nangles ; ++q) { // (*) this loop shouldn't appear here (inside the sift detection algorithm)
          
          vl_sift_calc_keypoint_descriptor(filt, &descr[0], keys+i, angles[q]); // <== Compute the descriptors from the extracted keypoints
          
          // Keypoint construction
          const SIOPointFeature fp(keys[i].x, keys[i].y,   // <== in openMVG, an extracted keypoint is named SIOPointFeature
            keys[i].sigma, static_cast<float>(angles[q])); // <== Construct the keypoint, the scale attribute passed is the associated sigma (3rd parameter), the last is the angle

          
          if(_params._root_sift) // Here the only difference is the normalization of the descriptor via the L1 norm. Should be kept as it is.
          {
            // rootsift = sqrt( sift / sum(sift) );
            const float sum = std::accumulate(&descr[0], &descr[0] + 128, 0.0f);
            for(int k = 0; k < 128; ++k)
              descr[k] = std::floor(512.f*sqrt(descr[k] / sum));
          }
          else
          {
            for(int k = 0; k < 128; ++k)
              descr[k] = std::floor(512.f*descr[k]);
          }

          #ifdef OPENMVG_USE_OPENMP
          #pragma omp critical
          #endif
          {
            regionsCasted->Features().push_back(fp); // <== Push the keypoint (called Point)
            regionsCasted->Descriptors().push_back(descr); // <== Push its associated descriptor.
          }
        }
      }
      // Go to the next octave
      if (vl_sift_process_next_octave(filt))
        break; // Last octave
    }
    vl_sift_delete(filt);

    vl_destructor();

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

