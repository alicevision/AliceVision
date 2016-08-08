#pragma once

#include <openMVG/features/descriptor.hpp>
#include <openMVG/features/image_describer.hpp>
#include <openMVG/features/regions_factory.hpp>

#include <openMVG/features/sift/sift.hpp>

#include <cereal/cereal.hpp>

#include <iostream>
#include <numeric>


namespace openMVG {
namespace features {

// Bibliography:
// [Morel 2009] J.M. Morel and G.Yu, ASIFT: A New Framework for Fully Affine Invariant Image Comparison, 
//        SIAM Journal on Imaging Sciences, vol. 2, issue 2, 2009.

class ASIFT_Image_describer : public Image_describer
{
public:
  ASIFT_Image_describer(const ASiftParams & params = ASiftParams(), bool bOrientation = true);

  ~ASIFT_Image_describer();

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
    const image::Image<unsigned char> * mask = NULL);

  template<class Archive>
  void serialize( Archive & ar )
  {
    ar(
     cereal::make_nvp("params", _params),
     cereal::make_nvp("bOrientation", _bOrientation));
  }

  void Allocate(std::unique_ptr<Regions>& regions) const
  {
    regions.reset(new SIFT_Regions);
  }

private:
  ASiftParams _params;
  bool _bOrientation;
};

} // namespace features
} // namespace openMVG

#include <cereal/types/polymorphic.hpp>
#include <cereal/archives/json.hpp>
CEREAL_REGISTER_TYPE_WITH_NAME(openMVG::features::ASIFT_Image_describer, "ASIFT_Image_describer");
