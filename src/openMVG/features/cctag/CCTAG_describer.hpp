#pragma once

#include <openMVG/features/image_describer.hpp>
#include <openMVG/features/regions_factory.hpp>
#include <openMVG/types.hpp>

#include <cereal/cereal.hpp>
#include <iostream>
#include <numeric>

namespace openMVG {
namespace features {

/** @brief CCTag filter pixel type */

class CCTAG_Image_describer : public Image_describer
{
public:
  CCTAG_Image_describer();
  CCTAG_Image_describer(const std::size_t nRings, const bool doAppend = false);
  ~CCTAG_Image_describer();

  bool Set_configuration_preset(EDESCRIBER_PRESET preset);

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

  /// Allocate Regions type depending of the Image_describer
  void Allocate(std::unique_ptr<Regions> &regions) const;

  template<class Archive>
  void serialize( Archive & ar )
  {
    //ar(
     //cereal::make_nvp("params", _params),
     //cereal::make_nvp("bOrientation", _bOrientation));
  }

private:
  //CCTag parameters
  struct CCTagParameters; // Hidden implementation
  CCTagParameters *_params;
  bool _doAppend;
};

/**
 * @brief Convert the descriptor representation into a CCTag ID.
 * @param[in] desc descriptor
 * @return cctag id or UndefinedIndexT if wrong cctag descriptor
 */
template <class DescriptorT>
IndexT getCCTagId(const DescriptorT & desc)
{
  std::size_t cctagId = UndefinedIndexT;
  for (int i = 0; i < desc.size(); ++i)
  {
    if (desc.getData()[i] == (unsigned char) 255)
    {
      if (cctagId != UndefinedIndexT)
      {
        return UndefinedIndexT;
      }
      cctagId = i;
    }
    else if(desc.getData()[i] != (unsigned char) 0)
    {
      return UndefinedIndexT;
    }
  }
  return cctagId;
}

} // namespace features
} // namespace openMVG

#include <cereal/types/polymorphic.hpp>
#include <cereal/archives/json.hpp>
CEREAL_REGISTER_TYPE_WITH_NAME(openMVG::features::CCTAG_Image_describer, "CCTAG_Image_describer");
