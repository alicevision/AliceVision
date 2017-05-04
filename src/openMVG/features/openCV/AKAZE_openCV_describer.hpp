#pragma once
#include "openMVG/features/ImageDescriberCommon.hpp"
#include "openMVG/features/image_describer.hpp"
#include "openMVG/features/regions_factory.hpp"

namespace openMVG {

namespace image {

template <typename T>
class Image;

} //namespace image

namespace features {

/**
 * @brief Create an Image_describer interface that use an OpenCV feature extraction method
 * Regions is the same as AKAZE floating point
 */
class AKAZE_openCV_ImageDescriber : public Image_describer
{
public:

  /**
   * @brief Get the corresponding EImageDescriberType
   * @return EImageDescriberType
   */
  virtual EImageDescriberType getDescriberType()
  {
    return EImageDescriberType::AKAZE_OCV;
  }

  /**
   * @brief Use a preset to control the number of detected regions
   * @param preset The preset configuration
   * @return True if configuration succeed. (here always false)
   */
  bool Set_configuration_preset(EDESCRIBER_PRESET preset)
  {
    return false;
  }

  /**
   * @brief Detect regions on the image and compute their attributes (description)
   * @param image Image.
   * @param regions The detected regions and attributes (the caller must delete the allocated data)
   * @param mask 8-bit gray image for keypoint filtering (optional).
   *    Non-zero values depict the region of interest.
   * @return True if detection succed.
   */
  bool Describe(const image::Image<unsigned char>& image,
                std::unique_ptr<Regions> &regions,
                const image::Image<unsigned char> * mask = NULL);

  /**
   * @brief Allocate Regions type depending of the Image_describer
   * @param regions
   */
  void Allocate(std::unique_ptr<Regions> &regions) const
  {
    regions.reset( new AKAZE_Float_Regions );
  }

  template<class Archive>
  void serialize(Archive & ar)
  {}

};

} //namespace features
} //namespace openMVG

#include <cereal/cereal.hpp>
#include <cereal/types/polymorphic.hpp>
#include <cereal/archives/json.hpp>
CEREAL_REGISTER_TYPE_WITH_NAME(openMVG::features::AKAZE_openCV_ImageDescriber, "AKAZE_OCV_Image_describer");
