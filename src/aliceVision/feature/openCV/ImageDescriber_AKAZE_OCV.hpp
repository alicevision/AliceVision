// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#pragma once
#include "aliceVision/feature/imageDescriberCommon.hpp"
#include "aliceVision/feature/ImageDescriber.hpp"
#include "aliceVision/feature/regionsFactory.hpp"

namespace aliceVision {

namespace image {

template <typename T>
class Image;

} //namespace image

namespace feature {

/**
 * @brief Create an ImageDescriber interface for OpenCV AKAZE feature extractor
 * Regions is the same as AKAZE floating point
 */
class ImageDescriber_AKAZE_OCV : public ImageDescriber
{
public:

  /**
   * @brief Get the corresponding EImageDescriberType
   * @return EImageDescriberType
   */
  virtual EImageDescriberType getDescriberType() const override
  {
    return EImageDescriberType::AKAZE_OCV;
  }

  /**
   * @brief Use a preset to control the number of detected regions
   * @param[in] preset The preset configuration
   * @return True if configuration succeed. (here always false)
   */
  bool Set_configuration_preset(EImageDescriberPreset preset) override
  {
    return false;
  }

  /**
   * @brief Detect regions on the image and compute their attributes (description)
   * @param[in] image Image.
   * @param[out] regions The detected regions and attributes (the caller must delete the allocated data)
   * @param[in] mask 8-bit gray image for keypoint filtering (optional).
   *    Non-zero values depict the region of interest.
   * @return True if detection succed.
   */
  bool Describe(const image::Image<unsigned char>& image,
                std::unique_ptr<Regions> &regions,
                const image::Image<unsigned char> * mask = NULL) override;

  /**
   * @brief Allocate Regions type depending of the ImageDescriber
   * @param[in,out] regions
   */
  void Allocate(std::unique_ptr<Regions> &regions) const override
  {
    regions.reset( new AKAZE_Float_Regions );
  }

};

} //namespace feature
} //namespace aliceVision
