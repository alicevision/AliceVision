// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#pragma once

#include <openMVG/features/descriptor.hpp>
#include <openMVG/features/image_describer.hpp>
#include <openMVG/features/regions_factory.hpp>
#include <openMVG/features/sift/SIFT.hpp>

extern "C" {
#include "nonFree/sift/vl/sift.h"
}

#include <iostream>
#include <numeric>

namespace openMVG {
namespace features {

/**
 * @brief Create an Image_describer interface for VLFeat SIFT Float feature extractor
 */
class SIFT_vlfeat_float_ImageDescriber : public Image_describer
{
public:
  SIFT_vlfeat_float_ImageDescriber(const SiftParams& params = SiftParams(), bool isOriented = true)
    : Image_describer()
    , _params(params)
    , _isOriented(isOriented)
  {
    // Configure VLFeat
    vl_constructor();
  }

  ~SIFT_vlfeat_float_ImageDescriber()
  {
    vl_destructor();
  }

  /**
   * @brief Get the corresponding EImageDescriberType
   * @return EImageDescriberType
   */
  virtual EImageDescriberType getDescriberType() const override
  {
    return EImageDescriberType::SIFT_FLOAT;
  }
  
  /**
   * @brief Use a preset to control the number of detected regions
   * @param[in] preset The preset configuration
   * @return True if configuration succeed. (here always false)
   */
  bool Set_configuration_preset(EDESCRIBER_PRESET preset) override
  {
    return _params.setPreset(preset);
  }
  

  /**
   * @brief Set image describer always upRight
   * @param[in] upRight
   */
  void setUpRight(bool upRight) override
  {
    _isOriented = !upRight;
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
    std::unique_ptr<Regions>& regions,
    const image::Image<unsigned char>* mask = NULL) override
  {
    return extractSIFT<float>(image, regions, _params, _isOriented, mask);
  }

  /**
   * @brief Allocate Regions type depending of the Image_describer
   * @param[in,out] regions
   */
  void Allocate(std::unique_ptr<Regions>& regions) const override
  {
    regions.reset(new SIFT_Float_Regions);
  }
  
private:
  SiftParams _params;
  bool _isOriented;
};

} // namespace features
} // namespace openMVG
