// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/feature/Descriptor.hpp>
#include <aliceVision/feature/ImageDescriber.hpp>
#include <aliceVision/feature/regionsFactory.hpp>
#include <aliceVision/feature/sift/SIFT.hpp>

extern "C" {
#include "nonFree/sift/vl/sift.h"
}

#include <iostream>
#include <numeric>

namespace aliceVision {
namespace feature {

/**
 * @brief Create an ImageDescriber interface for VLFeat SIFT Float feature extractor
 */
class ImageDescriber_SIFT_vlfeatFloat : public ImageDescriber
{
public:
  ImageDescriber_SIFT_vlfeatFloat(const SiftParams& params = SiftParams(), bool isOriented = true)
    : ImageDescriber()
    , _params(params)
    , _isOriented(isOriented)
  {
    // Configure VLFeat
    vl_constructor();
  }

  ~ImageDescriber_SIFT_vlfeatFloat()
  {
    vl_destructor();
  }

  /**
   * @brief Check if the image describer use float image
   * @return True if the image describer use float image
   */
  bool useFloatImage() const override
  {
    return true;
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
  bool Set_configuration_preset(EImageDescriberPreset preset) override
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
   * @brief Detect regions on the float image and compute their attributes (description)
   * @param[in] image Image.
   * @param[out] regions The detected regions and attributes (the caller must delete the allocated data)
   * @param[in] mask 8-bit gray image for keypoint filtering (optional).
   *    Non-zero values depict the region of interest.
   * @return True if detection succed.
   */
  bool Describe(const image::Image<float>& image,
    std::unique_ptr<Regions>& regions,
    const image::Image<unsigned char>* mask = NULL) override
  {
    return extractSIFT<float>(image, regions, _params, _isOriented, mask);
  }

  /**
   * @brief Allocate Regions type depending of the ImageDescriber
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

} // namespace feature
} // namespace aliceVision
