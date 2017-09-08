// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#pragma once

#include <aliceVision/features/descriptor.hpp>
#include <aliceVision/features/image_describer.hpp>
#include <aliceVision/features/regions_factory.hpp>
#include <aliceVision/features/sift/SIFT.hpp>

// PopSIFT includes
#include <popsift/popsift.h>
#include <popsift/sift_pyramid.h>
#include <popsift/sift_octave.h>
#include <popsift/common/device_prop.h>

#include <iostream>
#include <numeric>

namespace aliceVision {
namespace features {

/**
 * @brief Create an Image_describer interface for PopSIFT SIFT feature extractor
 */
class SIFT_popSIFT_ImageDescriber : public Image_describer
{
public:
  SIFT_popSIFT_ImageDescriber(const SiftParams& params = SiftParams(), bool isOriented = true)
    : Image_describer()
    , _params(params)
    , _isOriented(isOriented)
  {
    // Process SIFT computation
    cudaDeviceReset();

    popsift::Config config;
    config.setOctaves(_params._num_octaves);
    config.setLevels(_params._num_scales);
    config.setDownsampling(_params._first_octave);
    config.setThreshold(  _params._peak_threshold);
    config.setEdgeLimit(  _params._edge_threshold);
    config.setUseRootSift(_params._root_sift);
    config.setNormalizationMultiplier(9); // 2^9 = 512

    popsift::cuda::device_prop_t deviceInfo;

    deviceInfo.set(0, true); //Use only the first device
    deviceInfo.print();

    _popSift.reset( new PopSift(config) );
  }

  /**
   * @brief Get the corresponding EImageDescriberType
   * @return EImageDescriberType
   */
  EImageDescriberType getDescriberType() const override
  {
    return EImageDescriberType::SIFT;
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
                const image::Image<unsigned char>* mask = NULL) override;

  /**
   * @brief Allocate Regions type depending of the Image_describer
   * @param[in,out] regions
   */
  void Allocate(std::unique_ptr<Regions>& regions) const override
  {
    regions.reset(new SIFT_Regions);
  }

private:
  SiftParams _params;
  bool _isOriented = true;

  static std::unique_ptr<PopSift> _popSift;
};

} // namespace features
} // namespace aliceVision
