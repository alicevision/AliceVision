// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/feature/Descriptor.hpp>
#include <aliceVision/feature/ImageDescriber.hpp>
#include <aliceVision/feature/regionsFactory.hpp>
#include <aliceVision/feature/sift/SIFT.hpp>

#include <iostream>
#include <numeric>

class PopSift;

namespace aliceVision {
namespace feature {

/**
 * @brief Create an ImageDescriber interface for PopSIFT SIFT feature extractor
 */
class ImageDescriber_SIFT_popSIFT : public ImageDescriber
{
public:
  explicit ImageDescriber_SIFT_popSIFT(const SiftParams& params = SiftParams(), bool isOriented = true);

  /**
   * @brief Check if the image describer use CUDA
   * @return True if the image describer use CUDA
   */
  bool useCuda() const override
  {
    return true;
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
  EImageDescriberType getDescriberType() const override
  {
    if(!_isOriented)
      return EImageDescriberType::SIFT_UPRIGHT;
    return EImageDescriberType::SIFT;
  }

  /**
   * @brief Get the total amount of RAM needed for a
   * feature extraction of an image of the given dimension.
   * @param[in] width The image width
   * @param[in] height The image height
   * @return total amount of memory needed
   */
  std::size_t getMemoryConsumption(std::size_t width, std::size_t height) const override
  {
    //  GPU only
    return 4 * width * height * sizeof(float); // only use the input RGBA image
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
   * @brief Use a preset to control the number of detected regions
   * @param[in] preset The preset configuration
   */
  void setConfigurationPreset(ConfigurationPreset preset) override;

  /**
   * @brief Detect regions on the 8-bit image and compute their attributes (description)
   * @param[in] image Image.
   * @param[out] regions The detected regions and attributes (the caller must delete the allocated data)
   * @param[in] mask 8-bit grayscale image for keypoint filtering (optional)
   *    Non-zero values depict the region of interest.
   * @return True if detection succed.
   */
  bool describe(const image::Image<float>& image,
                std::unique_ptr<Regions>& regions,
                const image::Image<unsigned char>* mask = nullptr) override;

  /**
   * @brief Allocate Regions type depending of the ImageDescriber
   * @param[in,out] regions
   */
  void allocate(std::unique_ptr<Regions>& regions) const override
  {
    regions.reset(new SIFT_Regions);
  }

  /**
   * @brief Destructor
   */
  ~ImageDescriber_SIFT_popSIFT() override;

private:

  void resetConfiguration();

  SiftParams _params;
  bool _isOriented = true;
  static std::unique_ptr<PopSift> _popSift;
  static std::atomic<int> _instanceCounter;
};

} // namespace feature
} // namespace aliceVision
