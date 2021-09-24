// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/feature/imageDescriberCommon.hpp>
#include <aliceVision/feature/ImageDescriber.hpp>
#include <aliceVision/feature/regionsFactory.hpp>
#include <aliceVision/types.hpp>

#include <iostream>
#include <numeric>

namespace aliceVision {
namespace feature {

/**
 * @brief Create an ImageDescriber interface for AprilTag feature extractor
 */
class ImageDescriber_APRILTAG : public ImageDescriber
{
public:
  explicit ImageDescriber_APRILTAG();

  ~ImageDescriber_APRILTAG() override = default;

  /**
   * @brief Check if the image describer use CUDA
   * @return True if the image describer use CUDA
   */
  bool useCuda() const override
  {
    return false;
  }

  /**
   * @brief Check if the image describer use float image
   * @return True if the image describer use float image
   */
  bool useFloatImage() const override
  {
    return false;
  }

  /**
   * @brief Get the corresponding EImageDescriberType
   * @return EImageDescriberType
   */
  EImageDescriberType getDescriberType() const override;

  /**
   * @brief Get the total amount of RAM needed for a
   * feature extraction of an image of the given dimension.
   * @param[in] width The image width
   * @param[in] height The image height
   * @return total amount of memory needed
   */
  std::size_t getMemoryConsumption(std::size_t width, std::size_t height) const override
  {
    // this should be a reasonably safe upper bound, but to know this exactly,
    // much deeper exmination of the AprilTag source code would be needed:
    return 3 * width * height * sizeof(unsigned char);
  }

  /**
   * @brief Use a preset to control the number of detected regions
   * @param[in] preset The preset configuration
   * @return True if configuration succeed. (here always false)
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
  bool describe(const image::Image<unsigned char>& image,
    std::unique_ptr<Regions> &regions,
    const image::Image<unsigned char> * mask = nullptr) override;

  /**
   * @brief Allocate Regions type depending of the ImageDescriber
   * @param[in,out] regions
   */
  void allocate(std::unique_ptr<Regions> &regions) const override;

  struct AprilTagParameters
  {
    explicit AprilTagParameters() = default;

    ~AprilTagParameters() = default;

    void setPreset(EImageDescriberPreset preset);
  };
private:
  //AprilTag parameters
  AprilTagParameters _params;
};

/**
 * @brief Convert the descriptor representation into a AprilTag ID.
 * @param[in] desc descriptor
 * @return apriltag id or UndefinedIndexT if wrong apriltag descriptor
 */
template <class DescriptorT>
IndexT getAprilTagId(const DescriptorT & desc)
{
  std::size_t apriltagId = UndefinedIndexT;
  for (std::size_t i = 0; i < desc.size(); ++i)
  {
    if (desc.getData()[i] == (unsigned char) 255)
    {
      if (apriltagId != UndefinedIndexT)
      {
        return UndefinedIndexT;
      }
      apriltagId = i;
    }
    else if(desc.getData()[i] != (unsigned char) 0)
    {
      return UndefinedIndexT;
    }
  }
  return apriltagId;
}

} // namespace feature
} // namespace aliceVision
