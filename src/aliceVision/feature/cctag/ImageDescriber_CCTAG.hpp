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

namespace cctag {
  struct Parameters; // Hidden implementation
}

namespace aliceVision {
namespace feature {

/**
 * @brief Create an ImageDescriber interface for CCTag feature extractor
 */
class ImageDescriber_CCTAG : public ImageDescriber
{
public:
  explicit ImageDescriber_CCTAG(const std::size_t nRings = 3);

  ~ImageDescriber_CCTAG() override = default;

  /**
   * @brief Check if the image describer use CUDA
   * @return True if the image describer use CUDA
   */
  bool useCuda() const override;

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
    return 3 * width * height * sizeof(unsigned char);
  }

  /**
   * @brief Set if yes or no imageDescriber need to use cuda implementation
   * @param[in] useCuda
   */
  void setUseCuda(bool) override;

  /**
   * @brief set the CUDA pipe
   * @param[in] pipe The CUDA pipe id
   */
  void setCudaPipe(int pipe) override
  {
    _cudaPipe = pipe;
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

  struct CCTagParameters
  {
    explicit CCTagParameters(size_t nRings);

    ~CCTagParameters() = default;

    bool setPreset(EImageDescriberPreset preset);

    float _cannyThrLow;
    float _cannyThrHigh;
    std::unique_ptr<cctag::Parameters> _internalParams;
  };
private:
  //CCTag parameters
  CCTagParameters _params;
  bool _doAppend = false;
  int _cudaPipe = 0;
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
  for (std::size_t i = 0; i < desc.size(); ++i)
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

} // namespace feature
} // namespace aliceVision
