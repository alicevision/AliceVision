// This file is part of the AliceVision project.
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
  class Parameters; // Hidden implementation
}

namespace aliceVision {
namespace feature {

/**
 * @brief Create an ImageDescriber interface for CCTag feature extractor
 */
class ImageDescriber_CCTAG : public ImageDescriber
{
public:
  ImageDescriber_CCTAG(const std::size_t nRings = 3);
  ~ImageDescriber_CCTAG();

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
  EImageDescriberType getDescriberType() const override
  {
    // TODO: check nRings to decide between CCTAG3 and CCTAG4
    return EImageDescriberType::CCTAG3;
  }
  
  /**
   * @brief Use a preset to control the number of detected regions
   * @param[in] preset The preset configuration
   * @return True if configuration succeed. (here always false)
   */
  bool Set_configuration_preset(EImageDescriberPreset preset) override;

  /**
   * @brief Set if yes or no imageDescriber need to use cuda implementation
   * @param[in] useCuda
   */
  void setUseCuda(bool) override;

  void setCudaPipe(int pipe) override { _cudaPipe = pipe; }

  /**
   * @brief Detect regions on the 8-bit image and compute their attributes (description)
   * @param[in] image Image.
   * @param[out] regions The detected regions and attributes (the caller must delete the allocated data)
   * @param[in] mask 8-bit gray image for keypoint filtering (optional).
   *    Non-zero values depict the region of interest.
   * @return True if detection succed.
   */
  bool Describe(const image::Image<unsigned char>& image,
    std::unique_ptr<Regions> &regions,
    const image::Image<unsigned char> * mask = nullptr) override;

  /**
   * @brief Allocate Regions type depending of the ImageDescriber
   * @param[in,out] regions
   */
  void Allocate(std::unique_ptr<Regions> &regions) const override;

  struct CCTagParameters
  {
    CCTagParameters(size_t nRings);
    ~CCTagParameters();

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
