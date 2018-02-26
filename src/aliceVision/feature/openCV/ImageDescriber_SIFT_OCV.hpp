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

namespace aliceVision {

namespace image {

template <typename T>
class Image;

} //namespace image

namespace feature {

class SIFT_openCV_Params
{
public:

  /**
   * @brief Use a preset to control the number of detected regions
   * @param[in] preset The preset configuration
   * @return True if configuration succeed.
   */
  void setConfigurationPreset(EImageDescriberPreset preset);

  /// Parameters
  std::size_t gridSize = 4;
  std::size_t maxTotalKeypoints = 1000;
  int nOctaveLayers = 6;            //< default opencv value is 3
  double contrastThreshold = 0.04;  //< default opencv value is 0.04
  double edgeThreshold = 10;
  double sigma = 1.6;
  //bool rootSift = true;
};

/**
 * @brief Create an ImageDescriber interface for OpenCV SIFT feature extractor
 * Regions is the same as classic SIFT : 128 unsigned char
 */
class ImageDescriber_SIFT_openCV : public ImageDescriber
{
public:

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
  EImageDescriberType getDescriberType() const override
  {
    return EImageDescriberType::SIFT_OCV;
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
    return 3 * width * height * sizeof(unsigned char) + (_params.maxTotalKeypoints * 128 * sizeof(unsigned char));
  }

  /**
   * @brief Use a preset to control the number of detected regions
   * @param[in] preset The preset configuration
   * @return True if configuration succeed.
   */
  void setConfigurationPreset(EImageDescriberPreset preset) override
  {
    _params.setConfigurationPreset(preset);
  }

  /**
   * @brief Detect regions on the 8-bit image and compute their attributes (description)
   * @param[in] image Image.
   * @param[out] regions The detected regions and attributes (the caller must delete the allocated data)
   * @param[in] mask 8-bit grayscale image for keypoint filtering (optional)
   *    Non-zero values depict the region of interest.
   * @return True if detection succed.
   */
  bool describe(const image::Image<unsigned char>& image,
                std::unique_ptr<Regions>& regions,
                const image::Image<unsigned char>* mask = NULL) override;

  /**
   * @brief Allocate Regions type depending of the ImageDescriber
   * @param[in,out] regions
   */
  void allocate(std::unique_ptr<Regions> &regions) const override
  {
    regions.reset( new SIFT_Regions );
  }

private:
  SIFT_openCV_Params _params;
};

} //namespace feature
} //namespace openMV
