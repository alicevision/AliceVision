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

class SIFT_openCV_Params
{
public:

  /**
   * @brief Use a preset to control the number of detected regions
   * @param[in] preset The preset configuration
   * @return True if configuration succeed.
   */
  bool Set_configuration_preset(EDESCRIBER_PRESET preset);

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
   * @brief Get the corresponding EImageDescriberType
   * @return EImageDescriberType
   */
  EImageDescriberType getDescriberType() const override
  {
    return EImageDescriberType::SIFT_OCV;
  }

  /**
   * @brief Use a preset to control the number of detected regions
   * @param[in] preset The preset configuration
   * @return True if configuration succeed.
   */
  bool Set_configuration_preset(EDESCRIBER_PRESET preset) override
  {
    return _params.Set_configuration_preset(preset);
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
    regions.reset( new SIFT_Regions );
  }

private:
  SIFT_openCV_Params _params;
};

} //namespace feature
} //namespace openMV
