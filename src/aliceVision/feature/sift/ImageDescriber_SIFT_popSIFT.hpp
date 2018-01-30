// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/feature/Descriptor.hpp>
#include <aliceVision/feature/ImageDescriber.hpp>
#include <aliceVision/feature/regionsFactory.hpp>
#include <aliceVision/feature/sift/SIFT.hpp>

// PopSIFT includes
#include <popsift/popsift.h>
#include <popsift/sift_pyramid.h>
#include <popsift/sift_octave.h>
#include <popsift/common/device_prop.h>

#include <iostream>
#include <numeric>

namespace aliceVision {
namespace feature {

/**
 * @brief Create an ImageDescriber interface for PopSIFT SIFT feature extractor
 */
class ImageDescriber_SIFT_popSIFT : public ImageDescriber
{
public:
  ImageDescriber_SIFT_popSIFT(const SiftParams& params = SiftParams(), bool isOriented = true)
    : ImageDescriber()
    , _params(params)
    , _isOriented(isOriented)
  {
    // Process SIFT computation
    cudaDeviceReset();

    popsift::cuda::device_prop_t deviceInfo;
    deviceInfo.set(0, true); //Use only the first device
    //deviceInfo.print();
    resetConfiguration();
  }

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
    return EImageDescriberType::SIFT;
  }

  /**
   * @brief Get the total amount of RAM needed for a
   * feature extraction of an image of the given dimension.
   * @param[in] width The image width
   * @param[in] height The image height
   * @return total amount of memory needed
   */
  virtual std::size_t getMemoryConsumption(std::size_t width, std::size_t height) const override
  {
    return 3 * width * height * sizeof(float); //  GPU only
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
  void setConfigurationPreset(EImageDescriberPreset preset) override
  {
    _params.setPreset(preset);
    resetConfiguration();
  }

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
                const image::Image<unsigned char>* mask = NULL) override;

  /**
   * @brief Allocate Regions type depending of the ImageDescriber
   * @param[in,out] regions
   */
  void allocate(std::unique_ptr<Regions>& regions) const override
  {
    regions.reset(new SIFT_Regions);
  }

private:

  void resetConfiguration()
  {
    popsift::Config config;
    config.setOctaves(_params._numOctaves);
    config.setLevels(_params._numScales);
    config.setDownsampling(_params._firstOctave);
    config.setThreshold(_params._peakThreshold);
    config.setEdgeLimit(_params._edgeThreshold);
    config.setNormalizationMultiplier(9); // 2^9 = 512
    config.setNormMode( _params._rootSift ? popsift::Config::RootSift : popsift::Config::Classic);
    config.setFilterMaxExtrema(_params._maxTotalKeypoints);
    config.setFilterSorting(popsift::Config::LargestScaleFirst);

    _popSift.reset(new PopSift(config, popsift::Config::ExtractingMode, PopSift::FloatImages));
  }

  SiftParams _params;
  bool _isOriented = true;
  static std::unique_ptr<PopSift> _popSift;
};

} // namespace feature
} // namespace aliceVision
