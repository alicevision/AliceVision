// This file is part of the AliceVision project.
// Copyright (c) 2020 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/feature/Descriptor.hpp>
#include <aliceVision/feature/ImageDescriber.hpp>
#include <aliceVision/feature/regionsFactory.hpp>
#include <aliceVision/feature/sift/SIFT.hpp>

extern "C" {
#include <nonFree/sift/vl/sift.h>
}

#include <iostream>
#include <numeric>

namespace aliceVision {
namespace feature {

struct DspSiftParams : public SiftParams
{
    bool domainSizePooling = true;
    bool estimateAffineShape = false;
    double dspMinScale = 1.0 / 6.0;
    double dspMaxScale = 3.0;
    int dspNumScales = 10;

    void setPreset(ConfigurationPreset preset) override;
};


template <typename T>
bool extractDSPSIFT(const image::Image<float>& image, std::unique_ptr<Regions>& regions, const DspSiftParams& params,
                    bool orientation, const image::Image<unsigned char>* mask);

/**
 * @brief Create an ImageDescriber interface for VLFeat SIFT feature extractor
 *
 * "Domain-Size Pooling in Local Descriptors: DSP-SIFT", CVPR 2015
 * Jingming Dong, Stefano Soatto, University of California
 * https://www.cv-foundation.org/openaccess/content_cvpr_2015/papers/Dong_Domain-Size_Pooling_in_2015_CVPR_paper.pdf
 *
 * "A Theory of Local Matching: SIFT and Beyond", 2016
 * Hossein Mobahi CSAIL MIT, Stefano Soatto, CS Dept. UCLA
 * https://arxiv.org/pdf/1601.05116.pdf
 *
 * DSP-SIFT has been shown to outperform other SIFT variants and learned descriptors in
 * "Comparative Evaluation of Hand-Crafted and Learned Local Features", CVPR 2016
 * Schönberger, Hardmeier, Sattler, Pollefeys
 * https://openaccess.thecvf.com/content_cvpr_2017/papers/Schonberger_Comparative_Evaluation_of_CVPR_2017_paper.pdf
 */
class ImageDescriber_DSPSIFT_vlfeat : public ImageDescriber
{
public:
  explicit ImageDescriber_DSPSIFT_vlfeat(const DspSiftParams& params = DspSiftParams(), bool isOriented = true)
    : ImageDescriber()
    , _params(params)
    , _isOriented(isOriented)
  {
    // Configure VLFeat
    VLFeatInstance::initialize();
  }

  ~ImageDescriber_DSPSIFT_vlfeat() override
  {
    VLFeatInstance::destroy();
  }

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
    return true;
  }

  /**
   * @brief Get the corresponding EImageDescriberType
   * @return EImageDescriberType
   */
  EImageDescriberType getDescriberType() const override
  {
    return EImageDescriberType::DSPSIFT;
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
    return getMemoryConsumptionVLFeat(width, height, _params);
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
  void setConfigurationPreset(ConfigurationPreset preset) override
  {
    _params.setPreset(preset);
  }

  /**
   * @brief Detect regions on the float image and compute their attributes (description)
   * @param[in] image Image.
   * @param[out] regions The detected regions and attributes (the caller must delete the allocated data)
   * @param[in] mask 8-bit grayscale image for keypoint filtering (optional)
   *    Non-zero values depict the region of interest.
   * @return True if detection succed.
   */
  bool describe(const image::Image<float>& image,
    std::unique_ptr<Regions>& regions,
    const image::Image<unsigned char>* mask = nullptr) override
  {
    return extractDSPSIFT<unsigned char>(image, regions, _params, _isOriented, mask);
  }


  /**
   * @brief Allocate Regions type depending of the ImageDescriber
   * @param[in,out] regions
   */
  void allocate(std::unique_ptr<Regions>& regions) const override
  {
    regions.reset(new SIFT_Regions);
  }

private:
  DspSiftParams _params;
  bool _isOriented;
};

} // namespace feature
} // namespace aliceVision
