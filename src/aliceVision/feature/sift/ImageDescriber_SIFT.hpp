// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/config.hpp>
#include <aliceVision/feature/sift/ImageDescriber_SIFT_vlfeat.hpp>

#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_POPSIFT)
#include <aliceVision/gpu/gpu.hpp>
#include <aliceVision/feature/sift/ImageDescriber_SIFT_popSIFT.hpp>
#endif

namespace aliceVision {
namespace feature {

/**
 * @brief SIFT Image Describer class
 * use :
 *  - PopSIFT Image describer (if defined and only with compatible device)
 *  - VLFeat SIFT Image describer
 */
class ImageDescriber_SIFT : public ImageDescriber
{
public:
  explicit ImageDescriber_SIFT(const SiftParams& params = SiftParams(), bool isOriented = true)
    : _params(params)
    , _isOriented(isOriented)
  {
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_POPSIFT)
    setUseCuda(gpu::gpuSupportCUDA(3,0));
#else
    setUseCuda(false);
#endif
  }

  /**
   * @brief Check if the image describer use CUDA
   * @return True if the image describer use CUDA
   */
  bool useCuda() const override
  {
    return _imageDescriberImpl->useCuda();
  }

  /**
   * @brief Check if the image describer use float image
   * @return True if the image describer use float image
   */
  bool useFloatImage() const override
  {
    return _imageDescriberImpl->useFloatImage();
  }

  /**
   * @brief Get the corresponding EImageDescriberType
   * @return EImageDescriberType
   */
  EImageDescriberType getDescriberType() const override
  {
    return _imageDescriberImpl->getDescriberType();
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
    return _imageDescriberImpl->getMemoryConsumption(width, height);
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
   * @brief Set if yes or no imageDescriber need to use cuda implementation
   * @param[in] useCuda
   */
  void setUseCuda(bool useCuda) override
  {
    if(_imageDescriberImpl != nullptr && this->useCuda() == useCuda)
      return;

#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_POPSIFT)
    if(useCuda)
    {
      _imageDescriberImpl.release(); // release first to ensure that we don't create the new ImageDescriber before destroying the previous one
      _imageDescriberImpl.reset(new ImageDescriber_SIFT_popSIFT(_params, _isOriented));
      return;
    }
#endif

    _imageDescriberImpl.release(); // release first to ensure that we don't create the new ImageDescriber before destroying the previous one
    _imageDescriberImpl.reset(new ImageDescriber_SIFT_vlfeat(_params, _isOriented));
  }

  /**
   * @brief set the CUDA pipe
   * @param[in] pipe The CUDA pipe id
   */
  void setCudaPipe(int pipe) override
  {
    _imageDescriberImpl->setCudaPipe(pipe);
  }

  /**
   * @brief Use a preset to control the number of detected regions
   * @param[in] preset The preset configuration
   */
  void setConfigurationPreset(ConfigurationPreset preset) override
  {
     _params.setPreset(preset);
     _imageDescriberImpl->setConfigurationPreset(preset);
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
                const image::Image<unsigned char>* mask = nullptr) override
  {
    return _imageDescriberImpl->describe(image, regions, mask);
  }

  /**
   * @brief Detect regions on the float image and compute their attributes (description)
   * @param[in] image Image.
   * @param[out] regions The detected regions and attributes (the caller must delete the allocated data)
   * @param[in] mask 8-bit gray image for keypoint filtering (optional).
   *    Non-zero values depict the region of interest.
   * @return True if detection succed.
   */
  bool describe(const image::Image<float>& image,
                std::unique_ptr<Regions>& regions,
                const image::Image<unsigned char>* mask = nullptr) override
  {
    return _imageDescriberImpl->describe(image, regions, mask);
  }

  /**
   * @brief Allocate Regions type depending of the ImageDescriber
   * @param[in,out] regions
   */
  void allocate(std::unique_ptr<Regions>& regions) const override
  {
    _imageDescriberImpl->allocate(regions);
  }

  ~ImageDescriber_SIFT() override = default;

private:
  SiftParams _params;
  std::unique_ptr<ImageDescriber> _imageDescriberImpl = nullptr;
  bool _isOriented = true;
};

} // namespace feature
} // namespace aliceVision
