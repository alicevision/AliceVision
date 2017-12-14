// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/config.hpp>
#include <aliceVision/feature/sift/ImageDescriber_SIFT_vlfeat.hpp>

#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_POPSIFT)
#include <aliceVision/feature/sift/ImageDescriber_SIFT_popSIFT.hpp>
#endif

namespace aliceVision {
namespace feature {

class ImageDescriber_SIFT : public ImageDescriber
{
public:
  ImageDescriber_SIFT(const SiftParams& params = SiftParams(), bool isOriented = true)
    : _params(params)
  {
    // TODO: detect if CUDA is available on the computer
    setUseCuda(false);
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
   * @brief Use a preset to control the number of detected regions
   * @param[in] preset The preset configuration
   * @return True if configuration succeed. (here always false)
   */
  bool Set_configuration_preset(EImageDescriberPreset preset) override
  {
    bool isSuccess =_imageDescriberImpl->Set_configuration_preset(preset);
    return (isSuccess && _params.setPreset(preset));
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
    _useCuda = useCuda;

#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_POPSIFT)
    if(useCuda)
    {
      _imageDescriberImpl.reset(new ImageDescriber_SIFT_popSIFT(_params, _isOriented));
      return;
    }
#endif

    _imageDescriberImpl.reset(new ImageDescriber_SIFT_vlfeat(_params, _isOriented));
  }

  void setCudaPipe(int pipe) override
  {
    _imageDescriberImpl->setCudaPipe(pipe);
  }

  /**
   * @brief Detect regions on the 8-bit image and compute their attributes (description)
   * @param[in] image Image.
   * @param[out] regions The detected regions and attributes (the caller must delete the allocated data)
   * @param[in] mask 8-bit gray image for keypoint filtering (optional).
   *    Non-zero values depict the region of interest.
   * @return True if detection succed.
   */
  bool Describe(const image::Image<unsigned char>& image,
                std::unique_ptr<Regions>& regions,
                const image::Image<unsigned char>* mask = NULL) override
  {
    return _imageDescriberImpl->Describe(image, regions, mask);
  }

  /**
   * @brief Detect regions on the float image and compute their attributes (description)
   * @param[in] image Image.
   * @param[out] regions The detected regions and attributes (the caller must delete the allocated data)
   * @param[in] mask 8-bit gray image for keypoint filtering (optional).
   *    Non-zero values depict the region of interest.
   * @return True if detection succed.
   */
  bool Describe(const image::Image<float>& image,
                std::unique_ptr<Regions>& regions,
                const image::Image<unsigned char>* mask = NULL) override
  {
    return _imageDescriberImpl->Describe(image, regions, mask);
  }

  /**
   * @brief Allocate Regions type depending of the ImageDescriber
   * @param[in,out] regions
   */
  void Allocate(std::unique_ptr<Regions>& regions) const override
  {
    _imageDescriberImpl->Allocate(regions);
  }

private:
  SiftParams _params;
  std::unique_ptr<ImageDescriber> _imageDescriberImpl = nullptr;
  bool _isOriented = true;
  bool _useCuda = false;
};

} // namespace feature
} // namespace aliceVision
