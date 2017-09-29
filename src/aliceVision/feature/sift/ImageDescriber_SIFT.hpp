// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

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
   * @brief Detect regions on the image and compute their attributes (description)
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
