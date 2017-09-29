// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

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


/** @brief CCTag filter pixel type */
class ImageDescriber_CCTAG : public ImageDescriber
{
public:
  ImageDescriber_CCTAG(const std::size_t nRings = 3);
  ~ImageDescriber_CCTAG();

  EImageDescriberType getDescriberType() const override
  {
    // TODO: check nRings to decide between CCTAG3 and CCTAG4
    return EImageDescriberType::CCTAG3;
  }
  
  bool Set_configuration_preset(EImageDescriberPreset preset) override;

  void setUseCuda(bool) override;

  void setCudaPipe(int pipe) override { _cudaPipe = pipe; }

  /**
  @brief Detect regions on the image and compute their attributes (description)
  @param image Image.
  @param regions The detected regions and attributes (the caller must delete the allocated data)
  @param mask 8-bit gray image for keypoint filtering (optional).
     Non-zero values depict the region of interest.
  */
  bool Describe(const image::Image<unsigned char>& image,
    std::unique_ptr<Regions> &regions,
    const image::Image<unsigned char> * mask = nullptr) override;

  /// Allocate Regions type depending of the ImageDescriber
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
