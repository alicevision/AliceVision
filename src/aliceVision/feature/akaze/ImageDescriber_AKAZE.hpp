// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/config.hpp>
#include <aliceVision/feature/ImageDescriber.hpp>
#include <aliceVision/feature/imageDescriberCommon.hpp>
#include <aliceVision/feature/regionsFactory.hpp>
#include <aliceVision/feature/akaze/AKAZE.hpp>
#include <aliceVision/feature/akaze/descriptorLIOP.hpp>
#include <aliceVision/feature/akaze/descriptorMLDB.hpp>
#include <aliceVision/feature/akaze/descriptorMSURF.hpp>

#include <cereal/cereal.hpp>

using namespace std;

namespace aliceVision {
namespace feature {

enum EAKAZE_DESCRIPTOR
{
  AKAZE_MSURF,
  AKAZE_LIOP,
  AKAZE_MLDB
};

struct AKAZEParams
{
  AKAZEParams(
    AKAZEConfig config = AKAZEConfig(),
    EAKAZE_DESCRIPTOR eAkazeDescriptor = AKAZE_MSURF
  ):_options(config),_eAkazeDescriptor(eAkazeDescriptor){}

  template<class Archive>
  void serialize(Archive & ar)
  {
    ar(_options, _eAkazeDescriptor);
  }

  // Parameters
  AKAZEConfig _options;
  EAKAZE_DESCRIPTOR _eAkazeDescriptor;
};

class ImageDescriber_AKAZE : public ImageDescriber
{
public:
  ImageDescriber_AKAZE(
    const AKAZEParams & params = AKAZEParams(),
    bool bOrientation = true
  ):ImageDescriber(), _params(params), _bOrientation(bOrientation) {}

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
  virtual EImageDescriberType getDescriberType() const override
  {
    switch(_params._eAkazeDescriptor)
    {
      case AKAZE_MSURF: return EImageDescriberType::AKAZE;
      case AKAZE_LIOP: return EImageDescriberType::AKAZE_LIOP;
      case AKAZE_MLDB: return EImageDescriberType::AKAZE_MLDB;
    }
    throw std::logic_error("Unknown AKAZE type.");
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
    std::size_t fullImgSize = width * height;
    std::size_t memoryConsuption = 0;
    double downscale = 1.0;
    for(int octave = 0; octave < _params._options.iNbOctave; ++octave)
    {
      memoryConsuption += fullImgSize / (downscale * downscale);
      downscale *= 2.0;
    }
    memoryConsuption *= _params._options.iNbSlicePerOctave * sizeof(float);
    return 4 * memoryConsuption + (3 * width * height * sizeof(float)) + 1.5 * std::pow(2,30); // add arbitrary 1.5 GB
  }

  /**
   * @brief Set image describer always upRight
   * @param[in] upRight
   */
  void setUpRight(bool upRight) override
  {
    _bOrientation = !upRight;
  }

  /**
   * @brief Use a preset to control the number of detected regions
   * @param[in] preset The preset configuration
   */
  void setConfigurationPreset(EImageDescriberPreset preset) override
  {
    switch(preset)
    {
      case EImageDescriberPreset::LOW:
      case EImageDescriberPreset::MEDIUM:
      case EImageDescriberPreset::NORMAL:
        _params._options.fThreshold = AKAZEConfig().fThreshold;
      break;
      case EImageDescriberPreset::HIGH:
        _params._options.fThreshold = AKAZEConfig().fThreshold/10.;
      break;
      case EImageDescriberPreset::ULTRA:
       _params._options.fThreshold = AKAZEConfig().fThreshold/100.;
      break;
      default:
        throw std::out_of_range("Invalid image describer preset enum");
    }
  }

  /**
   * @brief Detect regions on the float image and compute their attributes (description)
   * @param[in] image Image.
   * @param[out] regions The detected regions and attributes
   * @param[in] mask 8-bit grayscale image for keypoint filtering (optional)
   * Non-zero values depict the region of interest.
   */
  bool describe(const image::Image<float>& image,
    std::unique_ptr<Regions> &regions,
    const image::Image<unsigned char> * mask = nullptr) override;

  /**
   * @brief Allocate Regions type depending of the ImageDescriber
   * @param[in,out] regions
   */
  void allocate(std::unique_ptr<Regions> &regions) const override
  {
    switch(_params._eAkazeDescriptor)
    {
      case AKAZE_MSURF: regions.reset(new AKAZE_Float_Regions); break;
      case AKAZE_LIOP:  regions.reset(new AKAZE_Liop_Regions);  break;
      case AKAZE_MLDB:  regions.reset(new AKAZE_BinaryRegions); break;
    }
  }

private:
  AKAZEParams _params;
  bool _bOrientation;
};

} // namespace feature
} // namespace aliceVision
