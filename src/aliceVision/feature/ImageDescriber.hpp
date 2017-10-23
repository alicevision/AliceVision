// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#ifndef ALICEVISION_FEATURES_IMAGE_DESCRIBER_HPP
#define ALICEVISION_FEATURES_IMAGE_DESCRIBER_HPP

#include "aliceVision/numeric/numeric.hpp"
#include "aliceVision/feature/imageDescriberCommon.hpp"
#include "aliceVision/feature/Regions.hpp"
#include "aliceVision/image/Image.hpp"
#include <memory>

#include <string>
#include <iostream>

namespace aliceVision {
namespace feature {

/**
 * @brief The preset to control the number of detected regions
 */
enum class EImageDescriberPreset
{
  LOW = 0,
  MEDIUM,
  NORMAL,
  HIGH,
  ULTRA
};

/**
 * @brief It returns the preset from a string.
 * @param[in] sPreset the input string.
 * @return the associated describer preset.
 */
EImageDescriberPreset EImageDescriberPreset_stringToEnum(const std::string& sPreset);

/**
 * @brief It converts a preset to a string.
 * @param[in] preset the describer preset enum to convert.
 * @return the string associated to the describer preset.
 */
std::string EImageDescriberPreset_enumToString(const EImageDescriberPreset preset);

/**
 * @brief It write a describer preset into a stream by converting it to a string. 
 * @param[in] os the stream where to write the preset.
 * @param[in] p the preset to write.
 * @return the modified stream.
 */
std::ostream& operator<<(std::ostream& os, EImageDescriberPreset p);

/**
 * @brief It read a describer preset from a stream. 
 * @param[in] in the stream from which the preset is read.
 * @param[out] p the preset read from the stream.
 * @return the modified stream without the read preset.
 */
std::istream& operator>>(std::istream& in, EImageDescriberPreset& p);


/// A pure virtual class for image description computation
class ImageDescriber
{
public:
  ImageDescriber() {}
  virtual ~ImageDescriber() {}
  
  virtual EImageDescriberType getDescriberType() const = 0;

  /**
  @brief Use a preset to control the number of detected regions
  @param preset The preset configuration
  @return True if configuration succeed.
  */
  virtual bool Set_configuration_preset(EImageDescriberPreset preset) = 0;

  virtual void setUseCuda(bool useCuda) {}
  virtual void setCudaPipe(int pipe) {}

  bool Set_configuration_preset(const std::string& preset)
  {
    return Set_configuration_preset(EImageDescriberPreset_stringToEnum(preset));
  }
  
  /**
   @brief Set image describer always upRight
   @param upRight
   */
  virtual void setUpRight(bool upRight) {}

  /**
  @brief Detect regions on the image and compute their attributes (description)
  @param image Image.
  @param regions The detected regions and attributes
  @param mask 8-bit gray image for keypoint filtering (optional).
     Non-zero values depict the region of interest.
  */
  virtual bool Describe(const image::Image<unsigned char> & image,
    std::unique_ptr<Regions> &regions,
    const image::Image<unsigned char> * mask = nullptr) = 0;

  /// Allocate regions depending of the ImageDescriber
  virtual void Allocate(std::unique_ptr<Regions> &regions) const = 0;

  //--
  // IO - one file for region features, one file for region descriptors
  //--

  virtual bool Load(Regions * regions,
    const std::string& sfileNameFeats,
    const std::string& sfileNameDescs) const
  {
    return regions->Load(sfileNameFeats, sfileNameDescs);
  }

  virtual bool Save(const Regions * regions,
    const std::string& sfileNameFeats,
    const std::string& sfileNameDescs) const
  {
    return regions->Save(sfileNameFeats, sfileNameDescs);
  }

  virtual bool LoadFeatures(Regions * regions,
    const std::string& sfileNameFeats) const
  {
    return regions->LoadFeatures(sfileNameFeats);
  }
};


/**
 * @brief Create the desired ImageDescriber method.
 * Don't use a factory, perform direct allocation.
 */
std::unique_ptr<ImageDescriber> createImageDescriber(EImageDescriberType imageDescriberType);


} // namespace feature
} // namespace aliceVision

#endif // ALICEVISION_FEATURES_IMAGE_DESCRIBER_HPP
