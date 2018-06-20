// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/numeric/numeric.hpp>
#include <aliceVision/feature/imageDescriberCommon.hpp>
#include <aliceVision/feature/Regions.hpp>
#include <aliceVision/image/Image.hpp>
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
  LOW = 0
  , MEDIUM
  , NORMAL
  , HIGH
  , ULTRA
};

/**
 * @brief It returns the preset from a string.
 * @param[in] imageDescriberPreset the input string.
 * @return the associated describer preset.
 */
EImageDescriberPreset EImageDescriberPreset_stringToEnum(const std::string& imageDescriberPreset);

/**
 * @brief It converts a preset to a string.
 * @param[in] imageDescriberPreset the describer preset enum to convert.
 * @return the string associated to the describer preset.
 */
std::string EImageDescriberPreset_enumToString(const EImageDescriberPreset imageDescriberPreset);

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

/**
 * @brief A pure virtual class for image description computation
 */
class ImageDescriber
{
public:
  ImageDescriber() {}

  virtual ~ImageDescriber() {}

  /**
   * @brief Check if the image describer use CUDA
   * @return True if the image describer use CUDA
   */
  virtual bool useCuda() const = 0;

  /**
   * @brief Check if the image describer use float image
   * @return True if the image describer use float image
   */
  virtual bool useFloatImage() const = 0;

  /**
   * @brief Get the corresponding EImageDescriberType
   * @return EImageDescriberType
   */
  virtual EImageDescriberType getDescriberType() const = 0;

  /**
   * @brief Get the total amount of RAM needed for a
   * feature extraction of an image of the given dimension.
   * @param[in] width The image width
   * @param[in] height The image height
   * @return total amount of memory needed
   */
  virtual std::size_t getMemoryConsumption(std::size_t width, std::size_t height) const = 0;

  /**
   * @brief Set image describer always upRight
   * @param[in] upRight
   */
  virtual void setUpRight(bool upRight) {}

  /**
   * @brief Set if yes or no imageDescriber need to use cuda implementation
   * @param[in] useCuda
   */
  virtual void setUseCuda(bool useCuda) {}

  /**
   * @brief set the CUDA pipe
   * @param[in] pipe The CUDA pipe id
   */
  virtual void setCudaPipe(int pipe) {}

  /**
   * @brief Use a preset to control the number of detected regions
   * @param[in] preset The preset configuration
   */
  virtual void setConfigurationPreset(EImageDescriberPreset preset) = 0;

  /**
   * @brief Use a preset to control the number of detected regions
   * @param[in] preset The preset configuration string
   */
  void setConfigurationPreset(const std::string& preset)
  {
    setConfigurationPreset(EImageDescriberPreset_stringToEnum(preset));
  }
  
  /**
   * @brief Detect regions on the 8-bit image and compute their attributes (description)
   * @param[in] image Image.
   * @param[out] regions The detected regions and attributes
   * @param[in] mask 8-bit grayscale image for keypoint filtering (optional)
   * Non-zero values depict the region of interest.
   */
  virtual bool describe(const image::Image<unsigned char>& image,
                        std::unique_ptr<Regions>& regions,
                        const image::Image<unsigned char>* mask = nullptr)
  {
    throw std::logic_error("Cannot use " + EImageDescriberType_enumToString(getDescriberType()) + " image describer with an 8-bit image.");
    return false;
  }

  /**
   * @brief Detect regions on the float image and compute their attributes (description)
   * @param[in] image Image.
   * @param[out] regions The detected regions and attributes
   * @param[in] mask 8-bit grayscale image for keypoint filtering (optional)
   * Non-zero values depict the region of interest.
   */
  virtual bool describe(const image::Image<float>& image,
                        std::unique_ptr<Regions>& regions,
                        const image::Image<unsigned char>* mask = nullptr)
  {
    throw std::logic_error("Cannot use " + EImageDescriberType_enumToString(getDescriberType()) + " image describer with a float image.");
    return false;
  }

  /**
   * @brief Allocate Regions type depending of the ImageDescriber
   * @param[in,out] regions
   */
  virtual void allocate(std::unique_ptr<Regions>& regions) const = 0;

  // IO - one file for region features, one file for region descriptors

  void Load(Regions* regions,
    const std::string& sfileNameFeats,
    const std::string& sfileNameDescs) const
  {
    regions->Load(sfileNameFeats, sfileNameDescs);
  }

  void Save(const Regions* regions,
    const std::string& sfileNameFeats,
    const std::string& sfileNameDescs) const;

  void LoadFeatures(Regions* regions,
    const std::string& sfileNameFeats) const
  {
    regions->LoadFeatures(sfileNameFeats);
  }
};

/**
 * @brief Create the desired ImageDescriber method.
 * Don't use a factory, perform direct allocation.
 */
std::unique_ptr<ImageDescriber> createImageDescriber(EImageDescriberType imageDescriberType);

} // namespace feature
} // namespace aliceVision
