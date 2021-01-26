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

inline std::string EImageDescriberPreset_information()
{
    return "Feature preset controls the density of the feature extraction:\n"
           "* LOW: Very low density (max 1K points).\n"
           "* MEDIUM: Low density (max 5K points).\n"
           "* NORMAL: Default feature density (max 10K points).\n"
           "* HIGH: High density (max 50K points).\n"
           "* ULTRA: Very high density (max 100K points). Can use large amount of storage and large amount of computation. Use only on small datasets.\n";
}

EImageDescriberPreset EImageDescriberPreset_stringToEnum(const std::string& imageDescriberPreset);
std::string EImageDescriberPreset_enumToString(const EImageDescriberPreset imageDescriberPreset);
std::ostream& operator<<(std::ostream& os, EImageDescriberPreset p);
std::istream& operator>>(std::istream& in, EImageDescriberPreset& p);

/**
 * @brief The quality of the detection. It's a trade-off between performance and precision.
 */
enum class EFeatureQuality
{
    LOW = 0,
    MEDIUM,
    NORMAL,
    HIGH,
    ULTRA,
};

inline std::string EFeatureQuality_information()
{
    return "Feature extraction contains a trade-off between speed and result accuracy:\n"
           "* LOW: Very quick results.\n"
           "* MEDIUM: Quick results.\n"
           "* NORMAL: Default feature quality.\n"
           "* HIGH: Improved quality over performances.\n"
           "* ULTRA: Highest quality at the expense of high computational cost.\n";
}

EFeatureQuality EFeatureQuality_stringToEnum(const std::string& v);
std::string EFeatureQuality_enumToString(const EFeatureQuality v);
std::ostream& operator<<(std::ostream& os, EFeatureQuality v);
std::istream& operator>>(std::istream& in, EFeatureQuality& v);


/**
 * @brief The method used to filter out features with too low constrast (that can be considered as noise).
 */
enum class EFeatureConstrastFiltering
{
    /// Use a fixed threshold for all the pixels
    Static = 0,
    /// Use a threshold for each image based on image statistics
    AdaptiveToMedianVariance,
    /// Disable contrast filtering
    NoFiltering,
    /// Grid sort by peak value per octave and by scale at the end
    GridSortOctaves,
    /// Grid sort by scale*peakValue per octave and at the end
    GridSort,
    /// Grid sort per scale steps and at the end (scale and then peak value)
    GridSortScaleSteps,
    /// Grid sort per octaves and at the end (scale and then peak value)
    GridSortOctaveSteps,
    /// Filter non-extrema peak values
    NonExtremaFiltering
};

inline std::string EFeatureConstrastFiltering_information()
{
    return "Contrast filtering method to ignore features with too low contrast that can be consided as noise:\n"
           "* Static: Fixed threshold.\n"
           "* AdaptiveToMedianVariance: Based on image content analysis.\n"
           "* NoFiltering: Disable contrast filtering.\n"
           "* GridSortOctaves: Grid sort by peak value per octave and by scale at the end.\n"
           "* GridSort: Grid sort by scale*peakValue per octave and at the end.\n"
           "* GridSortScaleSteps: Grid sort per scale steps and at the end (scale and then peak value).\n"
           "* GridSortOctaveSteps: Grid sort per octaves and at the end (scale and then peak value).\n"
           "* NonExtremaFiltering: Filter non-extrema peak values.\n";
}

EFeatureConstrastFiltering EFeatureConstrastFiltering_stringToEnum(const std::string& v);
std::string EFeatureConstrastFiltering_enumToString(const EFeatureConstrastFiltering v);
std::ostream& operator<<(std::ostream& os, EFeatureConstrastFiltering v);
std::istream& operator>>(std::istream& in, EFeatureConstrastFiltering& v);


struct ConfigurationPreset
{
    EImageDescriberPreset descPreset{EImageDescriberPreset::NORMAL};
    int maxNbFeatures{0};
    EFeatureQuality quality{EFeatureQuality::NORMAL};
    bool gridFiltering{true};
    EFeatureConstrastFiltering contrastFiltering{EFeatureConstrastFiltering::Static};
    float relativePeakThreshold{0.02f};

    inline ConfigurationPreset& setDescPreset(EImageDescriberPreset v)
    {
        descPreset = v;
        return *this;
    }
    inline ConfigurationPreset& setDescPreset(const std::string& v)
    {
        descPreset = EImageDescriberPreset_stringToEnum(v);
        return *this;
    }

    inline ConfigurationPreset& setGridFiltering(bool v)
    {
        gridFiltering = v;
        return *this;
    }

    inline ConfigurationPreset& setContrastFiltering(EFeatureConstrastFiltering v)
    {
        contrastFiltering = v;
        return *this;
    }
    inline ConfigurationPreset& setContrastFiltering(const std::string& v)
    {
        contrastFiltering = EFeatureConstrastFiltering_stringToEnum(v);
        return *this;
    }
};

/**
 * @brief A pure virtual class for image description computation
 */
class ImageDescriber
{
public:
  ImageDescriber() = default;

  virtual ~ImageDescriber() = default;

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
  virtual void setConfigurationPreset(ConfigurationPreset preset) = 0;

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
