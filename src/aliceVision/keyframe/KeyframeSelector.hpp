// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/keyframe/SharpnessSelectionPreset.hpp>
#include <aliceVision/feature/feature.hpp>
#include <aliceVision/dataio/FeedProvider.hpp>
#include <aliceVision/voctree/VocabularyTree.hpp>

#include <OpenImageIO/imageio.h>

#include <string>
#include <vector>
#include <memory>
#include <limits>

namespace aliceVision {
namespace image {

template<typename T>
class Image;

} // namespace image

namespace keyframe {

namespace oiio = OIIO;

class KeyframeSelector
{
private:
  // SIFT descriptor definition
  const static std::size_t _dimension = 128;
  using DescriptorFloat = aliceVision::feature::Descriptor<float, _dimension>;
  
public:

  /**
   * @brief Camera informations
   */
  struct CameraInfo {
    /// Camera brand
    std::string brand = "Custom";
    /// Camera model
    std::string model = "radial3";
    /// Focal length in mm or px
    float focalLength = 1.2f;
    /// Camera frame offset
    unsigned int frameOffset = 0;
    /// If focalIsMM is false, focalLength is in px
    bool focalIsMM = true;
  };
    
  /**
   * @brief KeyframeSelector constructor
   * @param[in] mediaPath video file path or image sequence directory
   * @param[in] sensorDbPath camera sensor width database path
   * @param[in] voctreeFilePath vocabulary tree path
   * @param[in] outputFolder output keyframes directory
   */
  KeyframeSelector(const std::vector<std::string>& mediaPaths,
                   const std::string& sensorDbPath,
                   const std::string& voctreeFilePath,
                   const std::string& outputFolder);

  /**
   * @brief KeyframeSelector copy constructor - NO COPY
   * @param[in] copy keyframeSelector
   */
  KeyframeSelector(const KeyframeSelector& copy) = delete;

  /**
   * @brief Process media paths and extract keyframes
   */
  void process();

  /**
   * @brief Set if selector use keyframe sparse distance selection
   * @param[in] useSparseDistanceSelection True or False
   */
  void useSparseDistanceSelection(bool useSparseDistanceSelection)
  {
    _hasSparseDistanceSelection = useSparseDistanceSelection;
  }

  /**
   * @brief Set if selector use keyframe sharpness selection
   * @param[in] useSharpnessSelection True or False
   */
  void useSharpnessSelection(bool useSharpnessSelection)
  {
    _hasSharpnessSelection = useSharpnessSelection;
  }

  /**
   * @brief Set cameras informations for output keyframes
   * @param[in] cameras informations
   */
  void setCameraInfos(const std::vector<CameraInfo>& cameraInfos)
  {
    _cameraInfos = cameraInfos;
  }

  /**
   * @brief Set sparse distance max score
   * @param[in] distScoreMax max strong common points
   */
  void setSparseDistanceMaxScore(float distScoreMax)
  {
    _distScoreMax = distScoreMax;
  }

  /**
   * @brief Set Sharpness selection preset
   * @param[in] sharpnessPreset enum
   */
  void setSharpnessSelectionPreset(ESharpnessSelectionPreset sharpnessPreset)
  {
    switch(sharpnessPreset)
    {
      // arbitrary thresholds
      case ESharpnessSelectionPreset::ULTRA:    _sharpnessThreshold = 20.0f;  break;
      case ESharpnessSelectionPreset::HIGH:     _sharpnessThreshold = 17.0f;  break;
      case ESharpnessSelectionPreset::NORMAL:   _sharpnessThreshold = 15.0f;  break;
      case ESharpnessSelectionPreset::MEDIUM:   _sharpnessThreshold = 10.0f;  break;
      case ESharpnessSelectionPreset::LOW:      _sharpnessThreshold =  8.0f;  break;
      case ESharpnessSelectionPreset::VERY_LOW: _sharpnessThreshold =  6.0f;  break;
      case ESharpnessSelectionPreset::NONE:     _sharpnessThreshold =   .0f;  break;
      default: throw std::out_of_range("Invalid sharpnessPreset enum");
    }
  }

  /**
   * @brief Set sharp subset size for process algorithm
   * @param[in] subset sharp part of the image (1 = all, 2 = size/2, ...)
   */
  void setSharpSubset(unsigned int subset)
  {
      _sharpSubset = subset;
  }

  /**
   * @brief Set min frame step for process algorithm
   * @param[in] frameStep minimum number of frames between two keyframes
   */
  void setMinFrameStep(unsigned int frameStep)
  {
      _minFrameStep = frameStep;
  }

  /**
   * @brief Set max frame step for process algorithm
   * @param[in] frameStep maximum number of frames after which a keyframe can be taken
   */
  void setMaxFrameStep(unsigned int frameStep)
  {
      _maxFrameStep = frameStep;
  }

  /**
   * @brief Set max output frame number for process algorithm
   * @param[in] nbFrame maximum number of output frames (if 0, no limit)
   */
  void setMaxOutFrame(unsigned int nbFrame)
  {
      _maxOutFrame = nbFrame;
  }

  /**
   * @brief Get sharp subset size for process algorithm
   * @return sharp part of the image (1 = all, 2 = size/2, ...)
   */
  unsigned int getSharpSubset() const 
  {
      return _sharpSubset;
  }

  /**
   * @brief Get min frame step for process algorithm
   * @return minimum number of frames between two keyframes
   */
  unsigned int getMinFrameStep() const 
  {
      return _minFrameStep;
  }

  /**
   * @brief Get max output frame number for process algorithm
   * @return maximum number of frames for trying to select a keyframe
   */
  unsigned int getMaxFrameStep() const 
  {
      return _maxFrameStep;
  }

  /**
   * @brief Get max output frame number for process algorithm
   * @return maximum number of output frames (if 0, no limit)
   */
  unsigned int getMaxOutFrame() const 
  {
      return _maxOutFrame;
  }
    
private:

  // Paths

  /// Media paths
  std::vector<std::string> _mediaPaths;
  /// Camera sensor width database
  std::string _sensorDbPath;
  /// Voctree file path
  std::string _voctreeFilePath;
  /// Output folder for keyframes
  std::string _outputFolder;

  // Algorithm variables

  /// Sharp part of the image (1 = all, 2 = size/2, ...)
  unsigned int _sharpSubset = 4; 
  /// Minimum number of frame between two keyframes
  unsigned int _minFrameStep = 12;
  /// Maximum number of frame for evaluation
  unsigned int _maxFrameStep = 36;
  /// Maximum number of output frame (0 = no limit)
  unsigned int _maxOutFrame = 0;
  /// Number of tiles per side
  unsigned int _nbTileSide = 20;
  /// Number of previous keyframe distances in order to evaluate distance score
  unsigned int _nbKeyFrameDist = 10;
  /// Use padding on digits for exported frames
  unsigned int _padding = 7;
  /// Sharpness threshold (image with higher sharpness will be selected)
  float _sharpnessThreshold = 15.0f;
  /// Distance max score (image with smallest distance from the last keyframe will be selected)
  float _distScoreMax = 100.0f;
  /// Use sharpness selection
  bool _hasSharpnessSelection = true;
  /// Use sparseDistance selection
  bool _hasSparseDistanceSelection = true;

  /// Camera metadatas
  std::vector<CameraInfo> _cameraInfos;

  // Tools

  /// Image describer in order to extract describer
  std::unique_ptr<feature::ImageDescriber> _imageDescriber;
  /// Voctree in order to compute sparseHistogram
  std::unique_ptr< aliceVision::voctree::VocabularyTree<DescriptorFloat> > _voctree;
  /// Feed provider for media paths images extraction
  std::vector< std::unique_ptr<dataio::FeedProvider> > _feeds;

  // Process structures

  /**
   * @brief Process media global informations
   */
  struct MediaInfo
  {
    /// height of the tile
    unsigned int tileHeight = 0;
    /// width of the tile
    unsigned int tileWidth = 0;
    /// openImageIO image spec
    oiio::ImageSpec spec;
  };

  /**
   * @brief Process media informations at a specific frame
   */
  struct MediaData
  {
    /// sharpness score
    float sharpness = 0;
    /// maximum distance score with keyframe media histograms
    float distScore = 0;
    /// sparseHistogram
    voctree::SparseHistogram histogram;
  };

  /**
   * @brief Process frame (or set of frames) informations
   */
  struct FrameData
  {
    /// average sharpness score of all media
    float avgSharpness = 0;
    /// maximum voctree distance score of all media
    float maxDistScore = 0;
    /// frame (or set of frames) selected for evaluation
    bool selected = false;
    /// frame is a keyframe
    bool keyframe = false;
    /// medias process data
    std::vector<MediaData> mediasData;

    /**
     * @brief Compute average sharpness score
     */
    void computeAvgSharpness()
    {
      for(const auto& media : mediasData)
        avgSharpness += media.sharpness;
      avgSharpness /= mediasData.size();
    }
  };

  /// MediaInfo structure per input medias
  std::vector<MediaInfo> _mediasInfo;
  /// FrameData structure per frame
  std::vector<FrameData> _framesData;
  /// Keyframe indexes container
  std::vector<std::size_t> _keyframeIndexes;

  /**
   * @brief Compute sharpness score of a given image
   * @param[in] imageGray given image in grayscale
   * @param[in] tileHeight height of tile
   * @param[in] tileWidth width of tile
   * @param[in] tileSharpSubset number of sharp tiles
   * @return sharpness score
   */
  float computeSharpness(const image::Image<float>& imageGray,
                         const unsigned int tileHeight,
                         const unsigned int tileWidth,
                         const unsigned int tileSharpSubset) const;

  /**
   * @brief Compute sharpness and distance score for a given image
   * @param[in] image an image of the media
   * @param[in] frameIndex the image index in the media sequence
   * @param[in] mediaIndex the media index
   * @param[in] tileSharpSubset number of sharp tiles
   * @return true if the frame is selected
   */
  bool computeFrameData(const image::Image<image::RGBColor>& image,
                        std::size_t frameIndex,
                        std::size_t mediaIndex,
                        unsigned int tileSharpSubset);

  /**
   * @brief Write a keyframe and metadata
   * @param[in] image an image of the media
   * @param[in] frameIndex the image index in the media sequence
   * @param[in] mediaIndex the media index
   */
  void writeKeyframe(const image::Image<image::RGBColor>& image, 
                     std::size_t frameIndex,
                     std::size_t mediaIndex);

  /**
   * @brief Convert focal length from px to mm using sensor width database
   * @param[in] camera informations
   * @param[in] imageWidth media image width in px
   */
  void convertFocalLengthInMM(CameraInfo& cameraInfo, int imageWidth);
};

} // namespace keyframe 
} // namespace aliceVision
