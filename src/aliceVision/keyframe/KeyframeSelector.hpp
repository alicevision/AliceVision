// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/dataio/FeedProvider.hpp>

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
public:
  /**
   * @brief KeyframeSelector constructor
   * @param[in] mediaPath video file path or image sequence directory
   * @param[in] sensorDbPath camera sensor width database path
   * @param[in] outputFolder output keyframes directory
   */
  KeyframeSelector(const std::vector<std::string>& mediaPaths,
                   const std::string& sensorDbPath,
                   const std::string& outputFolder);

  /**
   * @brief KeyframeSelector copy constructor - NO COPY
   * @param[in] copy keyframeSelector
   */
  KeyframeSelector(const KeyframeSelector& copy) = delete;

  /**
   * @brief Process media paths and build a list of selected keyframes using a regular sampling over time
   */
  void processRegular();

  /**
   * @brief Write the selected keyframes in the output folder
   * @param[in] brands brand name for each camera
   * @param[in] models model name for each camera
   * @param[in] mmFocals focal in millimeters for each camera
   * @return true if all the selected keyframes were successfully written, false otherwise
   */
  bool writeSelection(const std::vector<std::string>& brands, const std::vector<std::string>& models,
                 const std::vector<float>& mmFocals) const;

  /**
   * @brief Set the minimum frame step parameter for the processing algorithm
   * @param[in] frameStep minimum number of frames between two keyframes
   */
  void setMinFrameStep(unsigned int frameStep)
  {
      _minFrameStep = frameStep;
  }

  /**
   * @brief Set the maximum frame step parameter for the processing algorithm
   * @param[in] frameStep maximum number of frames between two keyframes
   */
  void setMaxFrameStep(unsigned int frameStep)
  {
      _maxFrameStep = frameStep;
  }

  /**
   * @brief Set the maximum output frame number parameter for the processing algorithm
   * @param[in] nbFrame maximum number of output frames (if 0, no limit)
   */
  void setMaxOutFrame(unsigned int nbFrame)
  {
      _maxOutFrame = nbFrame;
  }

  /**
   * @brief Get the minimum frame step parameter for the processing algorithm
   * @return minimum number of frames between two keyframes
   */
  unsigned int getMinFrameStep() const 
  {
      return _minFrameStep;
  }

  /**
   * @brief Get the maximum output frame number parameter for the processing algorithm
   * @return maximum number of frames between two keyframes
   */
  unsigned int getMaxFrameStep() const 
  {
      return _maxFrameStep;
  }

  /**
   * @brief Get the max output frame number for process algorithm
   * @return maximum number of output frames (if 0, no limit)
   */
  unsigned int getMaxOutFrame() const 
  {
      return _maxOutFrame;
  }
    
private:
  /// Selected keyframes IDs
  std::vector<unsigned int> _selectedKeyframes;

  /// Media paths
  std::vector<std::string> _mediaPaths;
  /// Camera sensor width database
  std::string _sensorDbPath;
  /// Output folder for keyframes
  std::string _outputFolder;


  /// Minimum number of frames between two keyframes
  unsigned int _minFrameStep = 12;
  /// Maximum number of frames between two keyframes
  unsigned int _maxFrameStep = 36;
  /// Maximum number of output frames (0 = no limit)
  unsigned int _maxOutFrame = 0;
};

} // namespace keyframe 
} // namespace aliceVision
