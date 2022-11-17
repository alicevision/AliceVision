// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

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


class KeyframeSelector
{
public:    

  /**
   * @brief Process media paths and build a list of selected frames using a smart method based on optical flow estimation
   * @param[in] mediaPaths for each camera, give the media file name (path)
   */
  void processSmart(const std::vector<std::string>& mediaPaths);

  /**
   * @brief Process media paths and build a list of selected frames using a regular sampling over time
   * @param[in] mediaPaths for each camera, give the media file name (path)
   */
  void processSimple(const std::vector<std::string>& mediaPaths);

  /**
   * @brief Write selected frames indices
   * @param[in] outputFolder folder for output images
   * @param[in] mediaPaths for each camera, give the media file name (path)
   * @param[in] brands for each camera, give the brand name
   * @param[in] models for each camera, give the models name
   * @param[in] mmFocals for each camera, give the focal in mm
   */
  bool writeSelection(const std::string& outputFolder, const std::vector<std::string>& mediaPaths, const std::vector<std::string>& brands, const std::vector<std::string>& models, const std::vector<float>& mmFocals);

  /**
   * @brief Set the mininum frame step for the processing algorithm
   * @param[in] frameStep minimum number of frames between two keyframes
   */
  void setMinFrameStep(unsigned int frameStep)
  {
      _minFrameStep = frameStep;
  }

  /**
   * @brief Set the maximum number of output frames for the processing algorithm
   * @param[in] nbFrame maximum number of output frames (if 0, no limit)
   */
  void setMaxOutFrame(unsigned int nbFrame)
  {
      _maxOutFrame = nbFrame;
  }

  /**
   * @brief Get the minimum frame step for the processing algorithm
   * @return minimum number of frames between two keyframes
   */
  unsigned int getMinFrameStep() const 
  {
      return _minFrameStep;
  }

  /**
   * @brief Get the maximum number of output frames for the processing algorithm
   * @return maximum number of output frames (if 0, no limit)
   */
  unsigned int getMaxOutFrame() const 
  {
      return _maxOutFrame;
  }
    
private:

  /// Minimum number of frame between two keyframes
  unsigned int _minFrameStep = 12;
  /// Maximum number of output frame (0 = no limit)
  unsigned int _maxOutFrame = 0;

  /// List of selected frames
  std::vector<unsigned int> _selected;
};

} // namespace keyframe 
} // namespace aliceVision
