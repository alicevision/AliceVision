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
   * @brief Process media paths and build a list of selected frames
   * @param mediaPaths for each camera, give the media file name (path)
   */
  void processSmart(const std::vector<std::string>& mediaPaths);

  /**
 * @brief build a list of selected frames
 * @param mediaPaths for each camera, give the media file name (path)
 */
  void processSimple(const std::vector<std::string>& mediaPaths);

  /**
  * @Brief write selected frames indices
  * @param outputFOlder folder for output images
  * @param mediaPaths for each camera, give the media file name (path)
  * @param brands for each camera, give the brand name
  * @param bmodelsrands for each camera, give the models name
  * @param mmFocals for each camera, give the focal in mm
  */
  bool writeSelection(const std::string& outputFolder, const std::vector<std::string>& mediaPaths, const std::vector<std::string>& brands, const std::vector<std::string>& models, const std::vector<float>& mmFocals);

  /**
   * @brief Set min frame step for process algorithm
   * @param[in] frameStep minimum number of frames between two keyframes
   */
  void setMinFrameStep(unsigned int frameStep)
  {
      _minFrameStep = frameStep;
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
   * @brief Get min frame step for process algorithm
   * @return minimum number of frames between two keyframes
   */
  unsigned int getMinFrameStep() const 
  {
      return _minFrameStep;
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
  /// Minimum number of frame between two keyframes
  unsigned int _minFrameStep = 12;
  /// Maximum number of output frame (0 = no limit)
  unsigned int _maxOutFrame = 0;

  /// List of selected frames
  std::vector<unsigned int> _selected;
};

} // namespace keyframe 
} // namespace aliceVision
