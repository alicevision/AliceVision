// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/system/Logger.hpp>
#include <aliceVision/mvsData/ROI.hpp>
#include <vector>

namespace aliceVision {
namespace mvsUtils {

/**
 * @brief Tile Parameters
 *
 * Tile size is in the coordinate system of the original input image (without any downscale applied).
 */
struct TileParams
{
  // user parameters

  int width  = 1024;
  int height = 1024;
  int padding = 128;
};

 /**
 * @brief Get tile list from tile parameters and image width/height
 * @param[in] tileParams the tile parameters
 * @param[in] imageWidth the image width
 * @param[in] imageHeight the image height
 * @param[out] out_tileList the output tile ROI list
 */
void getTileList(const TileParams& tileParams, int imageWidth, int imageHeight, std::vector<ROI>& out_tileList);

} // namespace mvsUtils
} // namespace aliceVision
