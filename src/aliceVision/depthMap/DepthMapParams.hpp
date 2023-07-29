// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/mvsUtils/TileParams.hpp>
#include <aliceVision/depthMap/CustomPatchPatternParams.hpp>
#include <aliceVision/depthMap/SgmParams.hpp>
#include <aliceVision/depthMap/RefineParams.hpp>

namespace aliceVision {
namespace depthMap {

/**
 * @brief Depth Map Parameters
 */
struct DepthMapParams
{
  // user parameters

  int maxTCams = 10;                  //< global T cameras maximum
  bool chooseTCamsPerTile = true;     //< choose T cameras per R tile or for the entire R image
  bool exportTilePattern = false;     //< export tile pattern obj
  bool autoAdjustSmallImage = true;   //< allow program to override parameters for the single tile case

  /// user custom patch pattern for similarity volume computation (both SGM & Refine)
  CustomPatchPatternParams customPatchPattern;

  // constant parameters

  const bool useRefine = true;        //< for debug purposes: enable or disable Refine process
};

} // namespace depthMap
} // namespace aliceVision
