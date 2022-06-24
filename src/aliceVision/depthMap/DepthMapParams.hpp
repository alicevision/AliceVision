// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/mvsUtils/TileParams.hpp>
#include <aliceVision/depthMap/SgmParams.hpp>
#include <aliceVision/depthMap/RefineParams.hpp>

namespace aliceVision {
namespace depthMap {

/**
 * @brief DepthMap Parameters
 */
struct DepthMapParams
{
  mvsUtils::TileParams tileParams;
  SgmParams sgmParams;
  RefineParams refineParams;

  bool mergeTiles = false;
  bool chooseTCamsPerTile = true;
  int maxTCams = 10;
};

} // namespace depthMap
} // namespace aliceVision
