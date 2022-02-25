// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

namespace aliceVision {
namespace depthMap {

/**
 * @brief Tile Parameters
 */
struct TileParams
{
  // user parameters

  int width = -1;
  int height = -1;
  int padding = -1;
};

} // namespace depthMap
} // namespace aliceVision
