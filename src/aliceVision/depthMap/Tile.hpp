// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/mvsData/ROI.hpp>

#include <vector>
#include <ostream>

namespace aliceVision {
namespace depthMap {

/**
 * @struct Depth Map Tile Structure
 * @brief Support class to keep tile information.
 */
struct Tile
{
    int id;                       //< tile index
    int nbTiles;                  //< number of tiles per image
    int rc;                       //< related R camera index
    std::vector<int> sgmTCams;    //< SGM T camera index list
    std::vector<int> refineTCams; //< Refine T camera index list
    ROI roi;                      //< 2d region of interest of the R image
};

inline std::ostream& operator<<(std::ostream& os, const Tile& tile)
{
    os << "(rc: " << tile.rc << ", tile: " << (tile.id + 1) << "/" << tile.nbTiles << ") ";
    return os;
}

} // namespace depthMap
} // namespace aliceVision
