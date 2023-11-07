// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/mvsData/ROI.hpp>
#include <vector>

namespace aliceVision {
namespace mvsUtils {

/**
 * @brief Tile Parameters
 * This structure handle tiling user parameters.
 */
struct TileParams
{
    // user parameters

    int bufferWidth = 1024;
    int bufferHeight = 1024;
    int padding = 64;
};

/**
 * @brief Check if the given image size can contain only one tile
 * @param[in] tileParams the tile parameters
 * @param[in] imageWidth the image width
 * @param[in] imageHeight the image height
 * @return true if single tile case
 */
inline bool hasOnlyOneTile(const TileParams& tileParams, int imageWidth, int imageHeight)
{
    return (tileParams.bufferHeight >= imageWidth && tileParams.bufferHeight >= imageHeight);
}

/**
 * @brief Get tile list from tile parameters and image width/height
 * @param[in] tileParams the tile parameters
 * @param[in] imageWidth the image width
 * @param[in] imageHeight the image height
 * @param[in] maxDownscale the maximum downscale that can be applied to the image
 * @param[out] out_tileRoiList the output tile ROI list
 */
void getTileRoiList(const TileParams& tileParams, int imageWidth, int imageHeight, int maxDownscale, std::vector<ROI>& out_tileRoiList);

/**
 * @brief Log tile list and tile parameters
 * @param[in] tileParams the tile parameters
 * @param[in] imageWidth the image width used for the tile ROI list computation
 * @param[in] imageHeight the image height used for the tile ROI list computation
 * @param[in] maxDownscale the maximum downscale that can be applied to the image
 * @param[in] in_tileRoiList the tile ROI list
 */
void logTileRoiList(const TileParams& tileParams, int imageWidth, int imageHeight, int maxDownscale, const std::vector<ROI>& in_tileRoiList);

}  // namespace mvsUtils
}  // namespace aliceVision
