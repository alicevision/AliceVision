// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/mvsUtils/MultiViewParams.hpp>
#include <aliceVision/mvsUtils/TileParams.hpp>
#include <aliceVision/image/Image.hpp>

#include <string>

namespace aliceVision {
namespace mvsUtils {

/**
 * @brief Add a tile to a full map with weighting.
 * @param[in] rc the related R camera index
 * @param[in] mp the multi-view parameters
 * @param[in] tileParams tile workflow parameters
 * @param[in] roi the 2d region of interest without any downscale apply
 * @param[in] downscale the map downscale factor
 * @param[in] in_tileMap the tile map to add
 * @param[in,out] inout_map the full output map
 */
void addTileMapWeighted(int rc,
                        const MultiViewParams& mp,
                        const TileParams& tileParams,
                        const ROI& roi,
                        int downscale,
                        image::Image<float>& in_tileMap,
                        image::Image<float>& inout_map);

/**
 * @brief Read a fullsize map from file(s).
 * @param[in] rc the related R camera index
 * @param[in] mp the multi-view parameters
 * @param[in] fileType the map fileType enum
 * @param[out] out_map the output map read from file(s)
 * @param[in] scale the map downscale factor
 * @param[in] step the map step factor
 * @param[in] customSuffix the map filename custom suffix
 */
void readMap(int rc,
             const MultiViewParams& mp,
             const EFileType fileType,
             image::Image<float>& out_map,
             int scale = 1,
             int step = 1,
             const std::string& customSuffix = "");

void readMap(int rc,
             const MultiViewParams& mp,
             const EFileType fileType,
             image::Image<image::RGBfColor>& out_map,
             int scale = 1,
             int step = 1,
             const std::string& customSuffix = "");

/**
 * @brief Write a fullsize or tile map in a file.
 * @param[in] rc the related R camera index
 * @param[in] mp the multi-view parameters
 * @param[in] fileType the map fileType enum
 * @param[in] tileParams tile workflow parameters
 * @param[in] roi the 2d region of interest without any downscale apply
 * @param[in] in_map the input fullsize or tile map to write
 * @param[in] scale the map downscale factor
 * @param[in] step the map step factor
 * @param[in] customSuffix the map filename custom suffix
 */
void writeMap(int rc,
              const MultiViewParams& mp,
              const EFileType fileType,
              const TileParams& tileParams,
              const ROI& roi,
              const image::Image<float>& in_map,
              int scale,
              int step,
              const std::string& customSuffix = "");

void writeMap(int rc,
              const MultiViewParams& mp,
              const EFileType fileType,
              const TileParams& tileParams,
              const ROI& roi,
              const image::Image<image::RGBfColor>& in_map,
              int scale,
              int step,
              const std::string& customSuffix = "");

/**
 * @brief Write a fullsize map in a file.
 * @param[in] rc the related R camera index
 * @param[in] mp the multi-view parameters
 * @param[in] fileType the map fileType enum
 * @param[in] in_map the input fullsize map to write
 * @param[in] scale the map downscale factor
 * @param[in] step the map step factor
 * @param[in] customSuffix the map filename custom suffix
 */
inline void writeMap(int rc,
                     const MultiViewParams& mp,
                     const EFileType fileType,
                     const image::Image<float>& in_map,
                     int scale = 1,
                     int step = 1,
                     const std::string& customSuffix = "")
{
    const TileParams tileParams;                                   // default tile parameters, no tiles
    const ROI roi = ROI(0, mp.getWidth(rc), 0, mp.getHeight(rc));  // fullsize roi
    writeMap(rc, mp, fileType, tileParams, roi, in_map, scale, step, customSuffix);
}

inline void writeMap(int rc,
                     const MultiViewParams& mp,
                     const EFileType fileType,
                     const image::Image<image::RGBfColor>& in_map,
                     int scale = 1,
                     int step = 1,
                     const std::string& customSuffix = "")
{
    const TileParams tileParams;                                   // default tile parameters, no tiles
    const ROI roi = ROI(0, mp.getWidth(rc), 0, mp.getHeight(rc));  // fullsize roi
    writeMap(rc, mp, fileType, tileParams, roi, in_map, scale, step, customSuffix);
}

/**
 * @brief Get depth map number of depth values from metadata or computation.
 * @param[in] rc the related R camera index
 * @param[in] mp the multi-view parameters
 * @param[in] scale the depth map downscale factor
 * @param[in] step the depth map step factor
 * @param[in] customSuffix the filename custom suffix
 */
unsigned long getNbDepthValuesFromDepthMap(int rc, const MultiViewParams& mp, int scale = 1, int step = 1, const std::string& customSuffix = "");

/**
 * @brief Delete map tiles files.
 * @param[in] rc the related R camera index
 * @param[in] mp the multi-view parameters
 * @param[in] fileType the map fileType enum
 * @param[in] customSuffix the filename custom suffix
 */
void deleteMapTiles(int rc, const MultiViewParams& mp, const EFileType fileType, const std::string& customSuffix = "");

}  // namespace mvsUtils
}  // namespace aliceVision
