// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/mvsData/ROI.hpp>
#include <aliceVision/mvsUtils/MultiViewParams.hpp>
#include <aliceVision/mvsUtils/TileParams.hpp>

#include <vector>
#include <string>

namespace aliceVision {
namespace mvsUtils {

/**
 * @brief Add a tile to a full map with weighting
 * @param[in] rc the related R camera index
 * @param[in] mp the multi-view parameters
 * @param[in] tileParams tile workflow parameters
 * @param[in] roi the 2d region of interest without any downscale apply
 * @param[in] downscale the depth/sim map downscale factor
 * @param[in] in_tileMap the tile map buffer to add
 * @param[in,out] inout_map the full output map
 */
void addTileMapWeighted(int rc,
                         const MultiViewParams& mp,
                         const TileParams& tileParams,
                         const ROI& roi,
                         int downscale,
                         std::vector<float>& in_tileMap,
                         std::vector<float>& inout_map);

/**
 * @brief Write the depth map and the similarity map
 * @param[in] rc the related R camera index
 * @param[in] mp the multi-view parameters
 * @param[in] tileParams tile workflow parameters
 * @param[in] roi the 2d region of interest without any downscale apply
 * @param[in] depthMap the corresponding depth map buffer
 * @param[in] simMap the corresponding similarity map buffer
 * @param[in] scale the depth/sim map downscale factor
 * @param[in] step the depth/sim map step factor
 * @param[in] customSuffix the filename custom suffix
 */
void writeDepthSimMap(int rc, 
                      const MultiViewParams& mp, 
                      const TileParams& tileParams, 
                      const ROI& roi,
                      const std::vector<float>& depthMap, 
                      const std::vector<float>& simMap, 
                      int scale,
                      int step,
                      const std::string& customSuffix = "");

/**
 * @brief Write the depth map and the similarity map
 * @param[in] rc the related R camera index
 * @param[in] mp the multi-view parameters
 * @param[in] depthMap the corresponding depth map buffer
 * @param[in] simMap the corresponding similarity map buffer
 * @param[in] scale the depth/sim map downscale factor
 * @param[in] step the depth/sim map step factor
 * @param[in] customSuffix the filename custom suffix
 */
void writeDepthSimMap(int rc, 
                      const MultiViewParams& mp,
                      const std::vector<float>& depthMap, 
                      const std::vector<float>& simMap, 
                      int scale = 1,
                      int step = 1,
                      const std::string& customSuffix = "");

/**
 * @brief Write the depth map 
 * @param[in] rc the related R camera index
 * @param[in] mp the multi-view parameters
 * @param[in] depthMap the corresponding depth map buffer
 * @param[in] scale the depth/sim map downscale factor
 * @param[in] step the depth/sim map step factor
 * @param[in] customSuffix the filename custom suffix
 */
void writeDepthMap(int rc, 
                   const MultiViewParams& mp,
                   const std::vector<float>& depthMap, 
                   int scale = 1,
                   int step = 1,
                   const std::string& customSuffix = "");

/**
 * @brief read the depth map and the similarity map from files
 * @param[in] rc the related R camera index
 * @param[in] mp the multi-view parameters
 * @param[out] out_depthMap the corresponding depth map buffer
 * @param[out] out_simMap the corresponding similarity map buffer
 * @param[in] scale the depth/sim map downscale factor
 * @param[in] step the depth/sim map step factor
 * @param[in] customSuffix the filename custom suffix
 */
void readDepthSimMap(int rc, 
                     const MultiViewParams& mp,
                     std::vector<float>& out_depthMap, 
                     std::vector<float>& out_simMap, 
                     int scale = 1,
                     int step = 1,
                     const std::string& customSuffix = "");

/**
 * @brief read the depth map from file(s)
 * @param[in] rc the related R camera index
 * @param[in] mp the multi-view parameters
 * @param[out] out_depthMap the corresponding depth map buffer
 * @param[in] scale the depth/sim map downscale factor
 * @param[in] step the depth/sim map step factor
 * @param[in] customSuffix the filename custom suffix
 */
void readDepthMap(int rc, 
                  const MultiViewParams& mp,
                  std::vector<float>& out_depthMap, 
                  int scale = 1,
                  int step = 1,
                  const std::string& customSuffix = "");

/**
 * @brief read the similarity map from file(s)
 * @param[in] rc the related R camera index
 * @param[in] mp the multi-view parameters
 * @param[out] out_simMap the corresponding similarity map buffer
 * @param[in] scale the depth/sim map downscale factor
 * @param[in] step the depth/sim map step factor
 * @param[in] customSuffix the filename custom suffix
 */
void readSimMap(int rc, 
                const MultiViewParams& mp,
                std::vector<float>& out_simMap, 
                int scale = 1,
                int step = 1,
                const std::string& customSuffix = "");

/**
 * @brief Get depth map number of depth values from metadata or count
 * @param[in] rc the related R camera index
 * @param[in] mp the multi-view parameters
 * @param[in] scale the depth/sim map downscale factor
 * @param[in] step the depth/sim map step factor
 * @param[in] customSuffix the filename custom suffix
 */
unsigned long getNbDepthValuesFromDepthMap(int rc, 
                                           const MultiViewParams& mp,
                                           int scale = 1,
                                           int step = 1,
                                           const std::string& customSuffix = "");

} // namespace mvsUtils
} // namespace aliceVision
