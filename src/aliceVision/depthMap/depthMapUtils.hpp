// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/mvsData/ROI.hpp>
#include <aliceVision/mvsUtils/MultiViewParams.hpp>
#include <aliceVision/mvsUtils/TileParams.hpp>
#include <aliceVision/depthMap/Tile.hpp>
#include <aliceVision/depthMap/cuda/host/memory.hpp>

#include <vector>
#include <string>

namespace aliceVision {
namespace depthMap {

/**
 * @brief Copy an image from device memory to host memory and write on disk.
 * @note  This function can be useful for code analysis and debugging. 
 * @param[in] in_img_dmp the image in device memory
 * @param[in] path the path of the output image on disk
 */
void writeDeviceImage(const CudaDeviceMemoryPitched<CudaRGBA, 2>& in_img_dmp, const std::string& path);

/**
 * @brief Copy an image from device memory to host memory and write on disk.
 * @note  This function can be useful for code analysis and debugging.
 * @param[in] in_img_dmp the image in device memory
 * @param[in] path the path of the output image on disk
 */
void writeDeviceImage(const CudaDeviceMemoryPitched<float3, 2>& in_img_dmp, const std::string& path);

/**
 * @brief Reset a depth/similarity map in host memory to the given default depth and similarity.
 * @param[in,out] inout_depthSimMap_hmh the depth/similarity map in host memory
 * @param[in] depth the depth reset value
 * @param[in] sim the sim reset value
 */
void resetDepthSimMap(CudaHostMemoryHeap<float2, 2>& inout_depthSimMap_hmh, float depth = -1.f, float sim = 1.f);

/**
 * @brief Copy a depth/similarity map from host memory to 2 vectors.
 * @param[out] out_depthMap the output depth vector
 * @param[out] out_simMap the output similarity vector
 * @param[in] in_depthSimMap_hmh the depth/similarity map in host memory
 * @param[in] roi the 2d region of interest without any downscale apply
 * @param[in] downscale the depth/similarity map downscale factor
 */
void copyDepthSimMap(std::vector<float>& out_depthMap,
                     std::vector<float>& out_simMap,
                     const CudaHostMemoryHeap<float2, 2>& in_depthSimMap_hmh,
                     const ROI& roi, 
                     int downscale);
/**
 * @brief Copy a depth/similarity map from device memory to 2 vectors.
 * @param[out] out_depthMap the output depth vector
 * @param[out] out_simMap the output similarity vector
 * @param[in] in_depthSimMap_dmp the depth/similarity map in device memory
 * @param[in] roi the 2d region of interest without any downscale apply
 * @param[in] downscale the depth/similarity map downscale factor
 */
void copyDepthSimMap(std::vector<float>& out_depthMap, 
                     std::vector<float>& out_simMap, 
                     const CudaDeviceMemoryPitched<float2, 2>& in_depthSimMap_dmp,
                     const ROI& roi, 
                     int downscale);

/**
 * @brief Write a depth/similarity map on disk from host memory.
 * @param[in] rc the related R camera index
 * @param[in] mp the multi-view parameters
 * @param[in] tileParams tile workflow parameters
 * @param[in] roi the 2d region of interest without any downscale apply
 * @param[in] in_depthSimMap_hmh the depth/similarity map in host memory
 * @param[in] scale the depth/similarity map downscale factor
 * @param[in] step the depth/similarity map step factor
 * @param[in] customSuffix the filename custom suffix
 */
void writeDepthSimMap(int rc,
                      const mvsUtils::MultiViewParams& mp,
                      const mvsUtils::TileParams& tileParams,
                      const ROI& roi, 
                      const CudaHostMemoryHeap<float2, 2>& in_depthSimMap_hmh,
                      int scale,
                      int step,
                      const std::string& customSuffix = "");

/**
 * @brief Write a depth/similarity map on disk from device memory.
 * @param[in] rc the related R camera index
 * @param[in] mp the multi-view parameters
 * @param[in] tileParams tile workflow parameters
 * @param[in] roi the 2d region of interest without any downscale apply
 * @param[in] in_depthSimMap_dmp the depth/similarity map in device memory
 * @param[in] scale the depth/similarity map downscale factor
 * @param[in] step the depth/similarity map step factor
 * @param[in] customSuffix the filename custom suffix
 */
void writeDepthSimMap(int rc,
                      const mvsUtils::MultiViewParams& mp,
                      const mvsUtils::TileParams& tileParams,
                      const ROI& roi, 
                      const CudaDeviceMemoryPitched<float2, 2>& in_depthSimMap_dmp,
                      int scale,
                      int step,
                      const std::string& customSuffix = "");

/**
 * @brief Write a depth/similarity map on disk from a tile list in host memory.
 * @param[in] rc the related R camera index
 * @param[in] mp the multi-view parameters
 * @param[in] tileParams tile workflow parameters
 * @param[in] tileRoiList the 2d region of interest of each tile
 * @param[in] in_depthSimMapTiles_hmh the depth/similarity map tile list in host memory
 * @param[in] scale the depth/similarity map downscale factor
 * @param[in] step the depth/similarity map step factor
 * @param[in] customSuffix the filename custom suffix
 */
void writeDepthSimMapFromTileList(int rc,
                                  const mvsUtils::MultiViewParams& mp,
                                  const mvsUtils::TileParams& tileParams,
                                  const std::vector<ROI>& tileRoiList,
                                  const std::vector<CudaHostMemoryHeap<float2, 2>>& in_depthSimMapTiles_hmh,
                                  int scale,
                                  int step,
                                  const std::string& customSuffix = "");

/**
 * @brief Merge depth/similarity map tiles on disk.
 * @param[in] rc the related R camera index
 * @param[in] mp the multi-view parameters
 * @param[in] scale the depth/similarity map downscale factor
 * @param[in] step the depth/similarity map step factor
 * @param[in] customSuffix the filename custom suffix
 */
void mergeDepthSimMapTiles(int rc,
                           const mvsUtils::MultiViewParams& mp,
                           int scale,
                           int step,
                           const std::string& customSuffix = "");

/**
 * @brief Build and write a debug OBJ file with all tiles areas
 * @param[in] rc the related R camera index
 * @param[in] mp the multi-view parameters
 * @param[in] tileRoiList tile region-of-interest list
 * @param[in] tileMinMaxDepthsList tile min/max depth list
 */
void exportDepthSimMapTilePatternObj(int rc,
                                     const mvsUtils::MultiViewParams& mp,
                                     const std::vector<ROI>& tileRoiList,
                                     const std::vector<std::pair<float, float>>& tileMinMaxDepthsList);

} // namespace depthMap
} // namespace aliceVision

