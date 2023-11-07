// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/image/Image.hpp>
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
 * @brief Write a normal map (depth map estimation) on disk from device memory.
 * @param[in] rc the related R camera index
 * @param[in] mp the multi-view parameters
 * @param[in] tileParams tile workflow parameters
 * @param[in] roi the 2d region of interest without any downscale apply
 * @param[in] in_normalMap_dmp the normal map in device memory
 * @param[in] scale the map downscale factor
 * @param[in] step the map step factor
 * @param[in] name the export filename suffix
 */
void writeNormalMap(int rc,
                    const mvsUtils::MultiViewParams& mp,
                    const mvsUtils::TileParams& tileParams,
                    const ROI& roi,
                    const CudaDeviceMemoryPitched<float3, 2>& in_normalMap_dmp,
                    int scale,
                    int step,
                    const std::string& name = "");

/**
 * @brief Write a normal map (depth map filtering) on disk from device memory.
 * @param[in] rc the related R camera index
 * @param[in] mp the multi-view parameters
 * @param[in] tileParams tile workflow parameters
 * @param[in] roi the 2d region of interest without any downscale apply
 * @param[in] in_normalMap_dmp the normal map in device memory
 * @param[in] scale the map downscale factor
 * @param[in] step the map step factor
 * @param[in] name the export filename suffix
 */
void writeNormalMapFiltered(int rc,
                            const mvsUtils::MultiViewParams& mp,
                            const mvsUtils::TileParams& tileParams,
                            const ROI& roi,
                            const CudaDeviceMemoryPitched<float3, 2>& in_normalMap_dmp,
                            int scale = 1,
                            int step = 1,
                            const std::string& name = "");

/**
 * @brief Write a depth/thickness map on disk from device memory.
 * @param[in] rc the related R camera index
 * @param[in] mp the multi-view parameters
 * @param[in] tileParams tile workflow parameters
 * @param[in] roi the 2d region of interest without any downscale apply
 * @param[in] in_depthThicknessMap_dmp the depth/thickness map in device memory
 * @param[in] scale the depth/thickness map downscale factor
 * @param[in] step the depth/thickness map step factor
 * @param[in] name the export filename suffix
 */
void writeDepthThicknessMap(int rc,
                            const mvsUtils::MultiViewParams& mp,
                            const mvsUtils::TileParams& tileParams,
                            const ROI& roi,
                            const CudaDeviceMemoryPitched<float2, 2>& in_depthThicknessMap_dmp,
                            int scale,
                            int step,
                            const std::string& name = "");

/**
 * @brief Write a depth/pixSize map on disk from device memory.
 * @param[in] rc the related R camera index
 * @param[in] mp the multi-view parameters
 * @param[in] tileParams tile workflow parameters
 * @param[in] roi the 2d region of interest without any downscale apply
 * @param[in] in_depthPixSize_dmp the depth/pixSize map in device memory
 * @param[in] scale the depth/pixSize map downscale factor
 * @param[in] step the depth/pixSize map step factor
 * @param[in] name the export filename suffix
 */
void writeDepthPixSizeMap(int rc,
                          const mvsUtils::MultiViewParams& mp,
                          const mvsUtils::TileParams& tileParams,
                          const ROI& roi,
                          const CudaDeviceMemoryPitched<float2, 2>& in_depthPixSize_dmp,
                          int scale,
                          int step,
                          const std::string& name = "");

/**
 * @brief Write a depth/similarity map on disk from device memory.
 * @param[in] rc the related R camera index
 * @param[in] mp the multi-view parameters
 * @param[in] tileParams tile workflow parameters
 * @param[in] roi the 2d region of interest without any downscale apply
 * @param[in] in_depthSimMap_dmp the depth/similarity map in device memory
 * @param[in] scale the depth/similarity map downscale factor
 * @param[in] step the depth/similarity map step factor
 * @param[in] name the export filename suffix
 */
void writeDepthSimMap(int rc,
                      const mvsUtils::MultiViewParams& mp,
                      const mvsUtils::TileParams& tileParams,
                      const ROI& roi,
                      const CudaDeviceMemoryPitched<float2, 2>& in_depthSimMap_dmp,
                      int scale,
                      int step,
                      const std::string& name = "");

/**
 * @brief Write a depth/similarity map on disk from a tile list in host memory.
 * @param[in] rc the related R camera index
 * @param[in] mp the multi-view parameters
 * @param[in] tileParams tile workflow parameters
 * @param[in] tileRoiList the 2d region of interest of each tile
 * @param[in] in_depthSimMapTiles_hmh the depth/similarity map tile list in host memory
 * @param[in] scale the depth/similarity map downscale factor
 * @param[in] step the depth/similarity map step factor
 * @param[in] name the export filename suffix
 */
void writeDepthSimMapFromTileList(int rc,
                                  const mvsUtils::MultiViewParams& mp,
                                  const mvsUtils::TileParams& tileParams,
                                  const std::vector<ROI>& tileRoiList,
                                  const std::vector<CudaHostMemoryHeap<float2, 2>>& in_depthSimMapTiles_hmh,
                                  int scale,
                                  int step,
                                  const std::string& name = "");

/**
 * @brief Reset a depth/similarity map in host memory to the given default depth and similarity.
 * @param[in,out] inout_depthSimMap_hmh the depth/similarity map in host memory
 * @param[in] depth the depth reset value
 * @param[in] sim the sim reset value
 */
void resetDepthSimMap(CudaHostMemoryHeap<float2, 2>& inout_depthSimMap_hmh, float depth = -1.f, float sim = 1.f);

/**
 * @brief Merge normal map tiles on disk.
 * @param[in] rc the related R camera index
 * @param[in] mp the multi-view parameters
 * @param[in] scale the normal map downscale factor
 * @param[in] step the normal map step factor
 * @param[in] name the export filename suffix
 */
void mergeNormalMapTiles(int rc, const mvsUtils::MultiViewParams& mp, int scale, int step, const std::string& name = "");

/**
 * @brief Merge depth/thickness map tiles on disk.
 * @param[in] rc the related R camera index
 * @param[in] mp the multi-view parameters
 * @param[in] scale the depth/thickness map downscale factor
 * @param[in] step the depth/thickness map step factor
 * @param[in] name the export filename suffix
 */
void mergeDepthThicknessMapTiles(int rc, const mvsUtils::MultiViewParams& mp, int scale, int step, const std::string& name = "");

/**
 * @brief Merge depth/pixSize map tiles on disk.
 * @param[in] rc the related R camera index
 * @param[in] mp the multi-view parameters
 * @param[in] scale the depth/pixSize map downscale factor
 * @param[in] step the depth/pixSize map step factor
 * @param[in] name the export filename suffix
 */
void mergeDepthPixSizeMapTiles(int rc, const mvsUtils::MultiViewParams& mp, int scale, int step, const std::string& name = "");

/**
 * @brief Merge depth/similarity map tiles on disk.
 * @param[in] rc the related R camera index
 * @param[in] mp the multi-view parameters
 * @param[in] scale the depth/similarity map downscale factor
 * @param[in] step the depth/similarity map step factor
 * @param[in] name the export filename suffix
 */
void mergeDepthSimMapTiles(int rc, const mvsUtils::MultiViewParams& mp, int scale, int step, const std::string& name = "");

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

}  // namespace depthMap
}  // namespace aliceVision
