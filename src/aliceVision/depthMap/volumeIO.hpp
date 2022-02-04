// This file is part of the AliceVision project.
// Copyright (c) 2019 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/mvsData/StaticVector.hpp>
#include <aliceVision/mvsData/Point3d.hpp>
#include <aliceVision/mvsUtils/common.hpp>
#include <aliceVision/mvsUtils/fileIO.hpp>
#include <aliceVision/depthMap/SgmParams.hpp>
#include <aliceVision/depthMap/RefineParams.hpp>
#include <aliceVision/depthMap/DepthSimMap.hpp>
#include <aliceVision/depthMap/cuda/host/memory.hpp>
#include <aliceVision/depthMap/cuda/planeSweeping/similarity.hpp>

#include <string>

namespace aliceVision {
namespace depthMap {

/**
 * @brief Export 9 similarity values over the entire depth in a CSV file.
 * @param[in] volumeSim the similarity in host memory
 * @param[in] depths the depth list
 * @param[in] camIndex the R cam global index
 * @param[in] name the export name
 * @param[in] filepath the export filepath
 */
void exportSimilaritySamplesCSV(const CudaHostMemoryHeap<TSim, 3>& volumeSim, 
                                const StaticVector<float>& depths, 
                                int camIndex, 
                                const std::string& name, 
                                const std::string& filepath);

/**
 * @brief Export 9 similarity values over the entire depth in a CSV file.
 * @param[in] volumeSim the similarity in host memory
 * @param[in] camIndex the R cam global index
 * @param[in] name the export name
 * @param[in] filepath the export filepath
 */
void exportSimilaritySamplesCSV(const CudaHostMemoryHeap<TSimRefine, 3>& volumeSim, 
                                int camIndex, 
                                const std::string& name, 
                                const std::string& filepath);

/**
 * @brief Export the given similarity volume to an Alembic file.
 * @param[in] volumeSim the similarity in host memory
 * @param[in] depths the depth list
 * @param[in] mp the multi-view parameters
 * @param[in] camIndex the R cam global index
 * @param[in] sgmParams the Semi Global Matching parameters
 * @param[in] filepath the export filepath
 */
void exportSimilarityVolume(const CudaHostMemoryHeap<TSim, 3>& volumeSim, 
                            const StaticVector<float>& depths, 
                            const mvsUtils::MultiViewParams& mp, 
                            int camIndex, 
                            const SgmParams& sgmParams,
                            const std::string& filepath);

/**
 * @brief Export a cross of the given similarity volume to an Alembic file.
 * @param[in] volumeSim the similarity in host memory
 * @param[in] depths the depth list
 * @param[in] mp the multi-view parameters
 * @param[in] camIndex the R cam global index
 * @param[in] sgmParams the Semi Global Matching parameters
 * @param[in] filepath the export filepath
 */
void exportSimilarityVolumeCross(const CudaHostMemoryHeap<TSim, 3>& volumeSim, 
                                 const StaticVector<float>& depths, 
                                 const mvsUtils::MultiViewParams& mp, 
                                 int camIndex, 
                                 const SgmParams& sgmParams,
                                 const std::string& filepath);

/**
 * @brief Export a cross of the given similarity volume to an Alembic file.
 * @param[in] volumeSim the similarity in host memory
 * @param[in] depthSimMapSgmUpscale the upscaled SGM depth/sim map
 * @param[in] mp the multi-view parameters
 * @param[in] camIndex the R cam global index
 * @param[in] refineParams the Refine parameters
 * @param[in] filepath the export filepath
 */
void exportSimilarityVolumeCross(const CudaHostMemoryHeap<TSimRefine, 3>& volumeSim,
                                 const DepthSimMap& depthSimMapSgmUpscale,
                                 const mvsUtils::MultiViewParams& mp, 
                                 int camIndex,
                                 const RefineParams& refineParams, 
                                 const std::string& filepath);

/**
 * @brief Export the given similarity volume to an Alembic file.
 */
void exportColorVolume(const CudaHostMemoryHeap<float4, 3>& volumeSim, 
                       const std::vector<float>& depths, 
                       int startDepth, 
                       int nbDepths, 
                       const mvsUtils::MultiViewParams& mp, 
                       int camIndex, 
                       int scale, 
                       int step, 
                       const std::string& filepath);

} // namespace depthMap
} // namespace aliceVision
