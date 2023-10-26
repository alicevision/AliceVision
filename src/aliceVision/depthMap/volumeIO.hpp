// This file is part of the AliceVision project.
// Copyright (c) 2019 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/mvsData/ROI.hpp>
#include <aliceVision/mvsUtils/MultiViewParams.hpp>
#include <aliceVision/depthMap/SgmParams.hpp>
#include <aliceVision/depthMap/RefineParams.hpp>
#include <aliceVision/depthMap/cuda/host/memory.hpp>
#include <aliceVision/depthMap/cuda/planeSweeping/similarity.hpp>

#include <string>
#include <vector>

namespace aliceVision {
namespace depthMap {

/**
 * @brief Export 9 similarity values over the entire depth in a CSV file.
 * @param[in] in_volumeSim_hmh the similarity in host memory
 * @param[in] in_depths the SGM depth list
 * @param[in] name the export name
 * @param[in] sgmParams the Semi Global Matching parameters
 * @param[in] filepath the export filepath
 * @param[in] roi the 2d region of interest
 */
void exportSimilaritySamplesCSV(const CudaHostMemoryHeap<TSim, 3>& in_volumeSim_hmh,
                                const std::vector<float>& in_depths,
                                const std::string& name,
                                const SgmParams& sgmParams,
                                const std::string& filepath,
                                const ROI& roi);

/**
 * @brief Export 9 similarity values over the entire depth in a CSV file.
 * @param[in] in_volumeSim_hmh the similarity in host memory
 * @param[in] name the export name
 * @param[in] refineParams the Refine parameters
 * @param[in] filepath the export filepath
 * @param[in] roi the 2d region of interest
 */
void exportSimilaritySamplesCSV(const CudaHostMemoryHeap<TSimRefine, 3>& in_volumeSim_hmh,
                                const std::string& name,
                                const RefineParams& refineParams,
                                const std::string& filepath,
                                const ROI& roi);

/**
 * @brief Export the given similarity volume to an Alembic file.
 * @param[in] in_volumeSim_hmh the similarity in host memory
 * @param[in] in_depths the SGM depth list
 * @param[in] mp the multi-view parameters
 * @param[in] camIndex the R cam global index
 * @param[in] sgmParams the Semi Global Matching parameters
 * @param[in] filepath the export filepath
 * @param[in] roi the 2d region of interest
 */
void exportSimilarityVolume(const CudaHostMemoryHeap<TSim, 3>& in_volumeSim_hmh,
                            const std::vector<float>& in_depths,
                            const mvsUtils::MultiViewParams& mp,
                            int camIndex,
                            const SgmParams& sgmParams,
                            const std::string& filepath,
                            const ROI& roi);

/**
 * @brief Export a cross of the given similarity volume to an Alembic file.
 * @param[in] in_volumeSim_hmh the similarity in host memory
 * @param[in] in_depths the SGM depth list
 * @param[in] mp the multi-view parameters
 * @param[in] camIndex the R cam global index
 * @param[in] sgmParams the Semi Global Matching parameters
 * @param[in] filepath the export filepath
 * @param[in] roi the 2d region of interest
 */
void exportSimilarityVolumeCross(const CudaHostMemoryHeap<TSim, 3>& in_volumeSim_hmh,
                                 const std::vector<float>& in_depths,
                                 const mvsUtils::MultiViewParams& mp,
                                 int camIndex,
                                 const SgmParams& sgmParams,
                                 const std::string& filepath,
                                 const ROI& roi);

/**
 * @brief Export a cross of the given similarity volume to an Alembic file.
 * @param[in] in_volumeSim_hmh the similarity in host memory
 * @param[in] in_depthSimMapSgmUpscale_hmh the upscaled SGM depth/sim map
 * @param[in] mp the multi-view parameters
 * @param[in] camIndex the R cam global index
 * @param[in] refineParams the Refine parameters
 * @param[in] filepath the export filepath
 * @param[in] roi the 2d region of interest
 */
void exportSimilarityVolumeCross(const CudaHostMemoryHeap<TSimRefine, 3>& in_volumeSim_hmh,
                                 const CudaHostMemoryHeap<float2, 2>& in_depthSimMapSgmUpscale_hmh,
                                 const mvsUtils::MultiViewParams& mp,
                                 int camIndex,
                                 const RefineParams& refineParams,
                                 const std::string& filepath,
                                 const ROI& roi);

/**
 * @brief Export a topographic cut of the given similarity volume to an Alembic file.
 * @param[in] in_volumeSim_hmh the similarity in host memory
 * @param[in] in_depths the SGM depth list
 * @param[in] mp the multi-view parameters
 * @param[in] camIndex the R cam global index
 * @param[in] sgmParams the Semi Global Matching parameters
 * @param[in] filepath the export filepath
 * @param[in] roi the 2d region of interest
 */
void exportSimilarityVolumeTopographicCut(const CudaHostMemoryHeap<TSim, 3>& in_volumeSim_hmh,
                                          const std::vector<float>& in_depths,
                                          const mvsUtils::MultiViewParams& mp,
                                          int camIndex,
                                          const SgmParams& sgmParams,
                                          const std::string& filepath,
                                          const ROI& roi);

/**
 * @brief Export a topographic cut of the given similarity volume to an Alembic file.
 * @param[in] in_volumeSim_hmh the similarity in host memory
 * @param[in] in_depthSimMapSgmUpscale_hmh the upscaled SGM depth/sim map
 * @param[in] mp the multi-view parameters
 * @param[in] camIndex the R cam global index
 * @param[in] refineParams the Refine parameters
 * @param[in] filepath the export filepath
 * @param[in] roi the 2d region of interest
 */
void exportSimilarityVolumeTopographicCut(const CudaHostMemoryHeap<TSimRefine, 3>& in_volumeSim_hmh,
                                          const CudaHostMemoryHeap<float2, 2>& in_depthSimMapSgmUpscale_hmh,
                                          const mvsUtils::MultiViewParams& mp,
                                          int camIndex,
                                          const RefineParams& refineParams,
                                          const std::string& filepath,
                                          const ROI& roi);

/**
 * @brief Export the given similarity volume to an Alembic file.
 */
void exportColorVolume(const CudaHostMemoryHeap<float4, 3>& in_volumeSim_hmh,
                       const std::vector<float>& in_depths,
                       int startDepth,
                       int nbDepths,
                       const mvsUtils::MultiViewParams& mp,
                       int camIndex,
                       int scale,
                       int step,
                       const std::string& filepath,
                       const ROI& roi);

}  // namespace depthMap
}  // namespace aliceVision
