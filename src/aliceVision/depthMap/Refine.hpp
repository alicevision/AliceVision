// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/mvsData/ROI.hpp>
#include <aliceVision/mvsUtils/MultiViewParams.hpp>
#include <aliceVision/mvsUtils/TileParams.hpp>
#include <aliceVision/depthMap/RefineParams.hpp>
#include <aliceVision/depthMap/cuda/host/memory.hpp>
#include <aliceVision/depthMap/cuda/planeSweeping/similarity.hpp>

#include <vector>
#include <string>

namespace aliceVision {
namespace depthMap {

/**
 * @brief Depth Map Estimation Refine
 */
class Refine
{
public:

    /**
     * @brief Refine constructor.
     * @param[in] mp the multi-view parameters
     * @param[in] tileParams tile workflow parameters
     * @param[in] refineParams the Refine parameters
     * @param[in] stream the stream for gpu execution
     */
    Refine(const mvsUtils::MultiViewParams& mp,
           const mvsUtils::TileParams& tileParams,   
           const RefineParams& refineParams, 
           cudaStream_t stream);

    // no default constructor
    Refine() = delete;

    // default destructor
    ~Refine() = default;

    inline const CudaDeviceMemoryPitched<float2, 2>& getDeviceDepthSimMap() const { return _optimizedDepthSimMap_dmp; }

    /**
     * @brief Get memory consumpyion in device memory.
     * @return device memory consumpyion (in MB)
     */
    double getDeviceMemoryConsumption() const;
    double getDeviceMemoryConsumptionUnpadded() const;

    /**
     * @brief Refine for a single R camera the Semi-Global Matching depth/sim map.
     * @param[in] rc the R camera index
     * @param[in] tCams the T cameras indexes
     * @param[in] in_sgmDepthSimMap_dmp the SGM result depth/sim map in device memory
     * @param[in] roi the 2d region of interest of the R image without any downscale apply
     */
    void refineRc(int rc, 
                  const std::vector<int>& in_tCams,
                  const CudaDeviceMemoryPitched<float2, 2>& in_sgmDepthSimMap_dmp,
                  const ROI& roi);

private:

    // private methods

    /**
     * @brief Refine and fuse the given depth/sim map using volume strategy.
     * @param[in] rc the R camera index
     * @param[in] tcams the T cameras indexes
     * @param[in] roi the 2d region of interest of the R image without any downscale apply
     */
    void refineAndFuseDepthSimMap(int rc, const std::vector<int>& tCams, const ROI& roi);

    /**
     * @brief Optimize the refined depth/sim maps.
     * @param[in] rc the R camera index
     * @param[in] roi the 2d region of interest of the R image without any downscale apply
     */
    void optimizeDepthSimMap(int rc, const ROI& roi);

    /**
     * @brief Export volume cross alembic file and 9 points csv file.
     * @param[in] rc the R camera index
     * @param[in] name the export filename
     * @param[in] roi the 2d region of interest of the R image without any downscale apply
     */
    void exportVolumeInformation(int rc, const std::string& name, const ROI& roi) const;

    // private members

    const mvsUtils::MultiViewParams& _mp;                          // Multi-view parameters
    const mvsUtils::TileParams& _tileParams;                       // tile workflow parameters
    const RefineParams& _refineParams;                             // Refine parameters

    // private members in device memory

    CudaDeviceMemoryPitched<float2, 2> _sgmDepthPixSizeMap_dmp;    // rc upscaled SGM depth/pixSize map
    CudaDeviceMemoryPitched<float2, 2> _refinedDepthSimMap_dmp;    // rc refined and fused depth/sim map
    CudaDeviceMemoryPitched<float2, 2> _optimizedDepthSimMap_dmp;  // rc optimized depth/sim map
    CudaDeviceMemoryPitched<TSimRefine, 3> _volumeRefineSim_dmp;   // rc refine similarity volume
    CudaDeviceMemoryPitched<float, 2> _optTmpDepthMap_dmp;         // for optimization: temporary depth map buffer
    CudaDeviceMemoryPitched<float, 2> _optImgVariance_dmp;         // for optimization: image variance buffer
    cudaStream_t _stream;                                          // stream for gpu execution
};

} // namespace depthMap
} // namespace aliceVision
