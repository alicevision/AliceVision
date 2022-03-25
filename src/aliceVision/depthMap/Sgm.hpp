// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/mvsData/ROI.hpp>
#include <aliceVision/mvsUtils/MultiViewParams.hpp>
#include <aliceVision/mvsUtils/TileParams.hpp>
#include <aliceVision/depthMap/SgmParams.hpp>
#include <aliceVision/depthMap/SgmDepthList.hpp>
#include <aliceVision/depthMap/cuda/host/memory.hpp>
#include <aliceVision/depthMap/cuda/planeSweeping/similarity.hpp>

#include <vector>
#include <string>

namespace aliceVision {
namespace depthMap {

/**
 * @brief Depth Map Estimation Semi-Global Matching
 */
class Sgm
{
public:

    /**
     * @brief Sgm constructor.
     * @param[in] mp the multi-view parameters
     * @param[in] tileParams tile workflow parameters
     * @param[in] sgmParams the Semi Global Matching parameters
     * @param[in] stream the stream for gpu execution
     */
    Sgm(const mvsUtils::MultiViewParams& mp, 
        const mvsUtils::TileParams& tileParams, 
        const SgmParams& sgmParams, 
        cudaStream_t stream);

    // no default constructor
    Sgm() = delete;

    // default destructor
    ~Sgm() = default;

    inline const CudaDeviceMemoryPitched<float2, 2>& getDeviceDepthSimMap() const { return _depthSimMap_dmp; }

    /**
     * @brief Get memory consumpyion in device memory.
     * @return device memory consumpyion (in MB)
     */
    double getDeviceMemoryConsumption() const;
    double getDeviceMemoryConsumptionUnpadded() const;

    /**
     * @brief Compute for a single R camera the Semi-Global Matching depth/sim map.
     * @param[in] rc the R camera index
     * @param[in] in_sgmDepthList the SGM depth list
     * @param[in] roi the 2d region of interest of the R image without any downscale apply
     */
    void sgmRc(int rc, const SgmDepthList& in_sgmDepthList, const ROI& roi);

private:

    // private methods

    /**
     * @brief Compute for each RcTc the best / second best similarity volumes.
     * @param[in] rc the R camera index
     * @param[in] in_sgmDepthList the SGM depth list
     * @param[in] roi the 2d region of interest of the R image without any downscale apply
     */
    void computeSimilarityVolumes(int rc, const SgmDepthList& in_sgmDepthList, const ROI& roi);

    /**
     * @brief Optimize the given similarity volume.
     * @note  Filter on the 3D volume to weight voxels based on their neighborhood strongness.
     *        So it downweights local minimums that are not supported by their neighborhood.
     * @param[in] rc the R camera index
     * @param[in] in_sgmDepthList the SGM depth list
     * @param[in] roi the 2d region of interest of the R image without any downscale apply
     */
    void optimizeSimilarityVolume(int rc, const SgmDepthList& in_sgmDepthList, const ROI& roi);

    /**
     * @brief Retrieve the best depths in the given similarity volume.
     * @note  For each pixel, choose the voxel with the minimal similarity value.
     * @param[in] rc the R camera index
     * @param[in] in_sgmDepthList the SGM depth list
     * @param[in] roi the 2d region of interest of the R image without any downscale apply
     */
    void retrieveBestDepth(int rc, const SgmDepthList& in_sgmDepthList, const ROI& roi);

    /**
     * @brief Export volume alembic files and 9 points csv file.
     * @param[in] rc the R camera index
     * @param[in] in_volume_dmp the input volume
     * @param[in] in_sgmDepthList the SGM depth list
     * @param[in] name the export filename
     * @param[in] roi the 2d region of interest of the R image without any downscale apply
     */
    void exportVolumeInformation(int rc, 
                                 const CudaDeviceMemoryPitched<TSim, 3>& in_volume_dmp,
                                 const SgmDepthList& in_sgmDepthList, 
                                 const std::string& name, 
                                 const ROI& roi) const;


    // private members 

    const mvsUtils::MultiViewParams& _mp;                      // Multi-view parameters
    const mvsUtils::TileParams& _tileParams;                   // tile workflow parameters
    const SgmParams& _sgmParams;                               // Semi Global Matching parameters

    // private members in device memory

    CudaHostMemoryHeap<float, 2> _depths_hmh;                  // rc depth data host memory
    CudaDeviceMemoryPitched<float, 2> _depths_dmp;             // rc depth data device memory
    CudaDeviceMemoryPitched<float2, 2> _depthSimMap_dmp;       // rc result depth/sim map
    CudaDeviceMemoryPitched<TSim, 3> _volumeBestSim_dmp;       // rc best similarity volume
    CudaDeviceMemoryPitched<TSim, 3> _volumeSecBestSim_dmp;    // rc second best similarity volume
    CudaDeviceMemoryPitched<TSimAcc, 2> _volumeSliceAccA_dmp;  // for optimization: volume accumulation slice A
    CudaDeviceMemoryPitched<TSimAcc, 2> _volumeSliceAccB_dmp;  // for optimization: volume accumulation slice B
    CudaDeviceMemoryPitched<TSimAcc, 2> _volumeAxisAcc_dmp;    // for optimization: volume accumulation axis
    cudaStream_t _stream;                                      // stream for gpu execution
};

} // namespace depthMap
} // namespace aliceVision
