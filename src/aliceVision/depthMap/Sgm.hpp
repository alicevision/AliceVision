// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/mvsData/StaticVector.hpp>
#include <aliceVision/mvsData/Pixel.hpp>
#include <aliceVision/mvsData/ROI.hpp>
#include <aliceVision/mvsUtils/ImagesCache.hpp>
#include <aliceVision/mvsUtils/MultiViewParams.hpp>
#include <aliceVision/mvsUtils/TileParams.hpp>
#include <aliceVision/depthMap/DepthSimMap.hpp>
#include <aliceVision/depthMap/SgmDepthList.hpp>
#include <aliceVision/depthMap/cuda/planeSweeping/similarity.hpp>

#include <cuda_runtime.h>

namespace aliceVision {
namespace depthMap {

struct SgmParams;

template <class Type, unsigned Dim>
class CudaDeviceMemoryPitched;

/**
 * @brief Depth Map Estimation Semi-Global Matching
 */
class Sgm
{
public:

    /**
     * @brief Sgm constructor.
     * @param[in] rc the R camera index
     * @param[in] ic the image cache 
     * @param[in] mp the multi-view parameters
     * @param[in] tileParams tile workflow parameters
     * @param[in] sgmParams the Semi Global Matching parameters
     * @param[in] roi the 2d region of interest of the R image without any downscale apply
     * @param[in] stream the stream for gpu execution
     */
    Sgm(int rc, 
        mvsUtils::ImagesCache<ImageRGBAf>& ic, 
        const mvsUtils::MultiViewParams& mp, 
        const mvsUtils::TileParams& tileParams, 
        const SgmParams& sgmParams, 
        const ROI& roi, 
        cudaStream_t stream);

    // default destructor
    ~Sgm() = default;

    /**
     * @brief Compute for a single R camera the Semi-Global Matching depth/sim map.
     */
    void sgmRc();

    /**
     * @brief Return true if no depths found for the R camera.
     * @return true if no depths found for the R camera
     */
    inline bool empty() const { return _sgmDepthList.getDepths().empty(); }

    /**
     * @brief Get the depth/sim map of the Semi-Global Matching result.
     * @return the depth/sim map of the Semi-Global Matching result
     */
    inline const DepthSimMap& getDepthSimMap() const { return _depthSimMap; }

private:

    // private methods

    /**
     * @brief Compute for each RcTc the best / second best similarity volumes.
     * @param[out] out_volBestSim_dmp the best similarity volume in device memory
     * @param[out] out_volSecBestSim_dmp the second best similarity volume in device memory
     */
    void computeSimilarityVolumes(CudaDeviceMemoryPitched<TSim, 3>& out_volBestSim_dmp, CudaDeviceMemoryPitched<TSim, 3>& out_volSecBestSim_dmp) const;

    /**
     * @brief Optimize the given similarity volume.
     * @note  Filter on the 3D volume to weight voxels based on their neighborhood strongness.
     *        So it downweights local minimums that are not supported by their neighborhood.
     * @param[out] out_volSimOptimized_dmp the output optimized similarity volume in device memory
     * @param[in] in_volSim_dmp the input similarity volume in device memory
     */
    void optimizeSimilarityVolume(CudaDeviceMemoryPitched<TSim, 3>& out_volSimOptimized_dmp, const CudaDeviceMemoryPitched<TSim, 3>& in_volSim_dmp) const;

    /**
     * @brief Retrieve the best depths in the given similarity volume.
     * @note  For each pixel, choose the voxel with the minimal similarity value.
     * @param[out] out_bestDepthSimMap the output best depth/sim map
     * @param[in] in_volSim_dmp the input similarity volume in device memory
     */
    void retrieveBestDepth(DepthSimMap& out_bestDepthSimMap, const CudaDeviceMemoryPitched<TSim, 3>& in_volSim_dmp) const;

    /**
     * @brief Export volume alembic files and 9 points csv file.
     * @param[in] in_volSim_dmp the given similarity volume in device memory
     * @param[in] name the export filename
     */
    void exportVolumeInformation(const CudaDeviceMemoryPitched<TSim, 3>& in_volSim_dmp, const std::string& name) const;


    // private members

    const int _rc;                            // related R camera index
    const SgmParams& _sgmParams;              // Semi Global Matching parameters
    const mvsUtils::TileParams& _tileParams;  // tile workflow parameters
    const mvsUtils::MultiViewParams& _mp;     // Multi-view parameters
    mvsUtils::ImagesCache<ImageRGBAf>& _ic;   // Image cache
    SgmDepthList _sgmDepthList;               // R camera Semi Global Matching depth list
    DepthSimMap _depthSimMap;                 // depth/sim map of the Semi Global Matching result
    cudaStream_t _stream;                     // stream for gpu execution
};

} // namespace depthMap
} // namespace aliceVision
