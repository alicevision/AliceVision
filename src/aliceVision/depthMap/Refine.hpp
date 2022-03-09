// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/mvsData/StaticVector.hpp>
#include <aliceVision/mvsData/ROI.hpp>
#include <aliceVision/mvsUtils/ImagesCache.hpp>
#include <aliceVision/mvsUtils/MultiViewParams.hpp>
#include <aliceVision/mvsUtils/TileParams.hpp>
#include <aliceVision/depthMap/DepthSimMap.hpp>
#include <aliceVision/depthMap/cuda/planeSweeping/similarity.hpp>

#include <cuda_runtime.h>

namespace aliceVision {
namespace depthMap {

struct RefineParams;

template <class Type, unsigned Dim>
class CudaDeviceMemoryPitched;

/**
 * @brief Depth Map Estimation Refine
 */
class Refine
{
public:

    /**
     * @brief Refine constructor.
     * @param[in] rc the R camera index
     * @param[in] ic the image cache
     * @param[in] mp the multi-view parameters
     * @param[in] tileParams tile workflow parameters
     * @param[in] refineParams the Refine parameters
     * @param[in] roi the 2d region of interest of the R image without any downscale apply
     * @param[in] stream the stream for gpu execution
     */
    Refine(int rc,
           mvsUtils::ImagesCache<ImageRGBAf>& ic,
           const mvsUtils::MultiViewParams& mp,
           const mvsUtils::TileParams& tileParams,   
           const RefineParams& refineParams, 
           const ROI& roi,
           cudaStream_t stream);

    // default destructor
    ~Refine() = default;

    /**
     * @brief Refine for a single R camera the Semi-Global Matching depth/sim map.
     */
    void refineRc(const DepthSimMap& sgmDepthSimMap);

    inline const StaticVector<int>& getTCams() const { return _tCams; }

    /**
     * @brief Get the depth/sim map of the Refine result.
     * @return the depth/sim map of the Refine result
     */
    inline const DepthSimMap& getDepthSimMap() const { return _depthSimMap; }

private:

    // private methods

    /**
     * @brief Upscale the given SGM depth/sim map.
     * @param[in] sgmDepthSimMap the given SGM depth/sim map
     * @param[in,out] out_depthSimMapUpscaled the given output depth/sim map
     * @note Dimensions of the given output depth/sim map are used to compute the scale factor.
     */
    void upscaleSgmDepthSimMap(const DepthSimMap& sgmDepthSimMap, DepthSimMap& out_depthSimMapUpscaled) const;

    /**
     * @brief Filter masked pixels (alpha < 0.1) of the given depth/sim map.
     * @param[in,out] inout_depthSimMap the given depth/sim map
     */
    void filterMaskedPixels(DepthSimMap& inout_depthSimMap);

    /**
     * @brief Refine and fuse the given depth/sim map.
     * @param[in] depthSimMapSgmUpscale_dmp the given upscaled SGM depth sim/map in device memory
     * @param[out] out_depthSimMapRefinedFused_dmp the given output refined and fused depth/sim map in device memory
     */
    void refineAndFuseDepthSimMap(const CudaDeviceMemoryPitched<float2, 2>& depthSimMapSgmUpscale_dmp, CudaDeviceMemoryPitched<float2, 2>& out_depthSimMapRefinedFused_dmp) const;

    /**
     * @brief Refine and fuse the given depth/sim map using volume strategy.
     * @param[in] depthSimMapSgmUpscale_dmp the given upscaled SGM depth sim/map in device memory
     * @param[out] out_depthSimMapRefinedFused_dmp the given output refined and fused depth/sim map in device memory
     */
    void refineAndFuseDepthSimMapVolume(const CudaDeviceMemoryPitched<float2, 2>& depthSimMapSgmUpscale_dmp, CudaDeviceMemoryPitched<float2, 2>& out_depthSimMapRefinedFused_dmp) const;

    /**
     * @brief Optimize the given depth/sim maps.
     * @param[in] depthSimMapSgmUpscale_dmp the given upscaled SGM depth/sim map in device memory
     * @param[in] depthSimMapRefinedFused_dmp the given refined and fused depth/sim map in device memory
     * @param[out] out_depthSimMapOptimized_dmp the given output optimized depth/sim map in device memory
     */
    void optimizeDepthSimMap(const CudaDeviceMemoryPitched<float2, 2>& depthSimMapSgmUpscale_dmp,
                             const CudaDeviceMemoryPitched<float2, 2>& depthSimMapRefinedFused_dmp,
                             CudaDeviceMemoryPitched<float2, 2>& out_depthSimMapOptimized_dmp) const;

    /**
     * @brief Export volume cross alembic file and 9 points csv file.
     * @param[in] in_volSim_dmp the given similarity volume in device memory
     * @param[in] depthSimMapSgmUpscale_dmp the given upscaled SGM depth/sim map in device memory
     * @param[in] name the export filename
     */
    void exportVolumeInformation(const CudaDeviceMemoryPitched<TSimRefine, 3>& in_volSim_dmp,
                                 const CudaDeviceMemoryPitched<float2, 2>& in_depthSimMapSgmUpscale_dmp,
                                 const std::string& name) const;

    // private members

    const int _rc;                            // related R camera index
    const RefineParams& _refineParams;        // Refine parameters
    const mvsUtils::MultiViewParams& _mp;     // Multi-view parameters
    const mvsUtils::TileParams& _tileParams;  // tile workflow parameters
    mvsUtils::ImagesCache<ImageRGBAf>& _ic;   // Image cache
    StaticVector<int> _tCams;                 // T camera indexes, computed in the constructor
    DepthSimMap _depthSimMap;                 // refined, fused and optimized depth map
    cudaStream_t _stream;                     // stream for gpu execution
};

} // namespace depthMap
} // namespace aliceVision
