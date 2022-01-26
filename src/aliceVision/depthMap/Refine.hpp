// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/mvsUtils/MultiViewParams.hpp>
#include <aliceVision/mvsData/StaticVector.hpp>
#include <aliceVision/depthMap/DepthSimMap.hpp>
#include <aliceVision/depthMap/cuda/memory.hpp>

namespace aliceVision {
namespace depthMap {

struct RefineParams;
class PlaneSweepingCuda;

/**
 * @brief Depth Map Estimation Refine
 */
class Refine
{
public:
    Refine(const RefineParams& refineParams, const mvsUtils::MultiViewParams& mp, PlaneSweepingCuda& cps, int rc);
    ~Refine();

    bool refineRc(const DepthSimMap& sgmDepthSimMap);

    const StaticVector<int>& getTCams() const { return _tCams; }
    const DepthSimMap& getDepthSimMap() const { return _depthSimMap; }

private:

    const RefineParams& _refineParams;
    const mvsUtils::MultiViewParams& _mp;
    PlaneSweepingCuda& _cps;

    const int _rc;            // refine R camera index
    StaticVector<int> _tCams; // refine T camera indexes, compute in the constructor
    DepthSimMap _depthSimMap; // refined, fused and optimized depth map

    /**
     * @brief Upscale the given SGM depth/sim map.
     * @param[in] sgmDepthSimMap the given SGM depth/sim map
     * @param[in,out] out_depthSimMapUpscaled the given output depth/sim map
     * @note Dimensions of the given output depth/sim map are used to compute the scale factor.
     */
    void upscaleSgmDepthSimMap(const DepthSimMap& sgmDepthSimMap, DepthSimMap& out_depthSimMapUpscaled) const;

    /**
     * @brief Filter masked pixels (alpha < 0.1) of the given depth/sim map.
     * @param[in,out] out_depthSimMap the given depth/sim map
     */
    void filterMaskedPixels(DepthSimMap& out_depthSimMap);

    /**
     * @brief Refine and fuse the given depth/sim map.
     * @param[in] depthSimMapSgmUpscale_dmp the given upscaled SGM depth sim/map in device memory
     * @param[out] out_depthSimMapRefinedFused_dmp the given output refined and fused depth/sim map in device memory
     */
    void refineAndFuseDepthSimMap(const CudaDeviceMemoryPitched<float2, 2>& depthSimMapSgmUpscale_dmp, CudaDeviceMemoryPitched<float2, 2>& out_depthSimMapRefinedFused_dmp) const;

    /**
     * @brief Optimize the given depth/sim maps.
     * @param[in] depthSimMapSgmUpscale_dmp the given upscaled SGM depth/sim map in device memory
     * @param[in] depthSimMapRefinedFused_dmp the given refined and fused depth/sim map in device memory
     * @param[out] out_depthSimMapOptimized_dmp the given output optimized depth/sim map in device memory
     */
    void optimizeDepthSimMap(const CudaDeviceMemoryPitched<float2, 2>& depthSimMapSgmUpscale_dmp,
                             const CudaDeviceMemoryPitched<float2, 2>& depthSimMapRefinedFused_dmp,
                             CudaDeviceMemoryPitched<float2, 2>& out_depthSimMapOptimized_dmp) const;
};

} // namespace depthMap
} // namespace aliceVision
