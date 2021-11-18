// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/mvsUtils/MultiViewParams.hpp>
#include <aliceVision/mvsData/StaticVector.hpp>
#include <aliceVision/depthMap/DepthSimMap.hpp>

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
     * @brief Refine the given depth/sim map with the given T camera.
     * @param[in] tc the given T camera index
     * @param[int,out] depthSimMap the given output refined depth/sim map
     */
    void refineDepthSimMapPerTc(int tc, DepthSimMap& depthSimMap) const;

    /**
     * @brief Refine and fuse the given depth/sim map.
     * @param[in] depthSimMapSgmUpscale the given upscaled SGM depth sim/map
     * @param[out] out_depthSimMapRefinedFused the given output refined and fused depth/sim map
     */
    void refineAndFuseDepthSimMap(const DepthSimMap& depthSimMapSgmUpscale, DepthSimMap& out_depthSimMapRefinedFused) const;

    /**
     * @brief Optimize the given depth/sim maps.
     * @param[in] depthSimMapSgmUpscale the given upscaled SGM depth/sim map
     * @param[in] depthSimMapRefinedFused the given refined and fused depth/sim map
     * @param[out] out_depthSimMapOptimized the given output optimized depth/sim map
     */
    void optimizeDepthSimMap(const DepthSimMap& depthSimMapSgmUpscale, const DepthSimMap& depthSimMapRefinedFused, DepthSimMap& out_depthSimMapOptimized) const;
};

} // namespace depthMap
} // namespace aliceVision
