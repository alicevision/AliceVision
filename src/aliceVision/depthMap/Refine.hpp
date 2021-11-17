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
    const int _rc;
    StaticVector<int> _tCams;
    DepthSimMap _depthSimMap; // refined, fused and optimized depth map

    void upscaleSgmDepthSimMap(const DepthSimMap& sgmDepthSimMap, DepthSimMap& out_depthSimMapUpscaled) const;
    void filterMaskedPixels(DepthSimMap& out_depthSimMap);
    void refineDepthSimMapPerTc(int tc, DepthSimMap& depthSimMap) const;
    void refineAndFuseDepthSimMap(const DepthSimMap& depthSimMapToRefine, DepthSimMap& out_depthSimMapRefinedFused) const;
    void optimizeDepthSimMap(const DepthSimMap& depthSimMapToRefine, const DepthSimMap& depthSimMapRefinedFused, DepthSimMap& out_depthSimMapOptimized) const;
};

} // namespace depthMap
} // namespace aliceVision
