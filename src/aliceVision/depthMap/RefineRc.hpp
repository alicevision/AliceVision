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
class RefineRc
{
public:
    RefineRc(const RefineParams& refineParams, const mvsUtils::MultiViewParams& mp, PlaneSweepingCuda& cps, int rc);
    ~RefineRc();

    bool refineRc(const DepthSimMap& depthSimMapToRefine);

    const StaticVector<int>& getTCams() const { return _tCams; }
    const DepthSimMap& getDepthSimMap() const { return _depthSimMap; }

private:

    const RefineParams& _refineParams;
    const mvsUtils::MultiViewParams& _mp;
    PlaneSweepingCuda& _cps;
    const int _rc;
    StaticVector<int> _tCams;
    DepthSimMap _depthSimMap;

    std::string getPhotoDepthMapFileName(IndexT viewId, int scale, int step) const;
    std::string getPhotoSimMapFileName(IndexT viewId, int scale, int step) const;
    std::string getOptDepthMapFileName(IndexT viewId, int scale, int step) const;
    std::string getOptSimMapFileName(IndexT viewId, int scale, int step) const;

    void getDepthPixSizeMapFromSGM(const DepthSimMap& sgmDepthSimMap, DepthSimMap& out_depthSimMapScale1Step1);
    void filterMaskedPixels(DepthSimMap& out_depthSimMap);
    void refineRcTcDepthSimMap(DepthSimMap& depthSimMap, int tc);
    void refineAndFuseDepthSimMapCUDA(DepthSimMap& out_depthSimMapFused, const DepthSimMap& depthPixSizeMapVis);
    void optimizeDepthSimMapCUDA(DepthSimMap& out_depthSimMapOptimized, const DepthSimMap& depthPixSizeMapVis, const DepthSimMap& depthSimMapPhoto);
};

} // namespace depthMap
} // namespace aliceVision
