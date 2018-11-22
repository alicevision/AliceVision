// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/mvsData/Point3d.hpp>
#include <aliceVision/depthMap/DepthSimMap.hpp>
#include <aliceVision/depthMap/cuda/PlaneSweepingCuda.hpp>

namespace aliceVision {
namespace depthMap {

class RcTc
{
public:
    mvsUtils::MultiViewParams* mp;
    PlaneSweepingCuda&         cps;
    bool                       verbose;

    RcTc(mvsUtils::MultiViewParams* _mp, PlaneSweepingCuda& _cps);

    void refineRcTcDepthSimMap(bool useTcOrRcPixSize, DepthSimMap* depthSimMap, int rc, int tc, int ndepthsToRefine,
                               int wsh, float gammaC, float gammaP, float epipShift);

    // void smoothDepthMap(DepthSimMap* depthSimMap, int rc, int wsh, float gammaC, float gammaP);
    // void filterDepthMap(DepthSimMap* depthSimMap, int rc, int wsh, float gammaC);
};

} // namespace depthMap
} // namespace aliceVision
