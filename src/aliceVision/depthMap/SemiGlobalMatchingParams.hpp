// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/mvsData/Rgb.hpp>
#include <aliceVision/mvsData/StaticVector.hpp>
#include <aliceVision/mvsUtils/ImagesCache.hpp>
#include <aliceVision/depthMap/DepthSimMap.hpp>
#include <aliceVision/depthMap/RcTc.hpp>
#include <aliceVision/depthMap/cuda/PlaneSweepingCuda.hpp>

namespace aliceVision {
namespace depthMap {

class SemiGlobalMatchingParams
{
public:
    mvsUtils::MultiViewParams& mp;
    PlaneSweepingCuda& cps;

    SemiGlobalMatchingParams(mvsUtils::MultiViewParams& mp, PlaneSweepingCuda& _cps);
    ~SemiGlobalMatchingParams();

    void getDepthSimMapFromBestIdVal(DepthSimMap& out_depthSimMap, int w, int h,
                                     StaticVector<IdValue>& volumeBestIdVal, int scale,
                                     int step, int rc, int zborder, const StaticVector<float>& planesDepths);
};

} // namespace depthMap
} // namespace aliceVision
