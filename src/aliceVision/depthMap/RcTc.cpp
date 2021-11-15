// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "RcTc.hpp"
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/mvsData/StaticVector.hpp>
#include <aliceVision/mvsUtils/common.hpp>

namespace aliceVision {
namespace depthMap {

RcTc::RcTc(mvsUtils::MultiViewParams& _mp, PlaneSweepingCuda& _cps)
    : cps(_cps)
    , mp(_mp)
{}

#if 0
void RcTc::smoothDepthMap(DepthSimMap* depthSimMap, int rc, int wsh, float gammaC, float gammaP)
{
    long t1 = clock();

    StaticVector<float>* depthMap = depthSimMap->getDepthMapStep1();
    cps.smoothDepthMap(depthMap, rc, depthSimMap->scale, gammaC, gammaP, wsh);

    for(int i = 0; i < depthSimMap->w * depthSimMap->h; i++)
    {
        int x = (i % depthSimMap->w) * depthSimMap->step;
        int y = (i / depthSimMap->w) * depthSimMap->step;
        depthSimMap->dsm[i].depth = (*depthMap)[y * depthSimMap->w * depthSimMap->step + x];
    }

    if(mp.verbose)
        mvsUtils::printfElapsedTime(t1, "smoothDepth11");

    delete depthMap;
}
#endif

#if 0
void RcTc::filterDepthMap(DepthSimMap* depthSimMap, int rc, int wsh, float gammaC)
{
    long t1 = clock();

    float minCostThr = 25.0f;

    StaticVector<float>* depthMap = depthSimMap->getDepthMapStep1();
    cps->filterDepthMap(depthMap, rc, depthSimMap->scale, gammaC, minCostThr, wsh);

    for(int i = 0; i < depthSimMap->w * depthSimMap->h; i++)
    {
        int x = (i % depthSimMap->w) * depthSimMap->step;
        int y = (i / depthSimMap->w) * depthSimMap->step;
        depthSimMap->dsm[i].depth = (*depthMap)[y * depthSimMap->w * depthSimMap->step + x];
    }

    if(mp.verbose)
        mvsUtils::printfElapsedTime(t1, "smoothDepth11");

    delete depthMap;
}
#endif

} // namespace depthMap
} // namespace aliceVision
