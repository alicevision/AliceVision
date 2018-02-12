// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/depthMap/DepthSimMap.hpp>
#include <aliceVision/depthMap/cuda/PlaneSweepingCuda.hpp>

class RcTc
{
public:
    multiviewParams* mp;
    PlaneSweepingCuda* cps;
    bool verbose;

    RcTc(multiviewParams* _mp, PlaneSweepingCuda* _cps);
    ~RcTc(void);

    void refineRcTcDepthSimMap(bool useTcOrRcPixSize, DepthSimMap* depthSimMap, int rc, int tc, int ndepthsToRefine,
                               int wsh, float gammaC, float gammaP, float epipShift);

    void smoothDepthMap(DepthSimMap* depthSimMap, int rc, int wsh, float gammaC, float gammaP);
    void filterDepthMap(DepthSimMap* depthSimMap, int rc, int wsh, float gammaC);
    void computeRotCSRcTcEpip(point3d& p, point3d& n, point3d& x, point3d& y, int rc, int tc);

};
