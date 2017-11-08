// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/CUDAInterfaces/cuda_plane_sweeping.hpp>
#include <aliceVision/output3D/mv_output3D.hpp>
#include "ps_depthSimMap.hpp"

class ps_rctc
{
public:
    multiviewParams* mp;
    mv_output3D* o3d;
    cuda_plane_sweeping* cps;
    bool verbose;

    ps_rctc(multiviewParams* _mp, cuda_plane_sweeping* _cps);
    ~ps_rctc(void);

    void refineRcTcDepthSimMap(bool userTcOrPixSize, ps_depthSimMap* depthSimMap, int rc, int tc, int ndepthsToRefine,
                               int wsh, float gammaC, float gammaP, float epipShift);

    void smoothDepthMap(ps_depthSimMap* depthSimMap, int rc, int wsh, float gammaC, float gammaP);
    void filterDepthMap(ps_depthSimMap* depthSimMap, int rc, int wsh, float gammaC);
    void computeRotCSRcTcEpip(point3d& p, point3d& n, point3d& x, point3d& y, int rc, int tc);

};
