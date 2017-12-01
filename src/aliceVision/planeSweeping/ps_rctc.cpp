// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "ps_rctc.hpp"


ps_rctc::ps_rctc(multiviewParams* _mp, cuda_plane_sweeping* _cps)
{
    cps = _cps;
    mp = _mp;
    o3d = new mv_output3D(mp);
    verbose = mp->verbose;
}

ps_rctc::~ps_rctc()
{
    delete o3d;
}

void ps_rctc::refineRcTcDepthSimMap(bool userTcOrPixSize, ps_depthSimMap* depthSimMap, int rc, int tc,
                                    int ndepthsToRefine, int wsh, float gammaC, float gammaP, float epipShift)
{
    int scale = depthSimMap->scale;
    int w = mp->mip->getWidth(rc) / scale;
    int h = mp->mip->getHeight(rc) / scale;

    if(verbose)
        printf("w %i, h %i\n", w, h);

    long t1 = clock();

    int nParts = 4;
    int wPart = w / nParts;
    for(int p = 0; p < nParts; p++)
    {
        int xFrom = p * wPart;
        int wPartAct = std::min(wPart, w - xFrom);
        staticVector<float>* depthMap = depthSimMap->getDepthMapStep1XPart(xFrom, wPartAct);
        staticVector<float>* simMap = depthSimMap->getSimMapStep1XPart(xFrom, wPartAct);

        cps->refineRcTcDepthMap(userTcOrPixSize, ndepthsToRefine, simMap, depthMap, rc, tc, scale, wsh, gammaC, gammaP,
                                epipShift, xFrom, wPartAct);

        for(int yp = 0; yp < h; yp++)
        {
            for(int xp = xFrom; xp < xFrom + wPartAct; xp++)
            {
                float depth = (*depthMap)[yp * wPartAct + (xp - xFrom)];
                float sim = (*simMap)[yp * wPartAct + (xp - xFrom)];
                float oldSim =
                    (*depthSimMap->dsm)[(yp / depthSimMap->step) * depthSimMap->w + (xp / depthSimMap->step)].sim;
                if((depth > 0.0f) && (sim < oldSim))
                {
                    (*depthSimMap->dsm)[(yp / depthSimMap->step) * depthSimMap->w + (xp / depthSimMap->step)] =
                        DepthSim(depth, sim);
                }
            }
        }

        if(verbose)
            printfElapsedTime(t1, "refineRcTcDepthSimMap");

        delete depthMap;
        delete simMap;
    }
}

void ps_rctc::smoothDepthMap(ps_depthSimMap* depthSimMap, int rc, int wsh, float gammaC, float gammaP)
{
    long t1 = clock();

    staticVector<float>* depthMap = depthSimMap->getDepthMapStep1();
    cps->smoothDepthMap(depthMap, rc, depthSimMap->scale, gammaC, gammaP, wsh);

    for(int i = 0; i < depthSimMap->w * depthSimMap->h; i++)
    {
        int x = (i % depthSimMap->w) * depthSimMap->step;
        int y = (i / depthSimMap->w) * depthSimMap->step;
        (*depthSimMap->dsm)[i].depth = (*depthMap)[y * depthSimMap->w * depthSimMap->step + x];
    }

    if(verbose)
        printfElapsedTime(t1, "smoothDepth11");

    delete depthMap;
}

void ps_rctc::filterDepthMap(ps_depthSimMap* depthSimMap, int rc, int wsh, float gammaC)
{
    long t1 = clock();

    float minCostThr = 25.0f;

    staticVector<float>* depthMap = depthSimMap->getDepthMapStep1();
    cps->filterDepthMap(depthMap, rc, depthSimMap->scale, gammaC, minCostThr, wsh);

    for(int i = 0; i < depthSimMap->w * depthSimMap->h; i++)
    {
        int x = (i % depthSimMap->w) * depthSimMap->step;
        int y = (i / depthSimMap->w) * depthSimMap->step;
        (*depthSimMap->dsm)[i].depth = (*depthMap)[y * depthSimMap->w * depthSimMap->step + x];
    }

    if(verbose)
        printfElapsedTime(t1, "smoothDepth11");

    delete depthMap;
}

void ps_rctc::computeRotCSRcTcEpip(point3d& p, point3d& n, point3d& x, point3d& y, int rc, int tc)
{
    n = (mp->CArr[rc] - p).normalize();
    x = (mp->CArr[tc] - mp->CArr[rc]).normalize();
    y = cross(n, x).normalize();
    x = cross(y, n).normalize();
}
