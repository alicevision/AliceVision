// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "SemiGlobalMatchingParams.hpp"
#include <aliceVision/mvsData/Pixel.hpp>
#include <aliceVision/mvsData/Point2d.hpp>
#include <aliceVision/mvsData/Point3d.hpp>
#include <aliceVision/mvsUtils/common.hpp>
#include <aliceVision/mvsUtils/fileIO.hpp>

#include <boost/filesystem.hpp>

namespace aliceVision {
namespace depthMap {

namespace bfs = boost::filesystem;

SemiGlobalMatchingParams::SemiGlobalMatchingParams(mvsUtils::MultiViewParams& _mp, PlaneSweepingCuda& _cps)
    : cps( _cps )
    , mp(_mp)
{
}

SemiGlobalMatchingParams::~SemiGlobalMatchingParams()
{
}

void SemiGlobalMatchingParams::getDepthSimMapFromBestIdVal(DepthSimMap& out_depthSimMap, int w, int h, StaticVector<IdValue>& volumeBestIdVal,
                                                           int scale, int step, int rc, int zborder,
                                                           const StaticVector<float>& planesDepths)
{
    long tall = clock();

    int volDimX = w;
    int volDimY = h;

    assert(out_depthSimMap._dsm.size() == volDimX * volDimY);

#pragma omp parallel for
    for(int y = 0; y < volDimY; y++)
    {
        for(int x = 0; x < volDimX; x++)
        {
            Pixel pix = Pixel(x * step, y * step);
            Pixel pixScale1 = Pixel(pix.x * scale, pix.y * scale);
            float sim = volumeBestIdVal[y * volDimX + x].value;
            int fpdepthId = volumeBestIdVal[y * volDimX + x].id;

            DepthSim& out_depthSim = out_depthSimMap._dsm[y * volDimX + x];

            if((fpdepthId >= zborder) && (fpdepthId < planesDepths.size() - zborder))
            {
                float fpPlaneDepth = planesDepths[fpdepthId];
                Point3d planen = (mp.iRArr[rc] * Point3d(0.0f, 0.0f, 1.0f)).normalize();
                Point3d planep = mp.CArr[rc] + planen * fpPlaneDepth;
                Point3d v = (mp.iCamArr[rc] * Point2d((float)(pixScale1.x), (float)(pixScale1.y))).normalize();
                Point3d p = linePlaneIntersect(mp.CArr[rc], v, planep, planen);
                float depth = (mp.CArr[rc] - p).size();

                // printf("fpdepthId %i, fpPlaneDepth %f, depth %f, x %i y
                // %i\n",fpdepthId,fpPlaneDepth,depth,pixScale1.x,pixScale1.y);

                out_depthSim.depth = depth;
                out_depthSim.sim = sim;
            }
            else
            {
                out_depthSim.depth = -1.0f;
                out_depthSim.sim = 1.0f;
            }
        }
    }

    mvsUtils::printfElapsedTime(tall, "getDepthSimMapFromBestIdVal");
}

} // namespace depthMap
} // namespace aliceVision
