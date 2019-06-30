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
    exportIntermediateResults = mp.userParams.get<bool>("depthMap.intermediateResults", exportIntermediateResults);
    doSmooth = mp.userParams.get<bool>("semiGlobalMatching.smooth", doSmooth);
    doRefine = mp.userParams.get<bool>("semiGlobalMatching.doRefine", doRefine);
    refineUseTcOrPixSize = mp.userParams.get<bool>("semiGlobalMatching.refineUseTcOrPixSize", refineUseTcOrPixSize);

    ndepthsToRefine = mp.userParams.get<int>("semiGlobalMatching.ndepthsToRefine", ndepthsToRefine);

    P1 = mp.userParams.get<float>("semiGlobalMatching.P1", P1);
    P2 = mp.userParams.get<float>("semiGlobalMatching.P2", P2); // direct P2 was 125.0, now it's only the P2 weighting

    stepZ = mp.userParams.get<int>("semiGlobalMatching.stepZ", stepZ);
    maxDepthsToStore = mp.userParams.get<int>("semiGlobalMatching.maxDepthsToStore", maxDepthsToStore);
    maxDepthsToSweep = mp.userParams.get<int>("semiGlobalMatching.maxDepthsToSweep", maxDepthsToSweep);
    rcTcDepthsHalfLimit = mp.userParams.get<int>("semiGlobalMatching.rcTcDepthsHalfLimit", rcTcDepthsHalfLimit);

    rcDepthsCompStep = mp.userParams.get<int>("semiGlobalMatching.rcDepthsCompStep", rcDepthsCompStep);

    useSeedsToCompDepthsToSweep =
        mp.userParams.get<bool>("semiGlobalMatching.useSeedsToCompDepthsToSweep", useSeedsToCompDepthsToSweep);
    seedsRangePercentile = (float)mp.userParams.get<double>("semiGlobalMatching.seedsRangePercentile", seedsRangePercentile);
    seedsRangeInflate = (float)mp.userParams.get<double>("semiGlobalMatching.seedsRangeInflate", seedsRangeInflate);

    saveDepthsToSweepToTxtForVis =
        mp.userParams.get<bool>("semiGlobalMatching.saveDepthsToSweepToTxtForVis", saveDepthsToSweepToTxtForVis);

    doSGMoptimizeVolume = mp.userParams.get<bool>("semiGlobalMatching.doSGMoptimizeVolume", doSGMoptimizeVolume);
    doRefineFuse = mp.userParams.get<bool>("semiGlobalMatching.doRefineFuse", doRefineFuse);
    doRefineOpt = mp.userParams.get<bool>("semiGlobalMatching.doRefineOpt", doRefineOpt);

    modalsMapDistLimit = mp.userParams.get<int>("semiGlobalMatching.modalsMapDistLimit", modalsMapDistLimit);
    minNumOfConsistentCams = mp.userParams.get<int>("semiGlobalMatching.minNumOfConsistentCams", minNumOfConsistentCams);
    maxTcRcPixSizeInVoxRatio =
        (float)mp.userParams.get<double>("semiGlobalMatching.maxTcRcPixSizeInVoxRatio", maxTcRcPixSizeInVoxRatio);
    nSGGCIters = mp.userParams.get<int>("semiGlobalMatching.nSGGCIters", nSGGCIters);

    SGMoutDirName = mp.userParams.get<std::string>("semiGlobalMatching.outDirName", SGMoutDirName);
    SGMtmpDirName = mp.userParams.get<std::string>("semiGlobalMatching.tmpDirName", SGMtmpDirName);

    useSilhouetteMaskCodedByColor = mp.userParams.get<bool>("global.useSilhouetteMaskCodedByColor", useSilhouetteMaskCodedByColor);
    silhouetteMaskColor.r = mp.userParams.get<int>("global.silhouetteMaskColorR", silhouetteMaskColor.r);
    silhouetteMaskColor.g = mp.userParams.get<int>("global.silhouetteMaskColorG", silhouetteMaskColor.g);
    silhouetteMaskColor.b = mp.userParams.get<int>("global.silhouetteMaskColorB", silhouetteMaskColor.b);
}

SemiGlobalMatchingParams::~SemiGlobalMatchingParams()
{
}

std::string SemiGlobalMatchingParams::getREFINE_photo_depthMapFileName(IndexT viewId, int scale, int step)
{
    return mp.getDepthMapsFolder() + std::to_string(viewId) + "_depthMap_scale" + mvsUtils::num2str(scale) + "_step" + mvsUtils::num2str(step) + "_refinePhoto.exr";
}

std::string SemiGlobalMatchingParams::getREFINE_photo_simMapFileName(IndexT viewId, int scale, int step)
{
    return mp.getDepthMapsFolder() + std::to_string(viewId) + "_simMap_scale" + mvsUtils::num2str(scale) + "_step" + mvsUtils::num2str(step) + "_refinePhoto.exr";
}

std::string SemiGlobalMatchingParams::getREFINE_opt_depthMapFileName(IndexT viewId, int scale, int step)
{
    return mp.getDepthMapsFolder() + std::to_string(viewId) + "_depthMap_scale" + mvsUtils::num2str(scale) + "_step" + mvsUtils::num2str(step) + "_refineOpt.exr";
}

std::string SemiGlobalMatchingParams::getREFINE_opt_simMapFileName(IndexT viewId, int scale, int step)
{
    return mp.getDepthMapsFolder() + std::to_string(viewId) + "_simMap_scale" + mvsUtils::num2str(scale) + "_step" + mvsUtils::num2str(step) + "_refineOpt.exr";
}

std::string SemiGlobalMatchingParams::getSGMTmpDir()
{
    return mp.getDepthMapsFolder() + SGMoutDirName + "/" + SGMtmpDirName + "/";
}

std::string SemiGlobalMatchingParams::getSGM_depthMapFileName(IndexT viewId, int scale, int step)
{
    return mp.getDepthMapsFolder() + std::to_string(viewId) + "_depthMap_scale" + mvsUtils::num2str(scale) + "_step" + mvsUtils::num2str(step) + "_SGM.bin";
}

std::string SemiGlobalMatchingParams::getSGM_simMapFileName(IndexT viewId, int scale, int step)
{
    return mp.getDepthMapsFolder() + std::to_string(viewId) + "_simMap_scale" + mvsUtils::num2str(scale) + "_step" + mvsUtils::num2str(step) + "_SGM.bin";
}

std::string SemiGlobalMatchingParams::getSGM_idDepthMapFileName(IndexT viewId, int scale, int step)
{
    return mp.getDepthMapsFolder() + std::to_string(viewId) + "_idDepthMap_scale" + mvsUtils::num2str(scale) + "_step" + mvsUtils::num2str(step) + "_SGM.png";
}

std::string SemiGlobalMatchingParams::getSGM_tcamsFileName(IndexT viewId)
{
    return mp.getDepthMapsFolder() + std::to_string(viewId) + "_tcams.bin";
}

std::string SemiGlobalMatchingParams::getSGM_depthsFileName(IndexT viewId)
{
    return mp.getDepthMapsFolder() + std::to_string(viewId) + "_depths.bin";
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
