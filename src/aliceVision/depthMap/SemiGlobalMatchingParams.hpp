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
    bool exportIntermediateResults = false;
    bool doSmooth = true;
    bool doRefine = true;
    int ndepthsToRefine = 15;
    float P1 = 10.0f;
    float P2 = 20.0f;
    int stepZ = -1;
    int maxDepthsToStore = 3000;
    int maxDepthsToSweep = 1500;
    int rcTcDepthsHalfLimit = 2048;
    int rcDepthsCompStep = 6;
    bool refineUseTcOrPixSize = true;
    bool useSeedsToCompDepthsToSweep = true;
    float seedsRangePercentile = 0.999;
    float seedsRangeInflate = 0.2;
    bool saveDepthsToSweepToTxtForVis = false;
    int modalsMapDistLimit = 2;
    int minNumOfConsistentCams = 2;
    float maxTcRcPixSizeInVoxRatio = 2.0f;
    int nSGGCIters = 0;
    bool doSGMoptimizeVolume = true;
    bool doRefineFuse = true;
    bool doRefineOpt = true;
    std::string SGMoutDirName = "SGM";
    std::string SGMtmpDirName = "_tmp";
    bool useSilhouetteMaskCodedByColor = false;
    rgb silhouetteMaskColor{0, 0, 0};

    SemiGlobalMatchingParams(mvsUtils::MultiViewParams& mp, PlaneSweepingCuda& _cps);
    ~SemiGlobalMatchingParams();

    void getDepthSimMapFromBestIdVal(DepthSimMap& out_depthSimMap, int w, int h,
                                     StaticVector<IdValue>& volumeBestIdVal, int scale,
                                     int step, int rc, int zborder, const StaticVector<float>& planesDepths);

    std::string getREFINE_photo_depthMapFileName(IndexT viewId, int scale, int step);
    std::string getREFINE_photo_simMapFileName(IndexT viewId, int scale, int step);
    std::string getREFINE_opt_depthMapFileName(IndexT viewId, int scale, int step);
    std::string getREFINE_opt_simMapFileName(IndexT viewId, int scale, int step);

    std::string getSGMTmpDir();
    std::string getSGM_idDepthMapFileName(IndexT viewId, int scale, int step);
    std::string getSGM_depthMapFileName(IndexT viewId, int scale, int step);
    std::string getSGM_simMapFileName(IndexT viewId, int scale, int step);
    std::string getSGM_tcamsFileName(IndexT viewId);
    std::string getSGM_depthsFileName(IndexT viewId);

};

} // namespace depthMap
} // namespace aliceVision
