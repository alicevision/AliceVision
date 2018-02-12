// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/depthMap/DepthSimMap.hpp>
#include <aliceVision/depthMap/RcTc.hpp>
#include <aliceVision/depthMap/cuda/PlaneSweepingCuda.hpp>
#include <aliceVision/common/ImagesCache.hpp>
#include <aliceVision/common/PreMatchCams.hpp>

class SemiGlobalMatchingParams
{
public:
    multiviewParams* mp;
    mv_prematch_cams* pc;
    RcTc* prt;
    PlaneSweepingCuda* cps;
    mv_images_cache* ic;
    bool visualizeDepthMaps;
    bool visualizePartialDepthMaps;
    bool doSmooth;
    // int   s_wsh;
    // float s_gammaC;
    // float s_gammaP;
    // int   wsh;
    // float gammaC;
    // float gammaP;
    bool doRefine;
    int ndepthsToRefine;
    unsigned char P1;
    unsigned char P2;
    unsigned char P3;
    int maxDepthsToStore;
    int maxDepthsToSweep;
    int rcTcDepthsHalfLimit;
    int rcDepthsCompStep;
    bool refineUseTcOrPixSize;
    bool useSeedsToCompDepthsToSweep;
    float seedsRangePercentile;
    float seedsRangeInflate;
    bool saveDepthsToSweepToTxtForVis;
    int modalsMapDistLimit;
    int minNumOfConsistentCams;
    int minObjectThickness;
    float maxTcRcPixSizeInVoxRatio;
    int nSGGCIters;
    bool doSGMoptimizeVolume;
    bool doRefineRc;
    std::string SGMoutDirName;
    std::string SGMtmpDirName;
    bool useSilhouetteMaskCodedByColor;
    rgb silhouetteMaskColor;

    SemiGlobalMatchingParams(multiviewParams* _mp, mv_prematch_cams* _pc, PlaneSweepingCuda* _cps);
    ~SemiGlobalMatchingParams(void);

    DepthSimMap* getDepthSimMapFromBestIdVal(int w, int h, staticVector<idValue>* volumeBestIdVal, int scale,
                                                int step, int rc, int zborder, staticVector<float>* planesDepths);

    std::string getREFINE_photo_depthMapFileName(int cam, int scale, int step);
    std::string getREFINE_photo_simMapFileName(int cam, int scale, int step);
    std::string getREFINE_opt_depthMapFileName(int cam, int scale, int step);
    std::string getREFINE_opt_simMapFileName(int cam, int scale, int step);

    std::string getSGMTmpDir();
    std::string getSGM_idDepthMapFileName(int cam, int scale, int step);
    std::string getSGM_depthMapFileName(int cam, int scale, int step);
    std::string getSGM_simMapFileName(int cam, int scale, int step);
    std::string getSGM_tcamsFileName(int cam);
    std::string getSGM_depthsFileName(int cam);

};
