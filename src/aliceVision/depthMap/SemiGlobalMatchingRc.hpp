// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/depthMap/SemiGlobalMatchingParams.hpp>

class SemiGlobalMatchingRc
{
public:
    SemiGlobalMatchingRc(bool doComputeDepthsAndResetTCams, int _rc, int _scale, int _step, SemiGlobalMatchingParams* _sp);
    ~SemiGlobalMatchingRc(void);

    bool sgmrc(bool checkIfExists = true);
    staticVector<int>* tcams;

protected:

    float getMinTcStepAtDepth(float depth, float minDepth, float maxDepth,
                              staticVector<staticVector<float>*>* alldepths);
    float getMeanTcStepAtDepth(float depth, float minDepth, float maxDepth,
                               staticVector<staticVector<float>*>* alldepths);
    staticVector<float>* getTcSeedsRcPlaneDists(int rc, staticVector<int>* tcams);
    bool selectBestDepthsRange(int nDepthsThr, staticVector<float>* rcSeedsDistsAsc);
    bool selectBestDepthsRange(int nDepthsThr, staticVector<staticVector<float>*>* alldepths);
    staticVector<staticVector<float>*>* computeAllDepthsAndResetTCams();
    void computeDepthsTcamsLimits(staticVector<staticVector<float>*>* alldepths);
    void computeDepths(float minDepth, float maxDepth, staticVector<staticVector<float>*>* alldepths);
    void computeDepthsAndResetTCams();

    staticVector<float>* getSubDepthsForTCam(int tcamid);

    SemiGlobalMatchingParams* sp;

    int rc, scale, step;
    int wsh;
    float gammaC, gammaP;
    staticVector<float>* depths;
    staticVector<pixel>* depthsTcamsLimits;
    int w, h;

    std::string outDir;
    std::string tmpDir;
    std::string tcamsFileName;
    std::string depthsFileName;
    std::string depthsTcamsLimitsFileName;
    std::string SGM_depthMapFileName;
    std::string SGM_simMapFileName;
    std::string SGM_idDepthMapFileName;
};

void computeDepthMapsPSSGM(multiviewParams* mp, mv_prematch_cams* pc, const staticVector<int>& cams);
void computeDepthMapsPSSGM(int CUDADeviceNo, multiviewParams* mp, mv_prematch_cams* pc, const staticVector<int>& cams);
