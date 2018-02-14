// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/structures/Pixel.hpp>
#include <aliceVision/structures/StaticVector.hpp>
#include <aliceVision/depthMap/SemiGlobalMatchingParams.hpp>

class SemiGlobalMatchingRc
{
public:
    SemiGlobalMatchingRc(bool doComputeDepthsAndResetTCams, int _rc, int _scale, int _step, SemiGlobalMatchingParams* _sp);
    ~SemiGlobalMatchingRc(void);

    bool sgmrc(bool checkIfExists = true);
    StaticVector<int>* tcams;

protected:

    float getMinTcStepAtDepth(float depth, float minDepth, float maxDepth,
                              StaticVector<StaticVector<float>*>* alldepths);
    float getMeanTcStepAtDepth(float depth, float minDepth, float maxDepth,
                               StaticVector<StaticVector<float>*>* alldepths);
    StaticVector<float>* getTcSeedsRcPlaneDists(int rc, StaticVector<int>* tcams);
    bool selectBestDepthsRange(int nDepthsThr, StaticVector<float>* rcSeedsDistsAsc);
    bool selectBestDepthsRange(int nDepthsThr, StaticVector<StaticVector<float>*>* alldepths);
    StaticVector<StaticVector<float>*>* computeAllDepthsAndResetTCams();
    void computeDepthsTcamsLimits(StaticVector<StaticVector<float>*>* alldepths);
    void computeDepths(float minDepth, float maxDepth, StaticVector<StaticVector<float>*>* alldepths);
    void computeDepthsAndResetTCams();

    StaticVector<float>* getSubDepthsForTCam(int tcamid);

    SemiGlobalMatchingParams* sp;

    int rc, scale, step;
    int wsh;
    float gammaC, gammaP;
    StaticVector<float>* depths;
    StaticVector<Pixel>* depthsTcamsLimits;
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

void computeDepthMapsPSSGM(MultiViewParams* mp, PreMatchCams* pc, const StaticVector<int>& cams);
void computeDepthMapsPSSGM(int CUDADeviceNo, MultiViewParams* mp, PreMatchCams* pc, const StaticVector<int>& cams);
