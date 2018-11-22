// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/mvsData/Pixel.hpp>
#include <aliceVision/mvsData/StaticVector.hpp>
#include <aliceVision/depthMap/SemiGlobalMatchingParams.hpp>

namespace aliceVision {
namespace depthMap {

class SemiGlobalMatchingRc
{
public:
    SemiGlobalMatchingRc(bool doComputeDepthsAndResetTCams, int _rc, int _scale, int _step, SemiGlobalMatchingParams* _sp);
    ~SemiGlobalMatchingRc(void);

    bool sgmrc(bool checkIfExists = true);

protected:
    StaticVector<int> tcams;

protected:

    float getMinTcStepAtDepth(float depth, float minDepth, float maxDepth,
                              StaticVector<StaticVector<float>*>* alldepths);
    StaticVector<float>* getTcSeedsRcPlaneDists(int rc, const StaticVector<int>& tcams);
    bool selectBestDepthsRange(int nDepthsThr, StaticVector<float>* rcSeedsDistsAsc);
    bool selectBestDepthsRange(int nDepthsThr, StaticVector<StaticVector<float>*>* alldepths);
    StaticVector<StaticVector<float>*>* computeAllDepthsAndResetTCams();
    void computeDepthsTcamsLimits(StaticVector<StaticVector<float>*>* alldepths);
    void computeDepths(float minDepth, float maxDepth, StaticVector<StaticVector<float>*>* alldepths);
    void computeDepthsAndResetTCams();

private:
    void getSubDepthsForTCam( int tcamid, std::vector<float>& subDepths );

protected:
    SemiGlobalMatchingParams* sp;

    const int rc;
    const int scale;
    const int step;
    int wsh;
    float gammaC, gammaP;
    StaticVector<float>* depths;
    StaticVector<Pixel> depthsTcamsLimits;
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

void computeDepthMapsPSSGM(mvsUtils::MultiViewParams* mp, mvsUtils::PreMatchCams* pc, const StaticVector<int>& cams);
void computeDepthMapsPSSGM(int CUDADeviceNo, mvsUtils::MultiViewParams* mp, mvsUtils::PreMatchCams* pc, const StaticVector<int>& cams);

} // namespace depthMap
} // namespace aliceVision
