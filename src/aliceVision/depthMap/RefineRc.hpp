// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/mvsData/StaticVector.hpp>
#include <aliceVision/depthMap/SemiGlobalMatchingRc.hpp>

namespace aliceVision {
namespace depthMap {

class RefineRc : public SemiGlobalMatchingRc
{
public:
    RefineRc(int rc, int scale, int step, SemiGlobalMatchingParams* sp);
    ~RefineRc();

    void preloadSgmTcams_async();

    bool refinerc(bool checkIfExists = true);

    void writeDepthMap();

private:
    StaticVector<int> _refineTCams;
    float _refineSigma;
    float _refineGammaC;
    float _refineGammaP;
    int _refineWsh;
    int _refineNSamplesHalf;
    int _refineNiters;
    int _nbDepthsToRefine;
    bool _userTcOrPixSize;

    DepthSimMap* _depthSimMapOpt = nullptr;

    DepthSimMap* getDepthPixSizeMapFromSGM();
    DepthSimMap* refineAndFuseDepthSimMapCUDA(DepthSimMap* depthPixSizeMapVis);
    DepthSimMap* optimizeDepthSimMapCUDA(DepthSimMap* depthPixSizeMapVis, DepthSimMap* depthSimMapPhoto);
};

void estimateAndRefineDepthMaps(mvsUtils::MultiViewParams* mp, const std::vector<int>& cams, int nbGPUs);
void estimateAndRefineDepthMaps(int cudaDeviceNo, mvsUtils::MultiViewParams* mp, const std::vector<int>& cams);

void computeNormalMaps(int CUDADeviceNo, mvsUtils::MultiViewParams* mp, const StaticVector<int>& cams);
void computeNormalMaps(mvsUtils::MultiViewParams* mp, const StaticVector<int>& cams);

} // namespace depthMap
} // namespace aliceVision
