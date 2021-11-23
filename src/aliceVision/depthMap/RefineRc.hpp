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
    RefineRc(int rc, int scale, int step, SemiGlobalMatchingParams& sp);
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

    DepthSimMap _depthSimMapOpt;

    void getDepthPixSizeMapFromSGM(DepthSimMap& out_depthSimMapScale1Step1);
    void filterMaskedPixels(DepthSimMap& out_depthSimMap, int rc);

    void refineAndFuseDepthSimMapCUDA(DepthSimMap& out_depthSimMapFused, const DepthSimMap& depthPixSizeMapVis);
    void optimizeDepthSimMapCUDA(DepthSimMap& out_depthSimMapOptimized, const DepthSimMap& depthPixSizeMapVis, const DepthSimMap& depthSimMapPhoto);
};

void estimateAndRefineDepthMaps(int cudaDeviceIndex, mvsUtils::MultiViewParams& mp, const std::vector<int>& cams);

void computeNormalMaps(int cudaDeviceIndex, mvsUtils::MultiViewParams& mp, const std::vector<int>& cams);

} // namespace depthMap
} // namespace aliceVision
