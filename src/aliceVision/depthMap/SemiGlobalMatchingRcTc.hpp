// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/mvsData/StaticVector.hpp>
#include <aliceVision/mvsData/Voxel.hpp>
#include <aliceVision/depthMap/SemiGlobalMatchingParams.hpp>

namespace aliceVision {
namespace depthMap {

class SemiGlobalMatchingRcTc
{
public:
    SemiGlobalMatchingRcTc(const std::vector<float>& _rcTcDepths, int _rc, int _tc, int _scale, int _step, SemiGlobalMatchingParams* _sp,
                StaticVectorBool* _rcSilhoueteMap = NULL);
    ~SemiGlobalMatchingRcTc(void);

    StaticVector<unsigned char>* computeDepthSimMapVolume(float& volumeMBinGPUMem, int wsh, float gammaC, float gammaP);

private:
    StaticVector<Voxel>* getPixels();
    const SemiGlobalMatchingParams* const sp;

    const int rc;

    int tc;
    const std::vector<float> rcTcDepths;
    const int _scale;
    const int _step;
    const int _w;
    const int _h;
    float epipShift;
    // int w, h;
    StaticVectorBool* rcSilhoueteMap;
};

} // namespace depthMap
} // namespace aliceVision
