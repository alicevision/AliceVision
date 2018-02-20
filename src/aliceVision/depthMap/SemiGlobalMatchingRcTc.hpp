// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/structures/StaticVector.hpp>
#include <aliceVision/structures/Voxel.hpp>
#include <aliceVision/depthMap/SemiGlobalMatchingParams.hpp>

namespace aliceVision {

class SemiGlobalMatchingRcTc
{
public:
    SemiGlobalMatchingRcTc(StaticVector<float>* _rcTcDepths, int _rc, int _tc, int _scale, int _step, SemiGlobalMatchingParams* _sp,
                StaticVectorBool* _rcSilhoueteMap = NULL);
    ~SemiGlobalMatchingRcTc(void);

    StaticVector<unsigned char>* computeDepthSimMapVolume(float& volumeMBinGPUMem, int wsh, float gammaC, float gammaP);

private:
    StaticVector<Voxel>* getPixels();

    SemiGlobalMatchingParams* sp;

    int rc, tc, scale, step;
    StaticVector<float>* rcTcDepths;
    float epipShift;
    int w, h;
    StaticVectorBool* rcSilhoueteMap;
};

} // namespace aliceVision
