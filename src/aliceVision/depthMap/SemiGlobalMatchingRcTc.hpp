// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/depthMap/SemiGlobalMatchingParams.hpp>

class SemiGlobalMatchingRcTc
{
public:
    SemiGlobalMatchingRcTc(staticVector<float>* _rcTcDepths, int _rc, int _tc, int _scale, int _step, SemiGlobalMatchingParams* _sp,
                staticVectorBool* _rcSilhoueteMap = NULL);
    ~SemiGlobalMatchingRcTc(void);

    staticVector<unsigned char>* computeDepthSimMapVolume(float& volumeMBinGPUMem, int wsh, float gammaC, float gammaP);

private:
    staticVector<voxel>* getPixels();

    SemiGlobalMatchingParams* sp;

    int rc, tc, scale, step;
    staticVector<float>* rcTcDepths;
    float epipShift;
    int w, h;
    staticVectorBool* rcSilhoueteMap;
};
