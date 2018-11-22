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
    SemiGlobalMatchingRcTc( const std::vector<int>& index_set,
                            const std::vector<std::vector<float> >& _rcTcDepths,
                            int _rc,
                            const StaticVector<int>& _tc,
                            int _scale,
                            int _step,
                            int zDimsAtATime,
                            SemiGlobalMatchingParams* _sp,
                            StaticVectorBool* _rcSilhoueteMap = NULL );
    ~SemiGlobalMatchingRcTc(void);

    void computeDepthSimMapVolume( std::vector<StaticVector<unsigned char> >& volume,
                                   std::vector<CudaDeviceMemoryPitched<float, 3>*>& volume_tmp_on_gpu,
                                   // float& volumeMBinGPUMem,
                                   int wsh,
                                   float gammaC,
                                   float gammaP );

private:
    const std::vector<int> _index_set;
    const SemiGlobalMatchingParams* const sp;

    const int rc;
    const StaticVector<int>& tc;
    const int _scale;
    const int _step;
    const int _w;
    const int _h;
    const int _zDimsAtATime;
    const std::vector<std::vector<float> >& rcTcDepths;
    float epipShift;
    // int w, h;
    StaticVectorBool* rcSilhoueteMap;
};

} // namespace depthMap
} // namespace aliceVision
