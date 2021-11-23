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
    SemiGlobalMatchingRcTc( const std::vector<float>& rcDepths,
                            const std::vector<Pixel>&  rcTcDepthRanges,
                            int _rc,
                            const StaticVector<int>& _tc,
                            int _scale,
                            int _step,
                            SemiGlobalMatchingParams& sp);
    ~SemiGlobalMatchingRcTc();

    void computeDepthSimMapVolume( CudaDeviceMemoryPitched<TSim, 3>& volumeBestSim,
                                   CudaDeviceMemoryPitched<TSim, 3>& volumeSecBestSim,
                                   int wsh,
                                   float gammaC,
                                   float gammaP );

private:
    const SemiGlobalMatchingParams& _sp;

    const int _rc;
    const StaticVector<int>& _tc;
    const int _scale;
    const int _step;
    const int _w;
    const int _h;
    const std::vector<float>& _rcDepths;
    const std::vector<Pixel>& _rcTcDepthRanges;
};

} // namespace depthMap
} // namespace aliceVision
