// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "SemiGlobalMatchingRcTc.hpp"
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/mvsUtils/common.hpp>
#include <aliceVision/depthMap/cuda/tcinfo.hpp>
#include <algorithm>

namespace aliceVision {
namespace depthMap {


SemiGlobalMatchingRcTc::SemiGlobalMatchingRcTc(
            const std::vector<float>& rcDepths,
            const std::vector<Pixel>&  rcTcDepthRanges,
            int rc,
            const StaticVector<int>& tc,
            int scale,
            int step,
            SemiGlobalMatchingParams& sp)
    : _sp( sp )
    , _rc( rc )
    , _tc( tc )
    , _scale( scale )
    , _step( step )
    , _w( sp.mp.getWidth(rc) / (scale * step) )
    , _h( sp.mp.getHeight(rc) / (scale * step) )
    , _rcDepths( rcDepths )
    , _rcTcDepthRanges( rcTcDepthRanges )
{
    ALICEVISION_LOG_DEBUG("Create SemiGlobalMatchingRcTc with " << rcTcDepthRanges.size() << " T cameras.");
}

SemiGlobalMatchingRcTc::~SemiGlobalMatchingRcTc()
{
    //
}

struct MinOffX
{
    bool operator()( const Pixel& l, const Pixel& r ) const
    {
        return ( l.x < r.x );
    }
};

struct MinOffXplusY
{
    bool operator()( const Pixel& l, const Pixel& r ) const
    {
        return ( l.x+l.y < r.x+r.y );
    }
};

void SemiGlobalMatchingRcTc::computeDepthSimMapVolume(
        CudaDeviceMemoryPitched<TSim, 3>& volumeBestSim,
        CudaDeviceMemoryPitched<TSim, 3>& volumeSecBestSim,
        int wsh,
        float gammaC,
        float gammaP)
{
    auto startTime = std::chrono::high_resolution_clock::now();

    const int volStepXY = _step;
    const int volDimX   = _w;
    const int volDimY   = _h;

    {
        const int startingDepth = std::min_element( _rcTcDepthRanges.begin(),
                                                    _rcTcDepthRanges.end(),
                                                    MinOffX())->x;
        auto depth_it = std::max_element( _rcTcDepthRanges.begin(),
                                          _rcTcDepthRanges.end(),
                                          MinOffXplusY());
        const int stoppingDepth = depth_it->x + depth_it->y;
    
        assert(startingDepth == 0);
        assert(_rcDepths.size() == stoppingDepth);
    }
    ALICEVISION_LOG_DEBUG("RC depths: [" << _rcDepths[0] << "-" << _rcDepths[_rcDepths.size() - 1] << "], "
                                         << _rcDepths.size() << " depth planes.");

    std::vector<OneTC> tcs;
    for(size_t j = 0; j < _rcTcDepthRanges.size(); ++j)
    {
        tcs.emplace_back(_tc[j], _rcTcDepthRanges[j].x, _rcTcDepthRanges[j].y);
        ALICEVISION_LOG_DEBUG(" RC: " << _rc << ", TC: " << _tc[j] << ", "
                              << _rcTcDepthRanges[j].y << " depth planes, "
                              << "depth range=[" << _rcDepths[_rcTcDepthRanges[j].x] << "-" << _rcDepths[_rcTcDepthRanges[j].x + _rcTcDepthRanges[j].y - 1] << "], "
                              << "range index=[" << _rcTcDepthRanges[j].x << "-" << _rcTcDepthRanges[j].x + _rcTcDepthRanges[j].y << "]"
                              );
    }

    _sp.cps.sweepPixelsToVolume( volumeBestSim,
                                 volumeSecBestSim,
                                 volDimX, volDimY,
                                 volStepXY,
                                 tcs,
                                 _rcDepths,
                                 _rc,
                                 wsh, gammaC, gammaP, _scale);

    ALICEVISION_LOG_INFO("==== computeDepthSimMapVolume done in : " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - startTime).count() << "ms.");
}

} // namespace depthMap
} // namespace aliceVision
