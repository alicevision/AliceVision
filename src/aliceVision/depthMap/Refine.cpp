// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Refine.hpp"

#include <aliceVision/alicevision_omp.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/Timer.hpp>
#include <aliceVision/gpu/gpu.hpp>

#include <aliceVision/depthMap/RefineParams.hpp>
#include <aliceVision/depthMap/cuda/PlaneSweepingCuda.hpp>

#include <aliceVision/mvsData/Point2d.hpp>
#include <aliceVision/mvsData/Point3d.hpp>
#include <aliceVision/mvsData/imageIO.hpp>

#include <aliceVision/mvsUtils/fileIO.hpp>
#include <aliceVision/mvsUtils/common.hpp>

#include <boost/filesystem.hpp>

namespace aliceVision {
namespace depthMap {

namespace bfs = boost::filesystem;

void copy(CudaDeviceMemoryPitched<float2, 2>& out_depthSimMap_dmp, const DepthSimMap& in_depthSimMap)
{
    const int width = in_depthSimMap.getWidth();
    const int height = in_depthSimMap.getHeight();

    CudaHostMemoryHeap<float2, 2> depthSimMap_hmh(CudaSize<2>(width, height));

    for(int y = 0; y < height; ++y)
    {
        for(int x = 0; x < width; ++x)
        {
            float2& depthSim_h = depthSimMap_hmh(x, y);
            const DepthSim& in_depthSim = in_depthSimMap.getDepthSim(x, y);
            depthSim_h.x = in_depthSim.depth;
            depthSim_h.y = in_depthSim.sim;
        }
    }

    assert(depthSimMap_hmh.getSize() == out_depthSimMap_dmp.getSize());

    out_depthSimMap_dmp.copyFrom(depthSimMap_hmh);
}

void copyDepth(CudaDeviceMemoryPitched<float2, 2>& out_depthSimMap_dmp, const CudaDeviceMemoryPitched<float2, 2>& in_depthSimMap_dmp, float defaultSim)
{
    const CudaSize<2>& depthSimMapSize = in_depthSimMap_dmp.getSize();

    CudaHostMemoryHeap<float2, 2> depthSimMap_hmh(depthSimMapSize);
    depthSimMap_hmh.copyFrom(in_depthSimMap_dmp);

    for(int y = 0; y < depthSimMapSize.y() ; ++y)
    {
        for(int x = 0; x < depthSimMapSize.x() ; ++x)
        {
            float2& depthSim_h = depthSimMap_hmh(x, y);
            depthSim_h.y = defaultSim;
        }
    }

    assert(in_depthSimMap_dmp.getSize() == out_depthSimMap_dmp.getSize());

    out_depthSimMap_dmp.copyFrom(depthSimMap_hmh);
}

void copy(DepthSimMap& out_depthSimMap, const CudaDeviceMemoryPitched<float2, 2>& in_depthSimMap_dmp)
{
    CudaHostMemoryHeap<float2, 2> depthSimMap_hmh(in_depthSimMap_dmp.getSize());
    depthSimMap_hmh.copyFrom(in_depthSimMap_dmp);

    const int width = in_depthSimMap_dmp.getSize().x();
    const int height = in_depthSimMap_dmp.getSize().y();

    assert(out_depthSimMap.getWidth() == width);
    assert(out_depthSimMap.getHeight() == height);

    for(int y = 0; y < height; ++y)
    {
        for(int x = 0; x < width; ++x)
        {
            const float2& depthSim_h = depthSimMap_hmh(x, y);
            DepthSim& out_depthSim = out_depthSimMap.getDepthSim(x, y);
            out_depthSim.depth = depthSim_h.x;
            out_depthSim.sim = depthSim_h.y;
        }
    }
}

Refine::Refine(const RefineParams& refineParams, const mvsUtils::MultiViewParams& mp, PlaneSweepingCuda& cps, int rc)
    : _rc(rc)
    , _mp(mp)
    , _cps(cps)
    , _refineParams(refineParams)
    , _depthSimMap(_rc, _mp, _refineParams.scale, 1)
{
    _tCams = _mp.findNearestCamsFromLandmarks(_rc, _refineParams.maxTCams);
}

Refine::~Refine()
{}

void Refine::upscaleSgmDepthSimMap(const DepthSimMap& sgmDepthSimMap, DepthSimMap& out_depthSimMapUpscaled) const
{
    const int w = _mp.getWidth(_rc);
    const int h = _mp.getHeight(_rc);

    out_depthSimMapUpscaled.initFromSmaller(sgmDepthSimMap);

    // set sim (y) to pixsize
    for(int y = 0; y < h; ++y)
    {
        for(int x = 0; x < w; ++x)
        {
            const Point3d p = _mp.CArr[_rc] + (_mp.iCamArr[_rc] * Point2d(static_cast<float>(x), static_cast<float>(y))).normalize() * out_depthSimMapUpscaled._dsm[y * w + x].depth;
            DepthSim& depthSim = out_depthSimMapUpscaled._dsm[y * w + x];

            if(_refineParams.useTcOrRcPixSize)
            {
                depthSim.sim = _mp.getCamsMinPixelSize(p, _tCams);
            }
            else
            {
                depthSim.sim = _mp.getCamPixelSize(p, _rc);
            }
        }
    }
}

void Refine::filterMaskedPixels(DepthSimMap& out_depthSimMap)
{
    mvsUtils::ImagesCache<ImageRGBAf>::ImgSharedPtr img = _cps._ic.getImg_sync(_rc);

    const int h = _mp.getHeight(_rc);
    const int w = _mp.getWidth(_rc);

    for(int y = 0; y < h; ++y)
    {
        for(int x = 0; x < w; ++x)
        {
            const ColorRGBAf& floatRGBA = img->at(x, y);

            if(floatRGBA.a < 0.1f)
            {
                DepthSim& depthSim = out_depthSimMap._dsm[y * w + x];

                depthSim.depth = -2.0;
                depthSim.sim = -1.0;
            }
        }
    }
}

void Refine::refineAndFuseDepthSimMap(const CudaDeviceMemoryPitched<float2, 2>& depthSimMapSgmUpscale_dmp, CudaDeviceMemoryPitched<float2, 2>& out_depthSimMapRefinedFused_dmp) const
{
    const system::Timer timer;

    ALICEVISION_LOG_INFO("Refine and fuse depth/sim map (rc: " << _rc << ")");

    _cps.refineAndFuseDepthSimMap(_rc, 
                                  out_depthSimMapRefinedFused_dmp,
                                  depthSimMapSgmUpscale_dmp,
                                  _tCams.getData(),
                                  _refineParams);

    ALICEVISION_LOG_INFO("Refine and fuse depth/sim map (rc: " << _rc << ") done in: " << timer.elapsedMs() << " ms.");
}

void Refine::optimizeDepthSimMap(const CudaDeviceMemoryPitched<float2, 2>& depthSimMapSgmUpscale_dmp,     // upscaled SGM depth/sim map
                                 const CudaDeviceMemoryPitched<float2, 2>& depthSimMapRefinedFused_dmp,   // refined and fused depth/sim map
                                 CudaDeviceMemoryPitched<float2, 2>& out_depthSimMapOptimized_dmp) const  // optimized depth/sim map
{
    const system::Timer timer;

    ALICEVISION_LOG_INFO("Optimize depth/sim map (rc: " << _rc << ")");

    if(_refineParams.nIters == 0)
    {
        out_depthSimMapOptimized_dmp.copyFrom(depthSimMapRefinedFused_dmp);
        return;
    }

    _cps.optimizeDepthSimMapGradientDescent(_rc, 
                                            out_depthSimMapOptimized_dmp, 
                                            depthSimMapSgmUpscale_dmp, 
                                            depthSimMapRefinedFused_dmp, 
                                            _refineParams);

    ALICEVISION_LOG_INFO("Optimize depth/sim map (rc: " << _rc << ") done in: " << timer.elapsedMs() << " ms.");
}

bool Refine::refineRc(const DepthSimMap& sgmDepthSimMap)
{
    const system::Timer timer;
    const IndexT viewId = _mp.getViewId(_rc);

    ALICEVISION_LOG_INFO("Refine depth/sim map of view id: " << viewId << ", rc: " << _rc << " (" << (_rc + 1) << " / " << _mp.ncams << ")");

    if(_tCams.empty())
    {
        return false;
    }

    // compute depth/sim map dimensions
    const int depthSimMapX = _mp.getWidth(_rc) / _refineParams.scale;
    const int depthSimMapY = _mp.getHeight(_rc) / _refineParams.scale;
    const CudaSize<2> depthSimMapDim(depthSimMapX, depthSimMapY);

    DepthSimMap depthSimMapSgmUpscale(_rc, _mp, _refineParams.scale, 1); // depthSimMapVis

    // upscale SGM depth/sim map
    upscaleSgmDepthSimMap(sgmDepthSimMap, depthSimMapSgmUpscale);

    // filter masked pixels (alpha < 0.1)
    filterMaskedPixels(depthSimMapSgmUpscale);

    if(_refineParams.exportIntermediateResults)
    {
        depthSimMapSgmUpscale.save("_sgmUpscaled");
    }


    CudaDeviceMemoryPitched<float2, 2> sgmDepthPixSizeMap_dmp(depthSimMapDim);
    CudaDeviceMemoryPitched<float2, 2> refinedDepthSimMap_dmp(depthSimMapDim);

    copy(sgmDepthPixSizeMap_dmp, depthSimMapSgmUpscale);

    // refine and fuse depth/sim map
    if(_refineParams.doRefineFuse)
    {
        refineAndFuseDepthSimMap(sgmDepthPixSizeMap_dmp, refinedDepthSimMap_dmp);

        if(_refineParams.exportIntermediateResults)
        {
            DepthSimMap depthSimMapRefinedFused(_rc, _mp, _refineParams.scale, 1); // depthSimMapPhoto
            copy(depthSimMapRefinedFused, refinedDepthSimMap_dmp);
            depthSimMapRefinedFused.save("_refinedFused");
        }
    }
    else
    {
        copyDepth(refinedDepthSimMap_dmp, sgmDepthPixSizeMap_dmp, 1.0f);
    }

    // optimize depth/sim map
    if(_refineParams.doRefineOpt && _refineParams.nIters != 0)
    {
        CudaDeviceMemoryPitched<float2, 2> optimizedDepthSimMap_dmp(depthSimMapDim);
        optimizeDepthSimMap(sgmDepthPixSizeMap_dmp, refinedDepthSimMap_dmp, optimizedDepthSimMap_dmp);
        copy(_depthSimMap, optimizedDepthSimMap_dmp);
    }
    else
    {
        copy(_depthSimMap, refinedDepthSimMap_dmp);
    }

    ALICEVISION_LOG_INFO("Refine depth/sim map (rc: " << _rc << ") done in: " << timer.elapsedMs() << " ms.");
    return true;
}

} // namespace depthMap
} // namespace aliceVision
