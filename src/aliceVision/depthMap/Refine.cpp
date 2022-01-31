// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Refine.hpp"

#include <aliceVision/alicevision_omp.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/Timer.hpp>
#include <aliceVision/mvsData/Point2d.hpp>
#include <aliceVision/mvsData/Point3d.hpp>
#include <aliceVision/depthMap/RefineParams.hpp>
#include <aliceVision/depthMap/volumeIO.hpp>
#include <aliceVision/depthMap/cuda/memory.hpp>
#include <aliceVision/depthMap/cuda/DeviceCache.hpp>
#include <aliceVision/depthMap/cuda/planeSweeping/deviceRefine.hpp>
#include <aliceVision/depthMap/cuda/planeSweeping/deviceFuse.hpp>
#include <aliceVision/depthMap/cuda/planeSweeping/deviceSimilarityVolume.hpp>

namespace aliceVision {
namespace depthMap {

// copy from DepthSimMap to CudaDeviceMemoryPitched
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

// copy from CudaDeviceMemoryPitched to DepthSimMap
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

// copy from CudaDeviceMemoryPitched to CudaDeviceMemoryPitched depth only
void copyDepthOnly(CudaDeviceMemoryPitched<float2, 2>& out_depthSimMap_dmp, const CudaDeviceMemoryPitched<float2, 2>& in_depthSimMap_dmp, float defaultSim)
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

// copy ROI part of CudaDeviceMemoryPitched to CudaDeviceMemoryPitched
void copyRoiPart(CudaDeviceMemoryPitched<float2, 2>& out_depthSimMapPart_dmp, const CudaDeviceMemoryPitched<float2, 2>& in_depthSimMap_dmp, const ROI& in_roiPart)
{
    CudaHostMemoryHeap<float2, 2> depthSimMap_hmh(in_depthSimMap_dmp.getSize());
    depthSimMap_hmh.copyFrom(in_depthSimMap_dmp);

    CudaHostMemoryHeap<float2, 2> depthSimMapPart_hmh({in_roiPart.width(), in_roiPart.height()});

    for(int y = 0; y < in_roiPart.height(); ++y)
    {
        for(int x = 0; x < in_roiPart.width(); ++x)
        {
            float2& depthSimPart_h = depthSimMapPart_hmh(x, y);
            const float2& depthSim_h = depthSimMap_hmh(int(in_roiPart.beginX) + x, int(in_roiPart.beginY) + y);
            depthSimPart_h = depthSim_h;
        }
    }

    assert(depthSimMapPart_hmh.getSize() == out_depthSimMapPart_dmp.getSize());

    out_depthSimMapPart_dmp.copyFrom(depthSimMapPart_hmh);
}

Refine::Refine(const RefineParams& refineParams, const mvsUtils::MultiViewParams& mp, mvsUtils::ImagesCache<ImageRGBAf>& ic, int rc)
    : _rc(rc)
    , _mp(mp)
    , _ic(ic)
    , _refineParams(refineParams)
    , _depthSimMap(_rc, _mp, _refineParams.scale, 1)
{
    _tCams = _mp.findNearestCamsFromLandmarks(_rc, _refineParams.maxTCams);
}

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
    mvsUtils::ImagesCache<ImageRGBAf>::ImgSharedPtr img = _ic.getImg_sync(_rc);

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

void Refine::refineAndFuseDepthSimMap(const CudaDeviceMemoryPitched<float2, 2>& in_depthSimMapSgmUpscale_dmp, CudaDeviceMemoryPitched<float2, 2>& out_depthSimMapRefinedFused_dmp) const
{
    const system::Timer timer;

    ALICEVISION_LOG_INFO("Refine and fuse depth/sim map (rc: " << _rc << ")");

    const ROI roi(0, out_depthSimMapRefinedFused_dmp.getSize().x(), 0, out_depthSimMapRefinedFused_dmp.getSize().y());

    // build the vector of ROI part of each refined RcTc depth/sim map
    std::vector<CudaDeviceMemoryPitched<float2, 2>> depthSimMapPartPerRcTc_dmp;
    depthSimMapPartPerRcTc_dmp.resize(_tCams.size());

    const CudaSize<2> roiSize(roi.width(), roi.height());

    // get the ROI part of the upscaled SGM depth/sim map
    CudaDeviceMemoryPitched<float2, 2> depthSimMapPartSgm_dmp(roiSize);
    copyRoiPart(depthSimMapPartSgm_dmp, in_depthSimMapSgmUpscale_dmp, roi);

    for(int tci = 0; tci < _tCams.size(); ++tci)
    {
        // get Tc global id
        const int tc = _tCams[tci];

        // allocate the ROI part of each refined RcTc depth/sim map
        CudaDeviceMemoryPitched<float2, 2>& rcTcDepthSimMapPart_dmp = depthSimMapPartPerRcTc_dmp.at(tci);
        rcTcDepthSimMapPart_dmp.allocate(roiSize);

        // initialize depth/sim map with the ROI part of SGM depth map (to get middle depth)
        copyDepthOnly(rcTcDepthSimMapPart_dmp, depthSimMapPartSgm_dmp, 1.0f); 

        // get device cameras
        DeviceCache& deviceCache = DeviceCache::getInstance();
        const DeviceCamera& rcDeviceCamera = deviceCache.requestCamera(_rc, _refineParams.scale, _ic, _mp);
        const DeviceCamera& tcDeviceCamera = deviceCache.requestCamera( tc, _refineParams.scale, _ic, _mp);

        // refined the ROI part of each RcTc depth/sim map
        cuda_refineDepthMap(rcTcDepthSimMapPart_dmp, 
                            rcDeviceCamera, 
                            tcDeviceCamera, 
                            _refineParams, 
                            roi, 
                            0 /*stream*/);
    }

    // fuse the ROI part of each refined RcTc depth/sim map
    cuda_fuseDepthSimMapsGaussianKernelVoting(out_depthSimMapRefinedFused_dmp, 
                                              depthSimMapPartSgm_dmp,
                                              depthSimMapPartPerRcTc_dmp, 
                                              _refineParams,
                                              roi, 
                                              0 /*stream*/);

    ALICEVISION_LOG_INFO("Refine and fuse depth/sim map (rc: " << _rc << ") done in: " << timer.elapsedMs() << " ms.");
}

void Refine::refineAndFuseDepthSimMapVolume(const CudaDeviceMemoryPitched<float2, 2>& in_depthSimMapSgmUpscale_dmp, CudaDeviceMemoryPitched<float2, 2>& out_depthSimMapRefinedFused_dmp) const
{
    const system::Timer timer;

    ALICEVISION_LOG_INFO("Refine and fuse depth/sim map volume (rc: " << _rc << ")");

    // compute volume dimensions
    // should be (rc with x rc height x number of depth to refine) 
    const int volDimX = _mp.getWidth(_rc) / _refineParams.scale;
    const int volDimY = _mp.getHeight(_rc) / _refineParams.scale;
    const int volDimZ = _refineParams.nDepthsToRefine; // default value is 31

    const CudaSize<3> volDim(volDimX, volDimY, volDimZ);

    const ROI roi(0, volDimX, 0, volDimY, 0, volDimZ);

    CudaDeviceMemoryPitched<TSimRefine, 3> volumeRefineSim_dmp(volDim);

    // initialize the similarity volume at 0
    // each tc inverted similarity value will be summed in this volume
    cuda_volumeInitialize(volumeRefineSim_dmp, 0.f, 0 /*stream*/);

    // load rc & tc images in the CPU ImageCache
    _ic.getImg_sync(_rc);
    for(const auto& tc : _tCams)
        _ic.getImg_sync(tc);

    // compute for each RcTc each similarity value for each depth to refine
    // sum the inverted / filtered similarity value, best value is the HIGHEST
    for(int tci = 0; tci < _tCams.size(); ++tci)
    {
        const system::Timer timerPerTc;

        CudaDeviceMemoryPitched<TSimRefine, 3>& volumeRefineRcTcSim_dmp = volumeRefineSim_dmp;

        // for debug purposes
        // use a different volume for each RcTc to easily observe and experiment 
        //CudaDeviceMemoryPitched<TSimRefine, 3> volumeRefineRcTcSim_dmp(volDim);
        //cuda_volumeInitialize(volumeRefineRcTcSim_dmp, 0.f, 0 /*stream*/);

        const int tc = _tCams[tci];

        DeviceCache& deviceCache = DeviceCache::getInstance();
        const DeviceCamera& rcDeviceCamera = deviceCache.requestCamera(_rc, _refineParams.scale, _ic, _mp, 0);
        const DeviceCamera& tcDeviceCamera = deviceCache.requestCamera( tc, _refineParams.scale, _ic, _mp, 0);

        ALICEVISION_LOG_DEBUG("Refine similarity volume:" << std::endl
                              << "\t- rc: " << _rc << std::endl
                              << "\t- tc: " << tc << " (" << tci << "/" << _tCams.size() << ")" << std::endl
                              << "\t- rc camera device id: " << rcDeviceCamera.getDeviceCamId() << std::endl
                              << "\t- tc camera device id: " << tcDeviceCamera.getDeviceCamId() << std::endl
                              << "\t- device similarity volume size: " << volumeRefineRcTcSim_dmp.getBytesPadded() / (1024.0 * 1024.0) << " MB" << std::endl
                              << "\t- device unpadded similarity volume size: " << volumeRefineRcTcSim_dmp.getBytesUnpadded() / (1024.0 * 1024.0) << " MB" << std::endl);

        cuda_volumeRefineSimilarity(volumeRefineRcTcSim_dmp, 
                                    in_depthSimMapSgmUpscale_dmp,
                                    rcDeviceCamera, 
                                    tcDeviceCamera,
                                    _refineParams, 
                                    roi, 
                                    0 /*stream*/);

        // for debug purposes
        // add each RcTc to volumeRefineSim_dmp
        //cuda_volumeAdd(volumeRefineSim_dmp, volumeRefineRcTcSim_dmp, 0 /*stream*/);

        ALICEVISION_LOG_DEBUG("Refine similarity volume (with rc: " << _rc << ", tc: " << tc << ") done in: " << timerPerTc.elapsedMs() << " ms.");
    }

    if(_refineParams.exportIntermediateResults)
        exportVolumeInformation(volumeRefineSim_dmp, in_depthSimMapSgmUpscale_dmp, "afterRefine");

    // retrieve the best depth/sim in the volume
    // compute sub-pixel sample using a sliding gaussian 
    {
        DeviceCache& deviceCache = DeviceCache::getInstance();
        const DeviceCamera& rcDeviceCamera = deviceCache.requestCamera(_rc, _refineParams.scale, _ic, _mp, 0);

        cuda_volumeRefineBestDepth(out_depthSimMapRefinedFused_dmp, 
                                   in_depthSimMapSgmUpscale_dmp, 
                                   volumeRefineSim_dmp,
                                   rcDeviceCamera, 
                                   _tCams.size(),
                                   _refineParams,
                                   roi, 
                                   0 /*stream*/);
    }
}

void Refine::optimizeDepthSimMap(const CudaDeviceMemoryPitched<float2, 2>& in_depthSimMapSgmUpscale_dmp,     // upscaled SGM depth/sim map
                                 const CudaDeviceMemoryPitched<float2, 2>& in_depthSimMapRefinedFused_dmp,   // refined and fused depth/sim map
                                 CudaDeviceMemoryPitched<float2, 2>& out_depthSimMapOptimized_dmp) const     // optimized depth/sim map
{
    const system::Timer timer;

    ALICEVISION_LOG_INFO("Optimize depth/sim map (rc: " << _rc << ")");

    if(_refineParams.nIters == 0)
    {
        out_depthSimMapOptimized_dmp.copyFrom(in_depthSimMapRefinedFused_dmp);
        return;
    }

    const ROI roi(0, out_depthSimMapOptimized_dmp.getSize().x(), 0, out_depthSimMapOptimized_dmp.getSize().y());
    
    DeviceCache& deviceCache = DeviceCache::getInstance();
    const DeviceCamera& rcDeviceCamera = deviceCache.requestCamera(_rc, _refineParams.scale, _ic, _mp);

    cuda_optimizeDepthSimMapGradientDescent(out_depthSimMapOptimized_dmp,
                                            in_depthSimMapSgmUpscale_dmp, 
                                            in_depthSimMapRefinedFused_dmp, 
                                            rcDeviceCamera,
                                            _refineParams,
                                            roi,
                                            0 /*stream*/);

    ALICEVISION_LOG_INFO("Optimize depth/sim map (rc: " << _rc << ") done in: " << timer.elapsedMs() << " ms.");
}

void Refine::exportVolumeInformation(const CudaDeviceMemoryPitched<TSimRefine, 3>& in_volSim_dmp,
                                     const CudaDeviceMemoryPitched<float2, 2>& in_depthSimMapSgmUpscale_dmp,
                                     const std::string& name) const
{
    const IndexT viewId = _mp.getViewId(_rc);
    CudaHostMemoryHeap<TSimRefine, 3> volumeSim_hmh(in_volSim_dmp.getSize());
    volumeSim_hmh.copyFrom(in_volSim_dmp);

    DepthSimMap depthSimMapSgmUpscale(_rc, _mp, _refineParams.scale, _refineParams.stepXY);
    copy(depthSimMapSgmUpscale, in_depthSimMapSgmUpscale_dmp);

    exportSimilarityVolumeCross(volumeSim_hmh, depthSimMapSgmUpscale, _mp, _rc, _refineParams, _mp.getDepthMapsFolder() + std::to_string(viewId) + "_vol-cross_" + name + ".abc");
    exportSimilaritySamplesCSV(volumeSim_hmh, _rc, name, _mp.getDepthMapsFolder() + std::to_string(viewId) + "_9p.csv");
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
        if(_refineParams.useRefineFuseVolumeStrategy)
        {
            // refine and fuse with volume strategy
            refineAndFuseDepthSimMapVolume(sgmDepthPixSizeMap_dmp, refinedDepthSimMap_dmp);
        }
        else
        {
            // refine and fuse with original strategy based on depth/sim map
            refineAndFuseDepthSimMap(sgmDepthPixSizeMap_dmp, refinedDepthSimMap_dmp);
        }

        if(_refineParams.exportIntermediateResults)
        {
            DepthSimMap depthSimMapRefinedFused(_rc, _mp, _refineParams.scale, 1); // depthSimMapPhoto
            copy(depthSimMapRefinedFused, refinedDepthSimMap_dmp);
            depthSimMapRefinedFused.save("_refinedFused");
        }
    }
    else
    {
        copyDepthOnly(refinedDepthSimMap_dmp, sgmDepthPixSizeMap_dmp, 1.0f);
    }

    // optimize depth/sim map
    if(_refineParams.doRefineOptimization && _refineParams.nIters > 0)
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
