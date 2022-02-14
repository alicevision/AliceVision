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
#include <aliceVision/depthMap/cuda/host/memory.hpp>
#include <aliceVision/depthMap/cuda/host/DeviceCache.hpp>
#include <aliceVision/depthMap/cuda/planeSweeping/deviceDepthSimilarityMap.hpp>
#include <aliceVision/depthMap/cuda/planeSweeping/deviceSimilarityVolume.hpp>

namespace aliceVision {
namespace depthMap {

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

Refine::Refine(const RefineParams& refineParams, const mvsUtils::MultiViewParams& mp, mvsUtils::ImagesCache<ImageRGBAf>& ic, int rc, const ROI& roi)
    : _rc(rc)
    , _mp(mp)
    , _ic(ic)
    , _refineParams(refineParams)
    , _depthSimMap(_rc, _mp, _refineParams.scale, _refineParams.stepXY, roi)
{
    _tCams = _mp.findNearestCamsFromLandmarks(_rc, _refineParams.maxTCams);
}

void Refine::upscaleSgmDepthSimMap(const DepthSimMap& sgmDepthSimMap, DepthSimMap& out_depthSimMapUpscaled) const
{
    const int width = out_depthSimMapUpscaled.getWidth();
    const int height = out_depthSimMapUpscaled.getHeight();

    out_depthSimMapUpscaled.initFromSmaller(sgmDepthSimMap);

    // set sim (y) to pixsize
    for(int y = 0; y < height; ++y)
    {
        for(int x = 0; x < width; ++x)
        {
            DepthSim& depthSim = out_depthSimMapUpscaled.getDepthSim(x, y);
            const Point2d p2d = out_depthSimMapUpscaled.getCorrespondingImagePoint(x, y);
            const Point3d p3d = _mp.CArr[_rc] + (_mp.iCamArr[_rc] * p2d).normalize() * depthSim.depth;
            
            if(_refineParams.useTcOrRcPixSize)
            {
                depthSim.sim = float(_mp.getCamsMinPixelSize(p3d, _tCams));
            }
            else
            {
                depthSim.sim = float(_mp.getCamPixelSize(p3d, _rc));
            }
        }
    }
}

void Refine::filterMaskedPixels(DepthSimMap& inout_depthSimMap)
{
    mvsUtils::ImagesCache<ImageRGBAf>::ImgSharedPtr img = _ic.getImg_sync(_rc);

    const int width = inout_depthSimMap.getWidth();
    const int height = inout_depthSimMap.getHeight();

    for(int y = 0; y < height; ++y)
    {
        for(int x = 0; x < width; ++x)
        {
            const Point2d p2d = inout_depthSimMap.getCorrespondingImagePoint(x, y);
            const ColorRGBAf& floatRGBA = img->at(p2d.x, p2d.y);

            if(floatRGBA.a < 0.1f)
            {
                DepthSim& depthSim = inout_depthSimMap.getDepthSim(x, y);

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

    // get the downscaled region of interest
    const ROI downscaledRoi = _depthSimMap.getDownscaledRoi();

    // get intermediate depth/sim map size
    const CudaSize<2> depthSimMapSize(in_depthSimMapSgmUpscale_dmp.getSize());

    // build the vector of refine RcTc depth/sim map
    std::vector<CudaDeviceMemoryPitched<float2, 2>> depthSimMapPerRcTc_dmp;
    depthSimMapPerRcTc_dmp.resize(_tCams.size());

    for(int tci = 0; tci < _tCams.size(); ++tci)
    {
        // get Tc global id
        const int tc = _tCams[tci];

        // allocate each refine RcTc depth/sim map
        CudaDeviceMemoryPitched<float2, 2>& rcTcDepthSimMap_dmp = depthSimMapPerRcTc_dmp.at(tci);
        rcTcDepthSimMap_dmp.allocate(depthSimMapSize);

        // initialize depth/sim map with the SGM depth map (to get middle depth)
        copyDepthOnly(rcTcDepthSimMap_dmp, in_depthSimMapSgmUpscale_dmp, 1.0f); 

        // get device cameras
        DeviceCache& deviceCache = DeviceCache::getInstance();
        const DeviceCamera& rcDeviceCamera = deviceCache.requestCamera(_rc, _refineParams.scale, _ic, _mp, 0 /*stream*/);
        const DeviceCamera& tcDeviceCamera = deviceCache.requestCamera( tc, _refineParams.scale, _ic, _mp, 0 /*stream*/);

        // refine each RcTc depth/sim map
        cuda_refineDepthMap(rcTcDepthSimMap_dmp, 
                            rcDeviceCamera, 
                            tcDeviceCamera, 
                            _refineParams, 
                            downscaledRoi, 
                            0 /*stream*/);
    }

    // fuse each refined RcTc depth/sim map
    cuda_fuseDepthSimMapsGaussianKernelVoting(out_depthSimMapRefinedFused_dmp, 
                                              in_depthSimMapSgmUpscale_dmp,
                                              depthSimMapPerRcTc_dmp, 
                                              _refineParams,
                                              downscaledRoi, 
                                              0 /*stream*/);

    ALICEVISION_LOG_INFO("Refine and fuse depth/sim map (rc: " << _rc << ") done in: " << timer.elapsedMs() << " ms.");
}

void Refine::refineAndFuseDepthSimMapVolume(const CudaDeviceMemoryPitched<float2, 2>& in_depthSimMapSgmUpscale_dmp, CudaDeviceMemoryPitched<float2, 2>& out_depthSimMapRefinedFused_dmp) const
{
    const system::Timer timer;

    ALICEVISION_LOG_INFO("Refine and fuse depth/sim map volume (rc: " << _rc << ")");

    // compute volume dimensions
    // should be (rc roi with x rc roi height x number of depth to refine) 
    const int volDimX = _depthSimMap.getWidth();
    const int volDimY = _depthSimMap.getHeight();
    const int volDimZ = _refineParams.nDepthsToRefine; // default value is 31

    const CudaSize<3> volDim(volDimX, volDimY, volDimZ);

    // get the downscaled region of interest
    ROI downscaledRoi = _depthSimMap.getDownscaledRoi();
    downscaledRoi.beginZ = 0;
    downscaledRoi.endZ = volDimZ;

    // allocate refine volume in device memory
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
        const DeviceCamera& rcDeviceCamera = deviceCache.requestCamera(_rc, _refineParams.scale, _ic, _mp, 0 /*stream*/);
        const DeviceCamera& tcDeviceCamera = deviceCache.requestCamera( tc, _refineParams.scale, _ic, _mp, 0 /*stream*/);

        ALICEVISION_LOG_DEBUG("Refine similarity volume:" << std::endl
                              << "\t- rc: " << _rc << std::endl
                              << "\t- tc: " << tc << " (" << tci << "/" << _tCams.size() << ")" << std::endl
                              << "\t- rc camera device id: " << rcDeviceCamera.getDeviceCamId() << std::endl
                              << "\t- tc camera device id: " << tcDeviceCamera.getDeviceCamId() << std::endl
                              << "\t- tile range x: [" << downscaledRoi.beginX << " - " << downscaledRoi.endX << "]" << std::endl
                              << "\t- tile range y: [" << downscaledRoi.beginY << " - " << downscaledRoi.endY << "]" << std::endl
                              << "\t- device similarity volume size: " << volumeRefineRcTcSim_dmp.getBytesPadded() / (1024.0 * 1024.0) << " MB" << std::endl
                              << "\t- device unpadded similarity volume size: " << volumeRefineRcTcSim_dmp.getBytesUnpadded() / (1024.0 * 1024.0) << " MB" << std::endl);

        cuda_volumeRefineSimilarity(volumeRefineRcTcSim_dmp, 
                                    in_depthSimMapSgmUpscale_dmp,
                                    rcDeviceCamera, 
                                    tcDeviceCamera,
                                    _refineParams, 
                                    downscaledRoi, 
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
        const DeviceCamera& rcDeviceCamera = deviceCache.requestCamera(_rc, _refineParams.scale, _ic, _mp, 0 /*stream*/);

        cuda_volumeRefineBestDepth(out_depthSimMapRefinedFused_dmp, 
                                   in_depthSimMapSgmUpscale_dmp, 
                                   volumeRefineSim_dmp,
                                   rcDeviceCamera, 
                                   _tCams.size(),
                                   _refineParams,
                                   downscaledRoi, 
                                   0 /*stream*/);
    }
}

void Refine::optimizeDepthSimMap(const CudaDeviceMemoryPitched<float2, 2>& in_depthSimMapSgmUpscale_dmp,     // upscaled SGM depth/sim map
                                 const CudaDeviceMemoryPitched<float2, 2>& in_depthSimMapRefinedFused_dmp,   // refined and fused depth/sim map
                                 CudaDeviceMemoryPitched<float2, 2>& out_depthSimMapOptimized_dmp) const     // optimized depth/sim map
{
    const system::Timer timer;

    ALICEVISION_LOG_INFO("Optimize depth/sim map (rc: " << _rc << ")");

    const ROI downscaledRoi = _depthSimMap.getDownscaledRoi();
    
    DeviceCache& deviceCache = DeviceCache::getInstance();
    const DeviceCamera& rcDeviceCamera = deviceCache.requestCamera(_rc, _refineParams.scale, _ic, _mp);

    cuda_optimizeDepthSimMapGradientDescent(out_depthSimMapOptimized_dmp,
                                            in_depthSimMapSgmUpscale_dmp, 
                                            in_depthSimMapRefinedFused_dmp, 
                                            rcDeviceCamera,
                                            _refineParams,
                                            downscaledRoi,
                                            0 /*stream*/);

    ALICEVISION_LOG_INFO("Optimize depth/sim map (rc: " << _rc << ") done in: " << timer.elapsedMs() << " ms.");
}

void Refine::exportVolumeInformation(const CudaDeviceMemoryPitched<TSimRefine, 3>& in_volSim_dmp,
                                     const CudaDeviceMemoryPitched<float2, 2>& in_depthSimMapSgmUpscale_dmp,
                                     const std::string& name) const
{
    const IndexT viewId = _mp.getViewId(_rc);
    const ROI& roi = _depthSimMap.getRoi();

    CudaHostMemoryHeap<TSimRefine, 3> volumeSim_hmh(in_volSim_dmp.getSize());
    volumeSim_hmh.copyFrom(in_volSim_dmp);

    DepthSimMap depthSimMapSgmUpscale(_rc, _mp, _refineParams.scale, _refineParams.stepXY, roi);
    depthSimMapSgmUpscale.copyFrom(in_depthSimMapSgmUpscale_dmp);

    const std::string volumeCrossPath = getFileNameFromIndex(_mp, _rc, mvsUtils::EFileType::volumeCross, _refineParams.scale, "_" + name, roi.beginX, roi.beginY);
    const std::string stats9Path = getFileNameFromIndex(_mp, _rc, mvsUtils::EFileType::stats9p, _refineParams.scale, "_refine", roi.beginX, roi.beginY);

    exportSimilarityVolumeCross(volumeSim_hmh, depthSimMapSgmUpscale, _mp, _rc, _refineParams, volumeCrossPath);
    exportSimilaritySamplesCSV(volumeSim_hmh, _rc, name, stats9Path);
}

void Refine::refineRc(const DepthSimMap& sgmDepthSimMap)
{
    const system::Timer timer;
    const IndexT viewId = _mp.getViewId(_rc);

    ALICEVISION_LOG_INFO("Refine depth/sim map of view id: " << viewId << ", rc: " << _rc << " (" << (_rc + 1) << " / " << _mp.ncams << ")");

    if(_tCams.empty())
    {
        return;
    }

    // compute depth/sim map dimensions
    const CudaSize<2> depthSimMapDim(_depthSimMap.getWidth(), _depthSimMap.getHeight());

    DepthSimMap depthSimMapSgmUpscale(_rc, _mp, _refineParams.scale, _refineParams.stepXY, _depthSimMap.getRoi());

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

    depthSimMapSgmUpscale.copyTo(sgmDepthPixSizeMap_dmp);

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
            DepthSimMap depthSimMapRefinedFused(_rc, _mp, _refineParams.scale, _refineParams.stepXY, _depthSimMap.getRoi()); // depthSimMapPhoto
            depthSimMapRefinedFused.copyFrom(refinedDepthSimMap_dmp);
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
        _depthSimMap.copyFrom(optimizedDepthSimMap_dmp);
    }
    else
    {
        _depthSimMap.copyFrom(refinedDepthSimMap_dmp);
    }

    ALICEVISION_LOG_INFO("Refine depth/sim map (rc: " << _rc << ") done in: " << timer.elapsedMs() << " ms.");
}

} // namespace depthMap
} // namespace aliceVision
