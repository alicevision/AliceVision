// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Sgm.hpp"

#include <aliceVision/alicevision_omp.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/Timer.hpp>
#include <aliceVision/depthMap/TileParams.hpp>
#include <aliceVision/depthMap/SgmParams.hpp>
#include <aliceVision/depthMap/volumeIO.hpp>
#include <aliceVision/depthMap/cuda/host/utils.hpp>
#include <aliceVision/depthMap/cuda/host/memory.hpp>
#include <aliceVision/depthMap/cuda/host/DeviceCache.hpp>
#include <aliceVision/depthMap/cuda/planeSweeping/deviceSimilarityVolume.hpp>

#include <iostream>
#include <sstream>

namespace aliceVision {
namespace depthMap {

Sgm::Sgm(int rc, 
         mvsUtils::ImagesCache<ImageRGBAf>& ic, 
         const mvsUtils::MultiViewParams& mp, 
         const SgmParams& sgmParams,
         const TileParams& tileParams, 
         const ROI& roi, 
         cudaStream_t stream)
    : _rc(rc)
    , _mp(mp)
    , _ic(ic)
    , _tileParams(tileParams)
    , _sgmParams(sgmParams)
    , _sgmDepthList(sgmParams, mp, rc, roi)
    , _depthSimMap(_rc, _mp, _sgmParams.scale, _sgmParams.stepXY, tileParams, roi)
    , _stream(stream)
{
    // compute the R camera depth list
    _sgmDepthList.computeListRc();

    if(_sgmDepthList.getDepths().empty())
        return;

    // log debug camera / depth information
    _sgmDepthList.logRcTcDepthInformation();

    // check if starting and stopping depth are valid
    _sgmDepthList.checkStartingAndStoppingDepth();

    // preload T cams async in image cache
    ic.refreshImages_async(_sgmDepthList.getTCams().getData());
}

void Sgm::sgmRc()
{
    const system::Timer timer;
    const IndexT viewId = _mp.getViewId(_rc);

    ALICEVISION_LOG_INFO("SGM depth/sim map of view id: " << viewId << ", rc: " << _rc << " (" << (_rc + 1) << " / " << _mp.ncams << ")");

    if(_sgmDepthList.getTCams().empty() | _sgmDepthList.getDepths().empty())
    {
      return;
    }

    // compute volume dimensions
    const int volDimX = _depthSimMap.getWidth();
    const int volDimY = _depthSimMap.getHeight();
    const int volDimZ = _sgmDepthList.getDepths().size();

    const CudaSize<3> volDim(volDimX, volDimY, volDimZ);

    // log volumes allocation size / gpu device id
    ALICEVISION_LOG_DEBUG("Allocating 2 volumes (x: " << volDim.x() << ", y: " << volDim.y() << ", z: " << volDim.z() << ") on GPU device " << getCudaDeviceId() << ".");

    // allocate best sim and second best sim volumes
    CudaDeviceMemoryPitched<TSim, 3> volumeBestSim_dmp(volDim);
    CudaDeviceMemoryPitched<TSim, 3> volumeSecBestSim_dmp(volDim);

    // compute best sim and second best sim volumes
    computeSimilarityVolumes(volumeBestSim_dmp, volumeSecBestSim_dmp);

    if(_sgmParams.exportIntermediateResults)
        exportVolumeInformation(volumeSecBestSim_dmp, "beforeFiltering");

    // reuse best sim to put optimized sim volume
    CudaDeviceMemoryPitched<TSim, 3>& volumeOptimizedSim_dmp = volumeBestSim_dmp;

    // this is here for experimental reason ... to show how SGGC work on non
    // optimized depthmaps ... it must equals to true in normal case
    if(_sgmParams.doSgmOptimizeVolume)                      
    {
        optimizeSimilarityVolume(volumeOptimizedSim_dmp, volumeSecBestSim_dmp);
    }
    else
    {
        volumeOptimizedSim_dmp.copyFrom(volumeSecBestSim_dmp);
    }

    if(_sgmParams.exportIntermediateResults)
        exportVolumeInformation(volumeOptimizedSim_dmp, "afterFiltering");

    // retrieve best depth
    retrieveBestDepth(_depthSimMap, volumeOptimizedSim_dmp);

    if(_sgmParams.exportIntermediateResults)
    {
        _depthSimMap.save("_sgm");
        _depthSimMap.save("_sgmStep1", true);
    }

    ALICEVISION_LOG_INFO("SGM depth/sim map (rc: " << _rc << ") done in: " << timer.elapsedMs() << " ms.");
}

void Sgm::computeSimilarityVolumes(CudaDeviceMemoryPitched<TSim, 3>& out_volBestSim_dmp, CudaDeviceMemoryPitched<TSim, 3>& out_volSecBestSim_dmp) const
{
    const system::Timer timer;

    const CudaSize<3>& volDim = out_volBestSim_dmp.getSize();

    ALICEVISION_LOG_INFO("SGM Compute similarity volume (rc: " << _rc << " x: " << volDim.x() << ", y: " << volDim.y() << ", z: " << volDim.z() << ")");

    // initialize the two similarity volumes at 255
    cuda_volumeInitialize(out_volBestSim_dmp, 255.f, _stream);
    cuda_volumeInitialize(out_volSecBestSim_dmp, 255.f, _stream);

    // load rc & tc images in the CPU ImageCache
    _ic.getImg_sync(_rc);
    for(int tc : _sgmDepthList.getTCams())
    {
        _ic.getImg_sync(tc);
    }

    // copy rc depth data in device memory
    CudaDeviceMemory<float> depths_d(_sgmDepthList.getDepths().getData().data(), _sgmDepthList.getDepths().size());

    // log memory information
    logDeviceMemoryInfo();

    // get the downscaled 2d region of interest
    const ROI downscaledRoi = _depthSimMap.getDownscaledRoi();

    for(int tci = 0; tci < _sgmDepthList.getTCams().size(); ++tci)
    {
        const system::Timer timerPerTc;

        const int tc = _sgmDepthList.getTCams()[tci];

        const int firstDepth = _sgmDepthList.getDepthsTcLimits()[tci].x;
        const int lastDepth = firstDepth + _sgmDepthList.getDepthsTcLimits()[tci].y;

        const Range tcDepthRange(firstDepth, lastDepth);

        DeviceCache& deviceCache = DeviceCache::getInstance();
        const DeviceCamera& rcDeviceCamera = deviceCache.requestCamera(_rc, _sgmParams.scale, _ic, _mp, _stream);
        const DeviceCamera& tcDeviceCamera = deviceCache.requestCamera( tc, _sgmParams.scale, _ic, _mp, _stream);

        ALICEVISION_LOG_DEBUG("Compute similarity volume:" << std::endl
                              << "\t- rc: " << _rc << std::endl
                              << "\t- tc: " << tc << " (" << tci << "/" << _sgmDepthList.getDepths().size() << ")" << std::endl
                              << "\t- rc camera device id: " << rcDeviceCamera.getDeviceCamId() << std::endl
                              << "\t- tc camera device id: " << tcDeviceCamera.getDeviceCamId() << std::endl
                              << "\t- tc first depth: " << firstDepth << std::endl
                              << "\t- tc last depth: " << lastDepth << std::endl
                              << "\t- rc width: " << rcDeviceCamera.getWidth() << std::endl
                              << "\t- rc height: " << rcDeviceCamera.getHeight() << std::endl
                              << "\t- tile range x: [" << downscaledRoi.x.begin << " - " << downscaledRoi.x.end << "]" << std::endl
                              << "\t- tile range y: [" << downscaledRoi.y.begin << " - " << downscaledRoi.y.end << "]" << std::endl
                              << "\t- device similarity volume size: " << out_volBestSim_dmp.getBytesPadded() / (1024.0 * 1024.0) << " MB" << std::endl
                              << "\t- device unpadded similarity volume size: " << out_volBestSim_dmp.getBytesUnpadded() / (1024.0 * 1024.0) << " MB" << std::endl);

        cuda_volumeComputeSimilarity(out_volBestSim_dmp, 
                                     out_volSecBestSim_dmp, 
                                     depths_d, 
                                     rcDeviceCamera, 
                                     tcDeviceCamera,
                                     _sgmParams, 
                                     tcDepthRange,
                                     downscaledRoi, 
                                     _stream);

        ALICEVISION_LOG_DEBUG("Compute similarity volume (with rc: " << _rc << ", tc: " << tc << ") done in: " << timerPerTc.elapsedMs() << " ms.");
    }
    ALICEVISION_LOG_INFO("SGM Compute similarity volume (rc: " << _rc << ") done in: " << timer.elapsedMs() << " ms.");
}

void Sgm::optimizeSimilarityVolume(CudaDeviceMemoryPitched<TSim, 3>& out_volSimOptimized_dmp, const CudaDeviceMemoryPitched<TSim, 3>& in_volSim_dmp) const
{
    const system::Timer timer;

    const CudaSize<3>& volDim = in_volSim_dmp.getSize();

    ALICEVISION_LOG_INFO("SGM Optimizing volume:" << std::endl
                          << "\t- filtering axes: " << _sgmParams.filteringAxes << std::endl
                          << "\t- volume dimensions: (x: " << volDim.x() << ", y: " << volDim.y() << ", z: " << volDim.z() << ")" << std::endl
                          << "\t- device similarity volume size: " << (double(in_volSim_dmp.getBytesPadded()) / (1024.0 * 1024.0)) << " MB" << std::endl);

    DeviceCache& deviceCache = DeviceCache::getInstance();
    const DeviceCamera& rcDeviceCamera = deviceCache.requestCamera(_rc, _sgmParams.scale, _ic, _mp, _stream);

    // get the downscaled 2d region of interest
    const ROI downscaledRoi = _depthSimMap.getDownscaledRoi();
    
    cuda_volumeOptimize(out_volSimOptimized_dmp, 
                        in_volSim_dmp, 
                        rcDeviceCamera, 
                        _sgmParams, 
                        downscaledRoi,
                        _stream);

    ALICEVISION_LOG_INFO("SGM Optimizing volume done in: " << timer.elapsedMs() << " ms.");
}

void Sgm::retrieveBestDepth(DepthSimMap& out_bestDepthSimMap, const CudaDeviceMemoryPitched<TSim, 3>& in_volSim_dmp) const
{
    const system::Timer timer;

    const CudaSize<3>& volDim = in_volSim_dmp.getSize();
  
    ALICEVISION_LOG_INFO("SGM Retrieve best depth in volume (x: " << volDim.x() << ", y: " << volDim.y() << ", z: " << volDim.z() << ")");

    DeviceCache& deviceCache = DeviceCache::getInstance();
    const DeviceCamera& rcDeviceCamera = deviceCache.requestCamera(_rc, 1, _ic, _mp, _stream);

    const CudaSize<2> depthSimDim(volDim.x(), volDim.y());

    CudaDeviceMemory<float> depths_d(_sgmDepthList.getDepths().getData().data(), _sgmDepthList.getDepths().size());
    CudaDeviceMemoryPitched<float, 2> bestDepth_dmp(depthSimDim);
    CudaDeviceMemoryPitched<float, 2> bestSim_dmp(depthSimDim);

    // get the downscaled 2d region of interest
    const ROI downscaledRoi = _depthSimMap.getDownscaledRoi();

    // get depth range
    const Range depthRange(0, in_volSim_dmp.getSize().z());

    cuda_volumeRetrieveBestDepth(bestDepth_dmp,
                                 bestSim_dmp, 
                                 in_volSim_dmp, 
                                 depths_d, 
                                 rcDeviceCamera,
                                 _sgmParams,
                                 depthRange,
                                 downscaledRoi, 
                                 _stream);

    out_bestDepthSimMap.copyFrom(bestDepth_dmp, bestSim_dmp);

    ALICEVISION_LOG_INFO("SGM Retrieve best depth in volume done in: " << timer.elapsedMs() << " ms.");
}

void Sgm::exportVolumeInformation(const CudaDeviceMemoryPitched<TSim, 3>& in_volSim_dmp, const std::string& name) const
{
    // get tile begin indexes (default no tile)
    int tileBeginX = -1;
    int tileBeginY = -1;

    if((_tileParams.width > 0) && (_tileParams.height > 0))
    {
        const ROI& roi = _depthSimMap.getRoi();
        tileBeginX = roi.x.begin;
        tileBeginY = roi.y.begin;
    }

    const IndexT viewId = _mp.getViewId(_rc);

    CudaHostMemoryHeap<TSim, 3> volumeSim_hmh(in_volSim_dmp.getSize());
    volumeSim_hmh.copyFrom(in_volSim_dmp);

    const std::string volumePath = getFileNameFromIndex(_mp, _rc, mvsUtils::EFileType::volume, _sgmParams.scale, "_" + name, tileBeginX, tileBeginY);
    const std::string volumeCrossPath = getFileNameFromIndex(_mp, _rc, mvsUtils::EFileType::volumeCross, _sgmParams.scale, "_" + name, tileBeginX, tileBeginY);
    const std::string stats9Path = getFileNameFromIndex(_mp, _rc, mvsUtils::EFileType::stats9p, _sgmParams.scale, "_sgm", tileBeginX, tileBeginY);

    exportSimilarityVolume(volumeSim_hmh, _sgmDepthList.getDepths(), _mp, _rc, _sgmParams, volumePath, _depthSimMap.getRoi());
    exportSimilarityVolumeCross(volumeSim_hmh, _sgmDepthList.getDepths(), _mp, _rc, _sgmParams, volumeCrossPath, _depthSimMap.getRoi());
    exportSimilaritySamplesCSV(volumeSim_hmh, _sgmDepthList.getDepths(), _rc, name, stats9Path);
}

} // namespace depthMap
} // namespace aliceVision
