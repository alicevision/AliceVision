// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Sgm.hpp"

#include <aliceVision/alicevision_omp.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/Timer.hpp>
#include <aliceVision/depthMap/SgmDepthList.hpp>
#include <aliceVision/depthMap/SgmParams.hpp>
#include <aliceVision/depthMap/volumeIO.hpp>
#include <aliceVision/depthMap/ROI.hpp>
#include <aliceVision/depthMap/cuda/host/utils.hpp>
#include <aliceVision/depthMap/cuda/host/memory.hpp>
#include <aliceVision/depthMap/cuda/host/DeviceCache.hpp>
#include <aliceVision/depthMap/cuda/planeSweeping/deviceSimilarityVolume.hpp>

#include <iostream>
#include <sstream>

namespace aliceVision {
namespace depthMap {

Sgm::Sgm(const SgmParams& sgmParams, const SgmDepthList& sgmDepthList, const mvsUtils::MultiViewParams& mp, mvsUtils::ImagesCache<ImageRGBAf>& ic, int rc)
    : _rc(rc)
    , _mp(mp)
    , _ic(ic)
    , _sgmParams(sgmParams)
    , _sgmDepthList(sgmDepthList)
    , _depthSimMap(_rc, _mp, _sgmParams.scale, _sgmParams.stepXY)
{}

bool Sgm::sgmRc()
{
    const system::Timer timer;
    const IndexT viewId = _mp.getViewId(_rc);

    ALICEVISION_LOG_INFO("SGM depth/sim map of view id: " << viewId << ", rc: " << _rc << " (" << (_rc + 1) << " / " << _mp.ncams << ")");

    if(_sgmDepthList.getTCams().empty())
    {
      return false;
    }

    // compute volume dimensions
    const int volDimX = _mp.getWidth(_rc) / (_sgmParams.scale * _sgmParams.stepXY);
    const int volDimY = _mp.getHeight(_rc) / (_sgmParams.scale * _sgmParams.stepXY);
    const int volDimZ = _sgmDepthList.getDepths().size();

    const CudaSize<3> volDim(volDimX, volDimY, volDimZ);

    // log volumes allocation size / gpu device id
    ALICEVISION_LOG_DEBUG("Allocating 2 volumes (x: " << volDim.x() << ", y: " << volDim.y() << ", z: " << volDim.z() << ") on GPU device " << getCudaDeviceId() << ".");

    // allocate best sim and second best sim volumes
    CudaDeviceMemoryPitched<TSim, 3> volumeBestSim_dmp(volDim);
    CudaDeviceMemoryPitched<TSim, 3> volumeSecBestSim_dmp(volDim);

    // compute best sim and second best sim volumes
    computeSimilarityVolumes(volumeBestSim_dmp, volumeSecBestSim_dmp);

    // particular case with only one tc
    if(_tCams.size() < 2)
    {
        // the second best volume has no valid similarity values
        volumeSecBestSim_dmp.copyFrom(volumeBestSim_dmp);
    }

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
    return true;
}

void Sgm::computeSimilarityVolumes(CudaDeviceMemoryPitched<TSim, 3>& out_volBestSim_dmp, CudaDeviceMemoryPitched<TSim, 3>& out_volSecBestSim_dmp) const
{
    const system::Timer timer;

    const CudaSize<3>& volDim = out_volBestSim_dmp.getSize();

    ALICEVISION_LOG_INFO("SGM Compute similarity volume (rc: " << _rc << " x: " << volDim.x() << ", y: " << volDim.y() << ", z: " << volDim.z() << ")");

    // initialize the two similarity volumes at 255
    cuda_volumeInitialize(out_volBestSim_dmp, 255.f, 0 /*stream*/);
    cuda_volumeInitialize(out_volSecBestSim_dmp, 255.f, 0 /*stream*/);

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

    for(int tci = 0; tci < _sgmDepthList.getTCams().size(); ++tci)
    {
        const system::Timer timerPerTc;

        const int tc = _sgmDepthList.getTCams()[tci];

        const int firstDepth = _sgmDepthList.getDepthsTcLimits()[tci].x;
        const int lastDepth = firstDepth + _sgmDepthList.getDepthsTcLimits()[tci].y;

        DeviceCache& deviceCache = DeviceCache::getInstance();

        const DeviceCamera& rcDeviceCamera = deviceCache.requestCamera(_rc, _sgmParams.scale, _ic, _mp, 0);
        const DeviceCamera& tcDeviceCamera = deviceCache.requestCamera( tc, _sgmParams.scale, _ic, _mp, 0);

        const ROI roi(0, out_volBestSim_dmp.getSize().x(), 0, out_volBestSim_dmp.getSize().y(), firstDepth, lastDepth);

        ALICEVISION_LOG_DEBUG("Compute similarity volume:" << std::endl
                              << "\t- rc: " << _rc << std::endl
                              << "\t- tc: " << tc << " (" << tci << "/" << _sgmDepthList.getDepths().size() << ")" << std::endl
                              << "\t- rc camera device id: " << rcDeviceCamera.getDeviceCamId() << std::endl
                              << "\t- tc camera device id: " << tcDeviceCamera.getDeviceCamId() << std::endl
                              << "\t- tc first depth: " << firstDepth << std::endl
                              << "\t- tc last depth: " << lastDepth << std::endl
                              << "\t- rc width: " << rcDeviceCamera.getWidth() << std::endl
                              << "\t- rc height: " << rcDeviceCamera.getHeight() << std::endl
                              << "\t- device similarity volume size: " << out_volBestSim_dmp.getBytesPadded() / (1024.0 * 1024.0) << " MB" << std::endl
                              << "\t- device unpadded similarity volume size: " << out_volBestSim_dmp.getBytesUnpadded() / (1024.0 * 1024.0) << " MB" << std::endl);

        cuda_volumeComputeSimilarity(out_volBestSim_dmp, 
                                     out_volSecBestSim_dmp, 
                                     depths_d, 
                                     rcDeviceCamera, 
                                     tcDeviceCamera,
                                     _sgmParams, 
                                     roi, 
                                     0 /*stream*/);

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

    const DeviceCamera& rcDeviceCamera = deviceCache.requestCamera(_rc, _sgmParams.scale, _ic, _mp);

    cuda_volumeOptimize(out_volSimOptimized_dmp, 
                        in_volSim_dmp, 
                        rcDeviceCamera, 
                        _sgmParams, 
                        0 /*stream*/);

    ALICEVISION_LOG_INFO("SGM Optimizing volume done in: " << timer.elapsedMs() << " ms.");
}

void Sgm::retrieveBestDepth(DepthSimMap& out_bestDepthSimMap, const CudaDeviceMemoryPitched<TSim, 3>& in_volSim_dmp) const 
{
    const system::Timer timer;

    const CudaSize<3>& volDim = in_volSim_dmp.getSize();
  
    ALICEVISION_LOG_INFO("SGM Retrieve best depth in volume (x: " << volDim.x() << ", y: " << volDim.y() << ", z: " << volDim.z() << ")");

    DeviceCache& deviceCache = DeviceCache::getInstance();

    const DeviceCamera& rcDeviceCamera = deviceCache.requestCamera(_rc, 1, _ic, _mp);

    const CudaSize<2> depthSimDim(volDim.x(), volDim.y());

    CudaDeviceMemory<float> depths_d(_sgmDepthList.getDepths().getData().data(), _sgmDepthList.getDepths().size());
    CudaDeviceMemoryPitched<float, 2> bestDepth_dmp(depthSimDim);
    CudaDeviceMemoryPitched<float, 2> bestSim_dmp(depthSimDim);

    const ROI roi(0, in_volSim_dmp.getSize().x(), 0, in_volSim_dmp.getSize().y(), 0, in_volSim_dmp.getSize().z());

    cuda_volumeRetrieveBestDepth(bestDepth_dmp,
                                 bestSim_dmp, 
                                 in_volSim_dmp, 
                                 depths_d, 
                                 rcDeviceCamera,
                                 _sgmParams,
                                 roi, 
                                 0 /*stream*/);

    CudaHostMemoryHeap<float, 2> bestDepth_hmh(depthSimDim);
    bestDepth_hmh.copyFrom(bestDepth_dmp);

    CudaHostMemoryHeap<float, 2> bestSim_hmh(depthSimDim);
    bestSim_hmh.copyFrom(bestSim_dmp);

    for(int y = 0; y < depthSimDim.y(); ++y)
    {
        for(int x = 0; x < depthSimDim.x(); ++x)
        {
            DepthSim& out = out_bestDepthSimMap._dsm[y * depthSimDim.x() + x];
            out.depth = bestDepth_hmh(x, y);
            out.sim = bestSim_hmh(x, y);
        }
    }

    ALICEVISION_LOG_INFO("SGM Retrieve best depth in volume done in: " << timer.elapsedMs() << " ms.");
}

void Sgm::exportVolumeInformation(const CudaDeviceMemoryPitched<TSim, 3>& in_volSim_dmp, const std::string& name) const
{
    const IndexT viewId = _mp.getViewId(_rc);
    CudaHostMemoryHeap<TSim, 3> volumeSim_hmh(in_volSim_dmp.getSize());
    volumeSim_hmh.copyFrom(in_volSim_dmp);

    exportSimilarityVolume(volumeSim_hmh, _sgmDepthList.getDepths(), _mp, _rc, _sgmParams, _mp.getDepthMapsFolder() + std::to_string(viewId) + "_vol_" + name + ".abc");
    exportSimilarityVolumeCross(volumeSim_hmh, _sgmDepthList.getDepths(), _mp, _rc, _sgmParams, _mp.getDepthMapsFolder() + std::to_string(viewId) + "_vol-cross_" + name + ".abc");
    exportSimilaritySamplesCSV(volumeSim_hmh, _sgmDepthList.getDepths(), _rc, name, _mp.getDepthMapsFolder() + std::to_string(viewId) + "_9p.csv");
}

} // namespace depthMap
} // namespace aliceVision
