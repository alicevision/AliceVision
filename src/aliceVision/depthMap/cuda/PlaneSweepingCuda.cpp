// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "PlaneSweepingCuda.hpp"

#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/Timer.hpp>
#include <aliceVision/system/nvtx.hpp>

#include <aliceVision/mvsData/Matrix3x3.hpp>
#include <aliceVision/mvsData/Matrix3x4.hpp>
#include <aliceVision/mvsData/OrientedPoint.hpp>

#include <aliceVision/mvsUtils/common.hpp>
#include <aliceVision/mvsUtils/fileIO.hpp>

#include <aliceVision/depthMap/volumeIO.hpp>

#include <aliceVision/depthMap/cuda/utils.hpp>
#include <aliceVision/depthMap/cuda/DeviceCache.hpp>
#include <aliceVision/depthMap/cuda/imageProcessing/deviceGaussianFilter.hpp>
#include <aliceVision/depthMap/cuda/normalMapping/deviceNormalMap.hpp>
#include <aliceVision/depthMap/cuda/planeSweeping/deviceSimilarityVolume.hpp>
#include <aliceVision/depthMap/cuda/planeSweeping/deviceRefine.hpp>
#include <aliceVision/depthMap/cuda/planeSweeping/deviceFuse.hpp>

#include <iostream>
#include <sstream>
#include <stdexcept>

namespace aliceVision {
namespace depthMap {

void copy(CudaHostMemoryHeap<float2, 2>& outHmh, const StaticVector<DepthSim>& inDepthSimMap, int yFrom)
{
    const int w = outHmh.getSize()[0];
    const int h = outHmh.getSize()[1];
    for (int y = 0; y < h; ++y)
    {
        for (int x = 0; x < w; ++x)
        {
            int jO = (y + yFrom) * w + x;
            float2& h_data = outHmh(x, y);
            const DepthSim& data = inDepthSimMap[jO];
            h_data.x = data.depth;
            h_data.y = data.sim;
        }
    }
}

void copy(StaticVector<DepthSim>& outDepthSimMap, const CudaHostMemoryHeap<float2, 2>& inHmh, int yFrom)
{
    const int w = inHmh.getSize()[0];
    const int h = inHmh.getSize()[1];
    for (int y = 0; y < h; ++y)
    {
        for (int x = 0; x < w; ++x)
        {
            int jO = (y + yFrom) * w + x;
            DepthSim& oDepthSim = outDepthSimMap[jO];
            const float2& h_depthSim = inHmh(x, y);

            oDepthSim.depth = h_depthSim.x;
            oDepthSim.sim = h_depthSim.y;
        }
    }
}

PlaneSweepingCuda::PlaneSweepingCuda(mvsUtils::ImagesCache<ImageRGBAf>& ic,
                                     mvsUtils::MultiViewParams& mp)
    : _ic(ic)
    , _mp(mp)
{}

PlaneSweepingCuda::~PlaneSweepingCuda() {}

bool PlaneSweepingCuda::refineRcTcDepthMap(int rc, int tc, 
                                           StaticVector<float>& inout_depthMap,
                                           StaticVector<float>& out_simMap, 
                                           const RefineParams& refineParams, 
                                           int xFrom, int wPart)
{
    DeviceCache& deviceCache = DeviceCache::getInstance();

    const DeviceCamera& rcDeviceCamera = deviceCache.requestCamera(rc, refineParams.scale, _ic, _mp);
    const DeviceCamera& tcDeviceCamera = deviceCache.requestCamera(tc, refineParams.scale, _ic, _mp);

    cuda_refineDepthMap(rcDeviceCamera, 
                        tcDeviceCamera, 
                        inout_depthMap.getDataWritable().data(), 
                        out_simMap.getDataWritable().data(),
                        refineParams, 
                        xFrom, wPart);
    return true;
}

/* Be very careful with volume indexes:
 * volume is indexed with the same index as tc. The values of tc can be quite different.
 * depths is indexed with the index_set elements
 */
void PlaneSweepingCuda::computeDepthSimMapVolume(int rc,
                                                 CudaDeviceMemoryPitched<TSim, 3>& volBestSim_dmp,
                                                 CudaDeviceMemoryPitched<TSim, 3>& volSecBestSim_dmp,
                                                 const std::vector<int>& tCams,
                                                 const std::vector<Pixel>& rcDepthsTcamsLimits,
                                                 const std::vector<float>& rcDepths,
                                                 const SgmParams& sgmParams)
{
    const system::Timer timer;

    const CudaSize<3>& volDim = volBestSim_dmp.getSize();

    ALICEVISION_LOG_INFO("SGM Compute similarity volume (x: " << volDim.x() << ", y: " << volDim.y() << ", z: " << volDim.z() << ")");

    // initialize the two similarity volumes at 255
    cuda_volumeInitialize(volBestSim_dmp, 255.f, 0);
    cuda_volumeInitialize(volSecBestSim_dmp, 255.f, 0);

    ALICEVISION_LOG_DEBUG("Initialize output volumes: " << std::endl
                      << "\t- volBestSim_dmp : " << volBestSim_dmp.getUnitsInDim(0) << ", " << volBestSim_dmp.getUnitsInDim(1) << ", " << volBestSim_dmp.getUnitsInDim(2) << std::endl
                      << "\t- volSecBestSim_dmp : " << volSecBestSim_dmp.getUnitsInDim(0) << ", " << volSecBestSim_dmp.getUnitsInDim(1) << ", " << volSecBestSim_dmp.getUnitsInDim(2) << std::endl);

    // load rc & tc images in the CPU ImageCache
    _ic.getImg_sync(rc);
    for(int tc : tCams){ _ic.getImg_sync(tc); }
        
    // copy rc depth data in device memory
    CudaDeviceMemory<float> depths_d(rcDepths.data(), rcDepths.size());

    // log memory information
    logDeviceMemoryInfo();

    for(int tci = 0; tci < tCams.size(); ++tci)
    {
        const system::Timer timerPerTc;

        const int tc = tCams.at(tci);

        const int firstDepth = rcDepthsTcamsLimits.at(tci).x;
        const int lastDepth = firstDepth + rcDepthsTcamsLimits.at(tci).y;

        DeviceCache& deviceCache = DeviceCache::getInstance();

        const DeviceCamera& rcDeviceCamera = deviceCache.requestCamera(rc, sgmParams.scale, _ic, _mp, 0);
        const DeviceCamera& tcDeviceCamera = deviceCache.requestCamera(tc, sgmParams.scale, _ic, _mp, 0);

        const ROI roi(0, volBestSim_dmp.getSize().x(), 0, volBestSim_dmp.getSize().y(), firstDepth, lastDepth);

        ALICEVISION_LOG_DEBUG("Compute similarity volume:" << std::endl
                              << "\t- rc: " << rc << std::endl
                              << "\t- tc: " << tc << " (" << tci << "/" << tCams.size() << ")" << std::endl 
                              << "\t- rc camera device id: " << rcDeviceCamera.getDeviceCamId() << std::endl 
                              << "\t- tc camera device id: " << tcDeviceCamera.getDeviceCamId() << std::endl 
                              << "\t- tc first depth: " << firstDepth << std::endl
                              << "\t- tc last depth: " << lastDepth << std::endl
                              << "\t- rc width: " << rcDeviceCamera.getWidth() << std::endl
                              << "\t- rc height: " << rcDeviceCamera.getHeight() << std::endl
                              << "\t- device similarity volume size: " << volBestSim_dmp.getBytesPadded() / (1024.0 * 1024.0) << " MB" << std::endl
                              << "\t- device unpadded similarity volume size: " << volBestSim_dmp.getBytesUnpadded() / (1024.0 * 1024.0) << " MB" << std::endl);

        cuda_volumeComputeSimilarity(volBestSim_dmp,
                                     volSecBestSim_dmp, 
                                     depths_d,
                                     rcDeviceCamera,
                                     tcDeviceCamera,
                                     sgmParams, 
                                     roi,
                                     0 /*stream*/);

        ALICEVISION_LOG_DEBUG("Compute similarity volume (with tc: " << tc << ") done in: " << timerPerTc.elapsedMs() << " ms.");
    }
    ALICEVISION_LOG_INFO("SGM Compute similarity volume done in: " << timer.elapsedMs() << " ms.");
}


/**
 * @param[inout] volume input similarity volume
 */
bool PlaneSweepingCuda::sgmOptimizeSimVolume(int rc, 
                                             CudaDeviceMemoryPitched<TSim, 3>& volSimFiltered_dmp,
                                             const CudaDeviceMemoryPitched<TSim, 3>& volSim_dmp,
                                             const SgmParams& sgmParams)
{
    const system::Timer timer;

    const CudaSize<3>& volDim = volSim_dmp.getSize();

    ALICEVISION_LOG_INFO("SGM Optimizing volume:" << std::endl
                          << "\t- filtering axes: " << sgmParams.filteringAxes << std::endl
                          << "\t- volume dimensions: (x: " << volDim.x() << ", y: " << volDim.y() << ", z: " << volDim.z() << ")" << std::endl
                          << "\t- device similarity volume size: " << (double(volSim_dmp.getBytesPadded()) / (1024.0 * 1024.0)) << " MB" << std::endl);

    DeviceCache& deviceCache = DeviceCache::getInstance();

    const DeviceCamera& rcDeviceCamera = deviceCache.requestCamera(rc, sgmParams.scale, _ic, _mp);

    cuda_volumeOptimize(volSimFiltered_dmp, 
                        volSim_dmp, 
                        rcDeviceCamera, 
                        sgmParams, 
                        0 /*stream*/);

    ALICEVISION_LOG_INFO("SGM Optimizing volume done in: " << timer.elapsedMs() << " ms.");
    return true;
}

void PlaneSweepingCuda::sgmRetrieveBestDepth(int rc, 
                                             DepthSimMap& bestDepth,
                                             const CudaDeviceMemoryPitched<TSim, 3>& volSim_dmp, 
                                             const StaticVector<float>& rcDepths, 
                                             const SgmParams& sgmParams)
{
  const system::Timer timer;

  const CudaSize<3>& volDim = volSim_dmp.getSize();
  
  ALICEVISION_LOG_INFO("SGM Retrieve best depth in volume (x: " << volDim.x() << ", y: " << volDim.y() << ", z: " << volDim.z() << ")");

  DeviceCache& deviceCache = DeviceCache::getInstance();

  const DeviceCamera& rcDeviceCamera = deviceCache.requestCamera(rc, 1, _ic, _mp);

  const CudaSize<2> depthSimDim(volDim.x(), volDim.y());

  CudaDeviceMemory<float> depths_d(rcDepths.getData().data(), rcDepths.size());
  CudaDeviceMemoryPitched<float, 2> bestDepth_dmp(depthSimDim);
  CudaDeviceMemoryPitched<float, 2> bestSim_dmp(depthSimDim);

  const ROI roi(0, volSim_dmp.getSize().x(), 0, volSim_dmp.getSize().y(), 0, volSim_dmp.getSize().z());

  cuda_volumeRetrieveBestDepth(bestDepth_dmp,
                               bestSim_dmp, 
                               volSim_dmp, 
                               depths_d, 
                               rcDeviceCamera,
                               sgmParams,
                               roi, 
                               0 /*stream*/);

  CudaHostMemoryHeap<float, 2> bestDepth_hmh(depthSimDim);
  bestDepth_hmh.copyFrom(bestDepth_dmp);
  bestDepth_dmp.deallocate();

  CudaHostMemoryHeap<float, 2> bestSim_hmh(depthSimDim);
  bestSim_hmh.copyFrom(bestSim_dmp);
  bestSim_dmp.deallocate();

  for(int y = 0; y < depthSimDim.y(); ++y)
  {
    for(int x = 0; x < depthSimDim.x(); ++x)
    {
      DepthSim& out = bestDepth._dsm[y * depthSimDim.x() + x];
      out.depth = bestDepth_hmh(x, y);
      out.sim = bestSim_hmh(x, y);
    }
  }

  ALICEVISION_LOG_INFO("SGM Retrieve best depth in volume done in: " << timer.elapsedMs() << " ms.");
}

bool PlaneSweepingCuda::fuseDepthSimMapsGaussianKernelVoting(int wPart, int hPart, 
                                                             StaticVector<DepthSim>& out_depthSimMap,
                                                             const StaticVector<StaticVector<DepthSim>*>& dataMaps,
                                                             const RefineParams& refineParams)
{
    const system::Timer timer;
    const CudaSize<2> depthSimMapPartDim(wPart, hPart);

    std::vector<CudaHostMemoryHeap<float2, 2>*> dataMaps_hmh(dataMaps.size());
    for(int i = 0; i < dataMaps.size(); i++)
    {
        dataMaps_hmh[i] = new CudaHostMemoryHeap<float2, 2>(depthSimMapPartDim);
        for(int y = 0; y < hPart; ++y)
        {
            for(int x = 0; x < wPart; ++x)
            {
                float2& data_hmh = (*dataMaps_hmh[i])(x, y);
                const DepthSim& data = (*dataMaps[i])[y * wPart + x];
                data_hmh.x = data.depth;
                data_hmh.y = data.sim;
            }
        }
    }

    CudaHostMemoryHeap<float2, 2> depthSimMap_hmh(depthSimMapPartDim);

    cuda_fuseDepthSimMapsGaussianKernelVoting(wPart, hPart, 
                                              &depthSimMap_hmh, 
                                              dataMaps_hmh, dataMaps.size(), 
                                              refineParams);
    for(int y = 0; y < hPart; ++y)
    {
        for(int x = 0; x < wPart; ++x)
        {
            const float2& depthSim_hmh = depthSimMap_hmh(x, y);
            DepthSim& out_depthSim = out_depthSimMap[y * wPart + x];
            out_depthSim.depth = depthSim_hmh.x;
            out_depthSim.sim = depthSim_hmh.y;
        }
    }

    for(int i = 0; i < dataMaps.size(); ++i)
    {
        delete dataMaps_hmh[i];
    }

    ALICEVISION_LOG_DEBUG("Fuse depth/sim maps gaussian kernel voting done in: " << timer.elapsedMs() << " ms.");

    return true;
}

bool PlaneSweepingCuda::optimizeDepthSimMapGradientDescent(int rc, 
                                                           StaticVector<DepthSim>& out_depthSimMapOptimized,
                                                           const StaticVector<DepthSim>& depthSimMapSgmUpscale,
                                                           const StaticVector<DepthSim>& depthSimMapRefinedFused,
                                                           const RefineParams& refineParams,
                                                           int yFrom, int hPart)
{
    const system::Timer timer;

    DeviceCache& deviceCache = DeviceCache::getInstance();
    const DeviceCamera& rcDeviceCamera = deviceCache.requestCamera(rc, refineParams.scale, _ic, _mp);

    const CudaSize<2> depthSimMapPartDim(size_t(_mp.getWidth(rc) / refineParams.scale), size_t(hPart));

    CudaHostMemoryHeap<float2, 2> sgmDepthPixSizeMap_hmh(depthSimMapPartDim);
    CudaHostMemoryHeap<float2, 2> refinedDepthSimMap_hmh(depthSimMapPartDim);

    copy(sgmDepthPixSizeMap_hmh, depthSimMapSgmUpscale, yFrom);
    copy(refinedDepthSimMap_hmh, depthSimMapRefinedFused, yFrom);

    CudaHostMemoryHeap<float2, 2> optimizedDepthSimMap_hmh(depthSimMapPartDim);

    cuda_optimizeDepthSimMapGradientDescent(rcDeviceCamera,
                                            optimizedDepthSimMap_hmh,
                                            sgmDepthPixSizeMap_hmh, 
                                            refinedDepthSimMap_hmh, 
                                            depthSimMapPartDim,
                                            refineParams,
                                            yFrom);

    copy(out_depthSimMapOptimized, optimizedDepthSimMap_hmh, yFrom);

    ALICEVISION_LOG_DEBUG("Optimize depth/sim map gradient descent done in: " << timer.elapsedMs() << " ms.");

    return true;
}

DeviceNormalMapper* PlaneSweepingCuda::createNormalMapping()
{
    return new DeviceNormalMapper;
}

void PlaneSweepingCuda::deleteNormalMapping(DeviceNormalMapper* m)
{
    delete m;
}

bool PlaneSweepingCuda::computeNormalMap(
    DeviceNormalMapper*            mapping,
    const std::vector<float>& depthMap,
    std::vector<ColorRGBf>&   normalMap,
    int rc, int scale,
    float igammaC, float igammaP, int wsh)
{
  const int w = _mp.getWidth(rc) / scale;
  const int h = _mp.getHeight(rc) / scale;

  const long t1 = clock();

  ALICEVISION_LOG_DEBUG("computeNormalMap rc: " << rc);

  // Fill Camera Struct

  fillHostCameraParameters(*mapping->cameraParameters_h, rc, scale, _mp);
  mapping->loadCameraParameters();
  mapping->allocHostMaps(w, h);
  mapping->copyDepthMap(depthMap);

  cuda_computeNormalMap(mapping, w, h, wsh, igammaC, igammaP);

  float3* normalMapPtr = mapping->getNormalMapHst();

  constexpr bool q = ( sizeof(ColorRGBf[2]) == sizeof(float3[2]) );
  if( q == true )
  {
    memcpy( normalMap.data(), mapping->getNormalMapHst(), w*h*sizeof(float3) );
  }
  else
  {
    for (int i = 0; i < w * h; i++)
    {
        normalMap[i].r = normalMapPtr[i].x;
        normalMap[i].g = normalMapPtr[i].y;
        normalMap[i].b = normalMapPtr[i].z;
    }
  }

  if (_mp.verbose)
    mvsUtils::printfElapsedTime(t1);

  return true;
}

} // namespace depthMap
} // namespace aliceVision
