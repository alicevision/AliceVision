// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "PlaneSweepingCuda.hpp"
#include <aliceVision/depthMap/volumeIO.hpp>

#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/Timer.hpp>
#include <aliceVision/system/nvtx.hpp>
#include <aliceVision/mvsData/Matrix3x3.hpp>
#include <aliceVision/mvsData/Matrix3x4.hpp>
#include <aliceVision/mvsData/OrientedPoint.hpp>
#include <aliceVision/mvsUtils/common.hpp>
#include <aliceVision/mvsUtils/fileIO.hpp>
#include <aliceVision/depthMap/cuda/planeSweeping/plane_sweeping_cuda.hpp>
#include <aliceVision/depthMap/cuda/normalmap/normal_map.hpp>
#include <aliceVision/depthMap/cuda/planeSweeping/host_utils.h>
#include <aliceVision/depthMap/cuda/images/gauss_filter.hpp>

#include <iostream>
#include <sstream>
#include <stdexcept>

namespace aliceVision {
namespace depthMap {

static void cps_host_fillCamera(CameraStructBase& base, int c, mvsUtils::MultiViewParams& mp, int scale )
{

    Matrix3x3 scaleM;
    scaleM.m11 = 1.0 / (float)scale;
    scaleM.m12 = 0.0;
    scaleM.m13 = 0.0;
    scaleM.m21 = 0.0;
    scaleM.m22 = 1.0 / (float)scale;
    scaleM.m23 = 0.0;
    scaleM.m31 = 0.0;
    scaleM.m32 = 0.0;
    scaleM.m33 = 1.0;
    Matrix3x3 K = scaleM * mp.KArr[c];

    Matrix3x3 iK = K.inverse();
    Matrix3x4 P = K * (mp.RArr[c] | (Point3d(0.0, 0.0, 0.0) - mp.RArr[c] * mp.CArr[c]));
    Matrix3x3 iP = mp.iRArr[c] * iK;

    base.C.x = mp.CArr[c].x;
    base.C.y = mp.CArr[c].y;
    base.C.z = mp.CArr[c].z;

    base.P[0] = P.m11;
    base.P[1] = P.m21;
    base.P[2] = P.m31;
    base.P[3] = P.m12;
    base.P[4] = P.m22;
    base.P[5] = P.m32;
    base.P[6] = P.m13;
    base.P[7] = P.m23;
    base.P[8] = P.m33;
    base.P[9] = P.m14;
    base.P[10] = P.m24;
    base.P[11] = P.m34;

    base.iP[0] = iP.m11;
    base.iP[1] = iP.m21;
    base.iP[2] = iP.m31;
    base.iP[3] = iP.m12;
    base.iP[4] = iP.m22;
    base.iP[5] = iP.m32;
    base.iP[6] = iP.m13;
    base.iP[7] = iP.m23;
    base.iP[8] = iP.m33;

    base.R[0] = mp.RArr[c].m11;
    base.R[1] = mp.RArr[c].m21;
    base.R[2] = mp.RArr[c].m31;
    base.R[3] = mp.RArr[c].m12;
    base.R[4] = mp.RArr[c].m22;
    base.R[5] = mp.RArr[c].m32;
    base.R[6] = mp.RArr[c].m13;
    base.R[7] = mp.RArr[c].m23;
    base.R[8] = mp.RArr[c].m33;

    base.iR[0] = mp.iRArr[c].m11;
    base.iR[1] = mp.iRArr[c].m21;
    base.iR[2] = mp.iRArr[c].m31;
    base.iR[3] = mp.iRArr[c].m12;
    base.iR[4] = mp.iRArr[c].m22;
    base.iR[5] = mp.iRArr[c].m32;
    base.iR[6] = mp.iRArr[c].m13;
    base.iR[7] = mp.iRArr[c].m23;
    base.iR[8] = mp.iRArr[c].m33;

    base.K[0] = K.m11;
    base.K[1] = K.m21;
    base.K[2] = K.m31;
    base.K[3] = K.m12;
    base.K[4] = K.m22;
    base.K[5] = K.m32;
    base.K[6] = K.m13;
    base.K[7] = K.m23;
    base.K[8] = K.m33;

    base.iK[0] = iK.m11;
    base.iK[1] = iK.m21;
    base.iK[2] = iK.m31;
    base.iK[3] = iK.m12;
    base.iK[4] = iK.m22;
    base.iK[5] = iK.m32;
    base.iK[6] = iK.m13;
    base.iK[7] = iK.m23;
    base.iK[8] = iK.m33;

    ps_initCameraMatrix( base );
}


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

int listCUDADevices(bool verbose)
{
    return ps_listCUDADevices(verbose);
}

/*********************************************************************************
 * CamSelection
 *********************************************************************************/

bool operator==(const CamSelection& l, const CamSelection& r)
{
    return (l.first == r.first && l.second == r.second);
}

bool operator<(const CamSelection& l, const CamSelection& r)
{
    return (l.first < r.first || (l.first == r.first && l.second < r.second));
}

/*********************************************************************************
 * PlaneSweepingCuda
 *********************************************************************************/

PlaneSweepingCuda::PlaneSweepingCuda( int CUDADeviceNo,
                                      mvsUtils::ImagesCache<ImageRGBAf>&     ic,
                                      mvsUtils::MultiViewParams& mp,
                                      int scales )
    : _scales( scales )
    , _CUDADeviceNo( CUDADeviceNo )
    , _ic( ic )
    , _mp(mp)
    , _cameraParamCache( MAX_CONSTANT_CAMERA_PARAM_SETS )
{
    /* The caller knows all camera that will become rc cameras, but it does not
     * pass that information to this function.
     * It knows the nearest cameras for each of those rc cameras, but it doesn't
     * pass that information, either.
     * So, the only task of this function is to allocate an amount of memory that
     * will hold CUDA memory for camera structs and bitmaps.
     */

    ps_testCUDAdeviceNo( _CUDADeviceNo );

    _nImgsInGPUAtTime = imagesInGPUAtTime( mp, scales );

    // allocate global on the device
    _hidden.reset(new FrameCacheMemory( _nImgsInGPUAtTime,
                                    mp.getMaxImageWidth(),
                                    mp.getMaxImageHeight(),
                                    scales,
                                    _CUDADeviceNo));


    ALICEVISION_LOG_INFO("PlaneSweepingCuda:" << std::endl
                         << "\t- _nImgsInGPUAtTime: " << _nImgsInGPUAtTime << std::endl
                         << "\t- scales: " << _scales);

    cudaError_t err;

    err = cudaMallocHost(&_camsBasesHst, MAX_CONSTANT_CAMERA_PARAM_SETS * sizeof(CameraStructBase));
    THROW_ON_CUDA_ERROR( err, "Could not allocate set of camera structs in pinned host memory in " << __FILE__ << ":" << __LINE__ << ", " << cudaGetErrorString(err) );

    _cams    .resize(_nImgsInGPUAtTime);
    _camsHost.resize(_nImgsInGPUAtTime);

    for( int rc = 0; rc < _nImgsInGPUAtTime; ++rc )
    {
        _cams[rc].camId = -1;
        _cams[rc].param_dev.i = rc;
        _cams[rc].pyramid   = _hidden->getPyramidPtr(rc); // &_hidden_pyramids[rc];

        err = cudaStreamCreate( &_cams[rc].stream );
        if( err != cudaSuccess )
        {
            ALICEVISION_LOG_WARNING("Failed to create a CUDA stream object for async sweeping");
            _cams[rc].stream = 0;
        }
    }
}

PlaneSweepingCuda::~PlaneSweepingCuda()
{
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // deallocate global on the device

    cudaFreeHost( _camsBasesHst );

    for(int c = 0; c < _cams.size(); c++)
    {
        cudaStreamDestroy( _cams[c].stream );
    }
}

/* static private function called by the constructor */
int PlaneSweepingCuda::imagesInGPUAtTime( mvsUtils::MultiViewParams& mp, int scales )
{
    int value;

    const int maxImageWidth = mp.getMaxImageWidth();
    const int maxImageHeight = mp.getMaxImageHeight();

    float oneimagemb = 4.0f * sizeof(float) * (((float)(maxImageWidth * maxImageHeight) / 1024.0f) / 1024.0f);
    for(int scale = 2; scale <= scales; ++scale)
    {
        oneimagemb += 4.0 * sizeof(float) * (((float)((maxImageWidth / scale) * (maxImageHeight / scale)) / 1024.0) / 1024.0);
    }
    float maxmbGPU = 400.0f; // TODO FACA

    value = (int)(maxmbGPU / oneimagemb);
    value = std::max(2, std::min(mp.ncams, value));

    if( value > MAX_CONSTANT_CAMERA_PARAM_SETS )
    {
        ALICEVISION_LOG_WARNING( "DepthMap has been compiled with a hard limit of "
                                 << MAX_CONSTANT_CAMERA_PARAM_SETS
                                 << " concurrent images. "<< std::endl
                                 << "Recompilation required for larger values." << std::endl
                                 << "Change define MAX_CONSTANT_CAMERA_PARAM_SETS "
                                 << " but consider hardware limits for CUDA constant memory." );
        value = MAX_CONSTANT_CAMERA_PARAM_SETS;
    }

    return value;
}

CamCacheIdx PlaneSweepingCuda::loadCameraParam( int global_cam_id, int scale, cudaStream_t stream )
{
    CamSelection newP( global_cam_id, scale );
    CamCacheIdx newPIndex;

    bool newCamParam = _cameraParamCache.insert( newP, &newPIndex.i );
    if( newCamParam )
    {
        cps_host_fillCamera(_camsBasesHst[newPIndex.i], global_cam_id, _mp, scale);
        ps_loadCameraStructs( _camsBasesHst, newPIndex, stream );
    }

    return newPIndex;
}

int PlaneSweepingCuda::addCam( int global_cam_id, int scale, cudaStream_t stream )
{
    // first is oldest
    int local_frame_id;
    bool newInsertion = _camsHost.insert( global_cam_id, &local_frame_id );

    CameraStruct& cam = _cams[local_frame_id];

    if( newInsertion )
    {
        cam.camId = local_frame_id;

        long t1 = clock();

        /* Fill slot id in the GPU-sided frame cache from the global image cache */
        _hidden->fillFrame( local_frame_id, global_cam_id, _ic, _mp, stream );

        mvsUtils::printfElapsedTime(t1, "Copy image (camera id="+std::to_string(global_cam_id)+") from CPU to GPU");
    }

    /* Fetch slot in constant memory that contains the camera parameters,
     * and fill it needed. */
    cam.param_dev = loadCameraParam( global_cam_id, scale, stream );

    _hidden->setLocalCamId( local_frame_id, cam.param_dev.i );

    if( _cams[local_frame_id].camId != local_frame_id )
    {
        std::cerr << "BUG in " << __FILE__ << ":" << __LINE__ << " ?"
                  << " The camId member should be initialized with the return value of addCam()."
                  << std::endl;
        exit( -1 );
    }

    return local_frame_id;
}

bool PlaneSweepingCuda::refineRcTcDepthMap(int rc, int tc, 
                                           StaticVector<float>& inout_depthMap,
                                           StaticVector<float>& out_simMap, 
                                           const RefineParams& refineParams, 
                                           int xFrom, int wPart)
{
    const int rcWidth = _mp.getWidth(rc) / refineParams.scale;
    const int rcHeight = _mp.getHeight(rc) / refineParams.scale;

    const int tcWidth = _mp.getWidth(tc) / refineParams.scale;
    const int tcHeight = _mp.getHeight(tc) / refineParams.scale;

    const int rcFrameCacheId = addCam(rc, refineParams.scale);
    const int tcFrameCacheId = addCam(tc, refineParams.scale);

    const CameraStruct& rcam = _cams[rcFrameCacheId];
    const CameraStruct& tcam = _cams[tcFrameCacheId];

    ps_refineRcDepthMap(rcam, tcam, 
                        inout_depthMap.getDataWritable().data(), 
                        out_simMap.getDataWritable().data(),
                        rcWidth, rcHeight, 
                        tcWidth, tcHeight, 
                        refineParams, 
                        xFrom, wPart, _CUDADeviceNo);
    return true;
}

/* Be very careful with volume indexes:
 * volume is indexed with the same index as tc. The values of tc can be quite different.
 * depths is indexed with the index_set elements
 */
void PlaneSweepingCuda::computeDepthSimMapVolume(int rc,
                                                 CudaDeviceMemoryPitched<TSim, 3>& volBestSim_dmp,
                                                 CudaDeviceMemoryPitched<TSim, 3>& volSecBestSim_dmp,
                                                 const CudaSize<3>& volDim,
                                                 const std::vector<int>& tCams,
                                                 const std::vector<Pixel>& rcDepthsTcamsLimits,
                                                 const std::vector<float>& rcDepths,
                                                 const SgmParams& sgmParams)
{
    const system::Timer timer;

    ALICEVISION_LOG_INFO("SGM Compute similarity volume (x: " << volDim.x() << ", y: " << volDim.y() << ", z: " << volDim.z() << ")");

    std::vector<OneTC> tcs;
    tcs.reserve(rcDepthsTcamsLimits.size());

    for(std::size_t i = 0; i < rcDepthsTcamsLimits.size(); ++i)
    {
        tcs.emplace_back(tCams[i], rcDepthsTcamsLimits[i].x, rcDepthsTcamsLimits[i].y);
    }

    nvtxPush("preload host cache ");
    _ic.getImg_sync(rc);
    for( const auto& tc : tcs) _ic.getImg_sync( tc.getTCIndex() );
    nvtxPop("preload host cache ");

    ps::SimilarityVolume vol(volDim, sgmParams.stepXY, sgmParams.scale, rcDepths);

    vol.initOutputVolumes(volBestSim_dmp, volSecBestSim_dmp, 0);
    vol.WaitSweepStream(0);

    ALICEVISION_LOG_DEBUG("Initialize output volumes: " << std::endl
                          << "\t- volBestSim_dmp : " << volBestSim_dmp.getUnitsInDim(0) << ", " << volBestSim_dmp.getUnitsInDim(1) << ", " << volBestSim_dmp.getUnitsInDim(2) << std::endl
                          << "\t- volSecBestSim_dmp : " << volSecBestSim_dmp.getUnitsInDim(0) << ", " << volSecBestSim_dmp.getUnitsInDim(1) << ", " << volSecBestSim_dmp.getUnitsInDim(2) << std::endl
                          << "\t- scale: " << vol.scale() << std::endl
                          << "\t- volStepXY: " << vol.stepXY() << std::endl);

    for(int tci = 0; tci < tcs.size(); ++tci)
    {
        vol.WaitSweepStream(tci);
        cudaStream_t stream = vol.SweepStream(tci);

        const system::Timer timerPerTc;

        const int tc = tcs[tci].getTCIndex();

        const int rcWidth = _mp.getWidth(rc);
        const int rcHeight = _mp.getHeight(rc);

        const int tcWidth = _mp.getWidth(tc);
        const int tcHeight = _mp.getHeight(tc);

        const int rcFrameCacheId = addCam(rc, vol.scale(), stream);
        const int tcFrameCacheId = addCam(tc, vol.scale(), stream);

        const CameraStruct& rcam = _cams[rcFrameCacheId];
        const CameraStruct& tcam = _cams[tcFrameCacheId];

        const auto deviceMemoryInfo = getDeviceMemoryInfo();

        ALICEVISION_LOG_DEBUG("Compute similarity volume:" << std::endl
                              << "\t- rc: " << rc << std::endl
                              << "\t- tc: " << tc << " (" << tci << "/" << tcs.size() << ")" << std::endl 
                              << "\t- rc frame cache id: " << rcFrameCacheId << std::endl 
                              << "\t- tc frame cache id: " << tcFrameCacheId << std::endl 
                              << "\t- tc depth to start: " << tcs[tci].getDepthToStart() << std::endl
                              << "\t- tc depths to search: " << tcs[tci].getDepthsToSearch() << std::endl
                              << "\t- device similarity volume size: " << volBestSim_dmp.getBytesPadded() / (1024.0 * 1024.0) << " MB" << std::endl
                              << "\t- device unpadded similarity volume size: " << volBestSim_dmp.getBytesUnpadded() / (1024.0 * 1024.0) << " MB" << std::endl
                              << "\t- device memory available: " << deviceMemoryInfo.x << "MB, total: " << deviceMemoryInfo.y << " MB" << std::endl);

        // last synchronous step
        // cudaDeviceSynchronize();
        vol.compute(
            volBestSim_dmp,
            volSecBestSim_dmp,
            rcam, rcWidth, rcHeight,
            tcam, tcWidth, tcHeight,
            tcs[tci],
            sgmParams,
            tci);

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
                                             const CudaSize<3>& volDim, 
                                             const SgmParams& sgmParams)
{
    const system::Timer timer;

    ALICEVISION_LOG_INFO("SGM Optimizing volume:" << std::endl
                          << "\t- filtering axes: " << sgmParams.filteringAxes << std::endl
                          << "\t- volume dimensions: (x: " << volDim.x() << ", y: " << volDim.y() << ", z: " << volDim.z() << ")" << std::endl
                          << "\t- device similarity volume size: " << (double(volSim_dmp.getBytesPadded()) / (1024.0 * 1024.0)) << " MB" << std::endl);

    const int rcFrameCacheId = addCam(rc, sgmParams.scale);

    // update aggregation volume
    int npaths = 0;
    const Pyramid& rcPyramid = *(_cams[rcFrameCacheId].pyramid);
    const size_t rcPyramidScaleIndex = size_t(sgmParams.scale) - 1;
    cudaTextureObject_t rc_tex = rcPyramid[rcPyramidScaleIndex].tex;

    const auto updateAggrVolume = [&](const CudaSize<3>& axisT, bool invX) 
    {
        ALICEVISION_LOG_DEBUG("Update aggregate volume (npaths: " << npaths << ", invX: " << invX << ")");

        ps_aggregatePathVolume(volSimFiltered_dmp, 
                               volSim_dmp, 
                               volDim, 
                               axisT, rc_tex, 
                               sgmParams, 
                               invX, npaths);
        npaths++;

        ALICEVISION_LOG_DEBUG("Update aggregate volume done.");
    };

    // filtering is done on the last axis
    const std::map<char, CudaSize<3>> mapAxes = {
        {'X', {1, 0, 2}}, // XYZ -> YXZ
        {'Y', {0, 1, 2}}, // XYZ
    };

    for(char axis : sgmParams.filteringAxes)
    {
        const CudaSize<3>& axisT = mapAxes.at(axis);
        updateAggrVolume(axisT, false); // without transpose
        updateAggrVolume(axisT, true);  // with transpose of the last axis
    }

    ALICEVISION_LOG_INFO("SGM Optimizing volume done in: " << timer.elapsedMs() << " ms.");
    return true;
}

void PlaneSweepingCuda::sgmRetrieveBestDepth(int rc, 
                                             DepthSimMap& bestDepth,
                                             const CudaDeviceMemoryPitched<TSim, 3>& volSim_dmp, 
                                             const CudaSize<3>& volDim,
                                             const StaticVector<float>& rcDepths, 
                                             const SgmParams& sgmParams)
{
  const system::Timer timer;
  
  ALICEVISION_LOG_INFO("SGM Retrieve best depth in volume (x: " << volDim.x() << ", y: " << volDim.y() << ", z: " << volDim.z() << ")");

  const int rcFrameCacheId = addCam(rc, 1);
  const int rcamCacheId = _hidden->getLocalCamId(rcFrameCacheId);
  const CudaSize<2> depthSimDim(volDim.x(), volDim.y());

  CudaDeviceMemory<float> depths_d(rcDepths.getData().data(), rcDepths.size());
  CudaDeviceMemoryPitched<float, 2> bestDepth_dmp(depthSimDim);
  CudaDeviceMemoryPitched<float, 2> bestSim_dmp(depthSimDim);

  const int scaleStep = sgmParams.scale * sgmParams.stepXY;

  ps_SGMretrieveBestDepth(
    rcamCacheId,
    bestDepth_dmp,
    bestSim_dmp, 
    volSim_dmp, 
    volDim,
    depths_d,
    scaleStep,
    sgmParams.interpolateRetrieveBestDepth);

  /*
  {
      CudaTexture<float> bestDepth_tex(bestDepth_dmp);
      ps_medianFilter3(bestDepth_tex.textureObj, bestDepth_dmp);
  }
  */

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

// make_float3(avail,total,used)
Point3d PlaneSweepingCuda::getDeviceMemoryInfo()
{
    size_t iavail;
    size_t itotal;

    cudaMemGetInfo(&iavail, &itotal);

    const double avail = double(iavail) / (1024.0 * 1024.0);
    const double total = double(itotal) / (1024.0 * 1024.0);
    const double used = double(itotal - iavail) / (1024.0 * 1024.0);

    return Point3d(avail, total, used);
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

    ps_fuseDepthSimMapsGaussianKernelVoting(wPart, hPart, 
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

    const CudaSize<2> depthSimMapPartDim(size_t(_mp.getWidth(rc) / refineParams.scale), size_t(hPart));

    const int rcFrameCacheId = addCam(rc, refineParams.scale);
    const CameraStruct& rcam = _cams[rcFrameCacheId];

    CudaHostMemoryHeap<float2, 2> sgmDepthPixSizeMap_hmh(depthSimMapPartDim);
    CudaHostMemoryHeap<float2, 2> refinedDepthSimMap_hmh(depthSimMapPartDim);

    copy(sgmDepthPixSizeMap_hmh, depthSimMapSgmUpscale, yFrom);
    copy(refinedDepthSimMap_hmh, depthSimMapRefinedFused, yFrom);

    CudaHostMemoryHeap<float2, 2> optimizedDepthSimMap_hmh(depthSimMapPartDim);

    ps_optimizeDepthSimMapGradientDescent(rcam,
                                          optimizedDepthSimMap_hmh,
                                          sgmDepthPixSizeMap_hmh, 
                                          refinedDepthSimMap_hmh, 
                                          depthSimMapPartDim,
                                          refineParams,
                                          _CUDADeviceNo, _nImgsInGPUAtTime, yFrom);

    copy(out_depthSimMapOptimized, optimizedDepthSimMap_hmh, yFrom);

    ALICEVISION_LOG_DEBUG("Optimize depth/sim map gradient descent done in: " << timer.elapsedMs() << " ms.");

    return true;
}

NormalMapping* PlaneSweepingCuda::createNormalMapping()
{
    return new NormalMapping;
}

void PlaneSweepingCuda::deleteNormalMapping( NormalMapping* m )
{
    delete m;
}

bool PlaneSweepingCuda::computeNormalMap(
    NormalMapping*            mapping,
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

  cps_host_fillCamera( *mapping->camsBasesHst, rc, _mp, scale );
  mapping->loadCameraParameters();
  mapping->allocHostMaps( w, h );
  mapping->copyDepthMap( depthMap );

  ps_computeNormalMap( mapping,
                       w, h, scale - 1,
                       _nImgsInGPUAtTime,
                       _scales, wsh, _mp.verbose, igammaC, igammaP);

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

bool PlaneSweepingCuda::getSilhoueteMap(StaticVectorBool* oMap, int scale, int step, const rgb maskColor, int rc)
{
    ALICEVISION_LOG_DEBUG("getSilhoueteeMap: rc: " << rc);

    int w = _mp.getWidth(rc) / scale;
    int h = _mp.getHeight(rc) / scale;

    long t1 = clock();

    int camId = addCam(rc, scale );
    CameraStruct& cam = _cams[camId];

    uchar4 maskColorRgb;
    maskColorRgb.x = maskColor.r;
    maskColorRgb.y = maskColor.g;
    maskColorRgb.z = maskColor.b;
    maskColorRgb.w = 1.0f;

    CudaHostMemoryHeap<bool, 2> omap_hmh(CudaSize<2>(w / step, h / step));

    ps_getSilhoueteMap( &omap_hmh, w, h, scale - 1,
                        step,
                        cam,
                        maskColorRgb, _mp.verbose );

    for(int i = 0; i < (w / step) * (h / step); i++)
    {
        (*oMap)[i] = omap_hmh.getBuffer()[i];
    }

    mvsUtils::printfElapsedTime(t1);

    return true;
}

} // namespace depthMap
} // namespace aliceVision
