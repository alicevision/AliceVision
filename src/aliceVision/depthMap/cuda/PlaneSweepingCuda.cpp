// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "PlaneSweepingCuda.hpp"
#include <aliceVision/depthMap/volumeIO.hpp>

#include <aliceVision/system/Logger.hpp>
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
void PlaneSweepingCuda::logCamerasRcTc( int rc, const StaticVector<int>& tcams )
{
    std::ostringstream ostr;

    ostr << "Called " << __FUNCTION__ << " with cameras:" << std::endl
         << "    rc = " << rc << ", tc = [";
    for( auto it : tcams )
    {
        ostr << it << ", ";
    }
    ostr << "]" << std::endl;

    ALICEVISION_LOG_DEBUG( ostr.str() );
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

void PlaneSweepingCuda::getMinMaxdepths(int rc, const StaticVector<int>& tcams, float& minDepth, float& midDepth,
                                          float& maxDepth)
{
  const bool minMaxDepthDontUseSeeds = _mp.userParams.get<bool>("prematching.minMaxDepthDontUseSeeds", false);
  const float maxDepthScale = static_cast<float>(_mp.userParams.get<double>("prematching.maxDepthScale", 1.5f));

  if(minMaxDepthDontUseSeeds)
  {
    const float minCamDist = static_cast<float>(_mp.userParams.get<double>("prematching.minCamDist", 0.0f));
    const float maxCamDist = static_cast<float>(_mp.userParams.get<double>("prematching.maxCamDist", 15.0f));

    minDepth = 0.0f;
    maxDepth = 0.0f;
    for(int c = 0; c < tcams.size(); c++)
    {
        int tc = tcams[c];
        minDepth += (_mp.CArr[rc] - _mp.CArr[tc]).size() * minCamDist;
        maxDepth += (_mp.CArr[rc] - _mp.CArr[tc]).size() * maxCamDist;
    }
    minDepth /= static_cast<float>(tcams.size());
    maxDepth /= static_cast<float>(tcams.size());
    midDepth = (minDepth + maxDepth) / 2.0f;
  }
  else
  {
    std::size_t nbDepths;
    _mp.getMinMaxMidNbDepth(rc, minDepth, maxDepth, midDepth, nbDepths);
    maxDepth = maxDepth * maxDepthScale;
  }
}

StaticVector<float>* PlaneSweepingCuda::getDepthsByPixelSize(int rc, float minDepth, float midDepth, float maxDepth,
                                                               int scale, int step, int maxDepthsHalf)
{
    float d = (float)step;

    OrientedPoint rcplane;
    rcplane.p = _mp.CArr[rc];
    rcplane.n = _mp.iRArr[rc] * Point3d(0.0, 0.0, 1.0);
    rcplane.n = rcplane.n.normalize();

    int ndepthsMidMax = 0;
    float maxdepth = midDepth;
    while((maxdepth < maxDepth) && (ndepthsMidMax < maxDepthsHalf))
    {
        Point3d p = rcplane.p + rcplane.n * maxdepth;
        float pixSize = _mp.getCamPixelSize(p, rc, (float)scale * d);
        maxdepth += pixSize;
        ndepthsMidMax++;
    }

    int ndepthsMidMin = 0;
    float mindepth = midDepth;
    while((mindepth > minDepth) && (ndepthsMidMin < maxDepthsHalf * 2 - ndepthsMidMax))
    {
        Point3d p = rcplane.p + rcplane.n * mindepth;
        float pixSize = _mp.getCamPixelSize(p, rc, (float)scale * d);
        mindepth -= pixSize;
        ndepthsMidMin++;
    }

    // getNumberOfDepths
    float depth = mindepth;
    int ndepths = 0;
    float pixSize = 1.0f;
    while((depth < maxdepth) && (pixSize > 0.0f) && (ndepths < 2 * maxDepthsHalf))
    {
        Point3d p = rcplane.p + rcplane.n * depth;
        pixSize = _mp.getCamPixelSize(p, rc, (float)scale * d);
        depth += pixSize;
        ndepths++;
    }

    StaticVector<float>* out = new StaticVector<float>();
    out->reserve(ndepths);

    // fill
    depth = mindepth;
    pixSize = 1.0f;
    ndepths = 0;
    while((depth < maxdepth) && (pixSize > 0.0f) && (ndepths < 2 * maxDepthsHalf))
    {
        out->push_back(depth);
        Point3d p = rcplane.p + rcplane.n * depth;
        pixSize = _mp.getCamPixelSize(p, rc, (float)scale * d);
        depth += pixSize;
        ndepths++;
    }

    // check if it is asc
    for(int i = 0; i < out->size() - 1; i++)
    {
        if((*out)[i] >= (*out)[i + 1])
        {

            for(int j = 0; j <= i + 1; j++)
            {
                ALICEVISION_LOG_TRACE("getDepthsByPixelSize: check if it is asc: " << (*out)[j]);
            }
            throw std::runtime_error("getDepthsByPixelSize not asc.");
        }
    }

    return out;
}

StaticVector<float>* PlaneSweepingCuda::getDepthsRcTc(int rc, int tc, int scale, float midDepth,
                                                        int maxDepthsHalf)
{
    OrientedPoint rcplane;
    rcplane.p = _mp.CArr[rc];
    rcplane.n = _mp.iRArr[rc] * Point3d(0.0, 0.0, 1.0);
    rcplane.n = rcplane.n.normalize();

    Point2d rmid = Point2d((float)_mp.getWidth(rc) / 2.0f, (float)_mp.getHeight(rc) / 2.0f);
    Point2d pFromTar, pToTar; // segment of epipolar line of the principal point of the rc camera to the tc camera
    getTarEpipolarDirectedLine(&pFromTar, &pToTar, rmid, rc, tc, _mp);

    int allDepths = static_cast<int>((pToTar - pFromTar).size());
    ALICEVISION_LOG_DEBUG("allDepths: " << allDepths);

    Point2d pixelVect = ((pToTar - pFromTar).normalize()) * std::max(1.0f, (float)scale);
    // printf("%f %f %i %i\n",pixelVect.size(),((float)(scale*step)/3.0f),scale,step);

    Point2d cg = Point2d(0.0f, 0.0f);
    Point3d cg3 = Point3d(0.0f, 0.0f, 0.0f);
    int ncg = 0;
    // navigate through all pixels of the epilolar segment
    // Compute the middle of the valid pixels of the epipolar segment (in rc camera) of the principal point (of the rc camera)
    for(int i = 0; i < allDepths; i++)
    {
        Point2d tpix = pFromTar + pixelVect * (float)i;
        Point3d p;
        if(triangulateMatch(p, rmid, tpix, rc, tc, _mp)) // triangulate principal point from rc with tpix
        {
            float depth = orientedPointPlaneDistance(p, rcplane.p, rcplane.n); // todo: can compute the distance to the camera (as it's the principal point it's the same)
            if( _mp.isPixelInImage(tpix, tc)
                && (depth > 0.0f)
                && checkPair(p, rc, tc, _mp, _mp.getMinViewAngle(), _mp.getMaxViewAngle()) )
            {
                cg = cg + tpix;
                cg3 = cg3 + p;
                ncg++;
            }
        }
    }
    if(ncg == 0)
    {
        return new StaticVector<float>();
    }
    cg = cg / (float)ncg;
    cg3 = cg3 / (float)ncg;
    allDepths = ncg;

    ALICEVISION_LOG_DEBUG("All correct depths: " << allDepths);

    Point2d midpoint = cg;
    if(midDepth > 0.0f)
    {
        Point3d midPt = rcplane.p + rcplane.n * midDepth;
        _mp.getPixelFor3DPoint(&midpoint, midPt, tc);
    }

    // compute the direction
    float direction = 1.0f;
    {
        Point3d p;
        if(!triangulateMatch(p, rmid, midpoint, rc, tc, _mp))
        {
            StaticVector<float>* out = new StaticVector<float>();
            return out;
        }

        float depth = orientedPointPlaneDistance(p, rcplane.p, rcplane.n);

        if(!triangulateMatch(p, rmid, midpoint + pixelVect, rc, tc, _mp))
        {
            StaticVector<float>* out = new StaticVector<float>();
            return out;
        }

        float depthP1 = orientedPointPlaneDistance(p, rcplane.p, rcplane.n);
        if(depth > depthP1)
        {
            direction = -1.0f;
        }
    }

    StaticVector<float>* out1 = new StaticVector<float>();
    out1->reserve(2 * maxDepthsHalf);

    Point2d tpix = midpoint;
    float depthOld = -1.0f;
    int istep = 0;
    bool ok = true;

    // compute depths for all pixels from the middle point to on one side of the epipolar line
    while((out1->size() < maxDepthsHalf) && (_mp.isPixelInImage(tpix, tc) == true) && (ok == true))
    {
        tpix = tpix + pixelVect * direction;

        Point3d refvect = _mp.iCamArr[rc] * rmid;
        Point3d tarvect = _mp.iCamArr[tc] * tpix;
        float rptpang = angleBetwV1andV2(refvect, tarvect);

        Point3d p;
        ok = triangulateMatch(p, rmid, tpix, rc, tc, _mp);

        float depth = orientedPointPlaneDistance(p, rcplane.p, rcplane.n);
        if (_mp.isPixelInImage(tpix, tc)
            && (depth > 0.0f) && (depth > depthOld)
            && checkPair(p, rc, tc, _mp, _mp.getMinViewAngle(), _mp.getMaxViewAngle())
            && (rptpang > _mp.getMinViewAngle())  // WARNING if vects are near parallel thaen this results to strange angles ...
            && (rptpang < _mp.getMaxViewAngle())) // this is the propper angle ... beacause is does not depend on the triangluated p
        {
            out1->push_back(depth);
            // if ((tpix.x!=tpixold.x)||(tpix.y!=tpixold.y)||(depthOld>=depth))
            //{
            // printf("after %f %f %f %f %i %f %f\n",tpix.x,tpix.y,depth,depthOld,istep,ang,kk);
            //};
        }
        else
        {
            ok = false;
        }
        depthOld = depth;
        istep++;
    }

    StaticVector<float>* out2 = new StaticVector<float>();
    out2->reserve(2 * maxDepthsHalf);
    tpix = midpoint;
    istep = 0;
    ok = true;

    // compute depths for all pixels from the middle point to the other side of the epipolar line
    while((out2->size() < maxDepthsHalf) && (_mp.isPixelInImage(tpix, tc) == true) && (ok == true))
    {
        Point3d refvect = _mp.iCamArr[rc] * rmid;
        Point3d tarvect = _mp.iCamArr[tc] * tpix;
        float rptpang = angleBetwV1andV2(refvect, tarvect);

        Point3d p;
        ok = triangulateMatch(p, rmid, tpix, rc, tc, _mp);

        float depth = orientedPointPlaneDistance(p, rcplane.p, rcplane.n);
        if(_mp.isPixelInImage(tpix, tc)
            && (depth > 0.0f) && (depth < depthOld) 
            && checkPair(p, rc, tc, _mp, _mp.getMinViewAngle(), _mp.getMaxViewAngle())
            && (rptpang > _mp.getMinViewAngle())  // WARNING if vects are near parallel thaen this results to strange angles ...
            && (rptpang < _mp.getMaxViewAngle())) // this is the propper angle ... beacause is does not depend on the triangluated p
        {
            out2->push_back(depth);
            // printf("%f %f\n",tpix.x,tpix.y);
        }
        else
        {
            ok = false;
        }

        depthOld = depth;
        tpix = tpix - pixelVect * direction;
    }

    // printf("out2\n");
    StaticVector<float>* out = new StaticVector<float>();
    out->reserve(2 * maxDepthsHalf);
    for(int i = out2->size() - 1; i >= 0; i--)
    {
        out->push_back((*out2)[i]);
        // printf("%f\n",(*out2)[i]);
    }
    // printf("out1\n");
    for(int i = 0; i < out1->size(); i++)
    {
        out->push_back((*out1)[i]);
        // printf("%f\n",(*out1)[i]);
    }

    delete out2;
    delete out1;

    // we want to have it in ascending order
    if(out->size() > 0 && (*out)[0] > (*out)[out->size() - 1])
    {
        StaticVector<float>* outTmp = new StaticVector<float>();
        outTmp->reserve(out->size());
        for(int i = out->size() - 1; i >= 0; i--)
        {
            outTmp->push_back((*out)[i]);
        }
        delete out;
        out = outTmp;
    }

    // check if it is asc
    for(int i = 0; i < out->size() - 1; i++)
    {
        if((*out)[i] > (*out)[i + 1])
        {

            for(int j = 0; j <= i + 1; j++)
            {
                ALICEVISION_LOG_TRACE("getDepthsRcTc: check if it is asc: " << (*out)[j]);
            }
            ALICEVISION_LOG_WARNING("getDepthsRcTc: not asc");

            if(out->size() > 1)
            {
                qsort(&(*out)[0], out->size(), sizeof(float), qSortCompareFloatAsc);
            }
        }
    }

    ALICEVISION_LOG_DEBUG("used depths: " << out->size());

    return out;
}

bool PlaneSweepingCuda::refineRcTcDepthMap(bool useTcOrRcPixSize, int nStepsToRefine, StaticVector<float>& out_simMap,
                                             StaticVector<float>& out_rcDepthMap,
                                             int rc_global_id,
                                             int tc_global_id, int scale, int wsh,
                                             float gammaC, float gammaP, int xFrom, int wPart)
{
    int h = _mp.getHeight(rc_global_id) / scale;

    long t1 = clock();

    ALICEVISION_LOG_DEBUG("\t- rc: " << rc_global_id << std::endl << "\t- tcams: " << tc_global_id);

    int rc_idx = addCam(rc_global_id, scale );
    int tc_idx = addCam(tc_global_id, scale );

    // sweep
    ps_refineRcDepthMap(out_simMap.getDataWritable().data(),
                        out_rcDepthMap.getDataWritable().data(), nStepsToRefine,
                        _cams[rc_idx],
                        _cams[tc_idx], wPart, h,
                        _mp.getWidth(rc_global_id)/scale, _mp.getHeight(rc_global_id)/scale,
                        _mp.getWidth(tc_global_id)/scale, _mp.getHeight(tc_global_id)/scale,
                        scale - 1, _CUDADeviceNo, _nImgsInGPUAtTime, _mp.verbose, wsh,
                        gammaC, gammaP, useTcOrRcPixSize, xFrom);

    mvsUtils::printfElapsedTime(t1);

    return true;
}

/* Be very careful with volume indexes:
 * volume is indexed with the same index as tc. The values of tc can be quite different.
 * depths is indexed with the index_set elements
 */
void PlaneSweepingCuda::sweepPixelsToVolume( CudaDeviceMemoryPitched<TSim, 3>& volBestSim_dmp,
                                             CudaDeviceMemoryPitched<TSim, 3>& volSecBestSim_dmp,
                                             const int volDimX,
                                             const int volDimY,
                                             const int volStepXY,
                                             const std::vector<OneTC>& tcs,
                                             const std::vector<float>& rc_depths,
                                             int rc_global_id,
                                             int wsh, float gammaC, float gammaP,
                                             const int scale)
{
    nvtxPush("preload host cache ");
    _ic.getImg_sync( rc_global_id );
    for( const auto& tc : tcs) _ic.getImg_sync( tc.getTCIndex() );
    nvtxPop("preload host cache ");

    const int volDimZ = rc_depths.size();

    ps::SimilarityVolume vol( volDimX, volDimY, volDimZ,
                              volStepXY,
                              scale,
                              rc_depths,
                              _mp.verbose );

    vol.initOutputVolumes( volBestSim_dmp,
                           volSecBestSim_dmp,
                           0 );
    vol.WaitSweepStream( 0 );

    for(int tci = 0; tci < tcs.size(); ++tci)
    {
        if( tci % MAX_CONSTANT_CAMERA_PARAM_SETS == MAX_CONSTANT_CAMERA_PARAM_SETS - 1 )
        {
            vol.WaitSweepStream( tci );
        }

        int tc_global_id = tcs[tci].getTCIndex();

        const int rcamCacheId = addCam(rc_global_id, vol.Scale(), vol.SweepStream( tci ) );
        CameraStruct& rcam = _cams[rcamCacheId];

        const int tcamCacheId = addCam(tc_global_id, vol.Scale(), vol.SweepStream( tci ) );
        CameraStruct& tcam = _cams[tcamCacheId];

        ALICEVISION_LOG_DEBUG("rc: " << rc_global_id << " tcams: " << tc_global_id);
        ALICEVISION_LOG_DEBUG("rcamCacheId: " << rcamCacheId << ", tcamCacheId: " << tcamCacheId);

        {
            pr_printfDeviceMemoryInfo();
            ALICEVISION_LOG_DEBUG(__FUNCTION__ << ": total size of one similarity volume map in GPU memory: approx " << volBestSim_dmp.getBytesPadded() / (1024.0 * 1024.0) << " MB");
            ALICEVISION_LOG_DEBUG(__FUNCTION__ << ": UNPADDED total size of one similarity volume map in GPU memory: approx " << volBestSim_dmp.getBytesUnpadded() / (1024.0 * 1024.0) << " MB");
        }

        const int rcWidth  = _mp.getWidth(rc_global_id);
        const int rcHeight = _mp.getHeight(rc_global_id);
        const int tcWidth  = _mp.getWidth(tc_global_id);
        const int tcHeight = _mp.getHeight(tc_global_id);

        ALICEVISION_LOG_DEBUG("sweepPixelsToVolume:" << std::endl
            << "\t- tci: " << tci << std::endl
            << "\t- tcs[tci].getDepthToStart(): " << tcs[tci].getDepthToStart() << std::endl
            << "\t- tcs[tci].getDepthsToSearch(): " << tcs[tci].getDepthsToSearch() << std::endl
            << "\t- volBestSim_dmp : " << volBestSim_dmp.getUnitsInDim(0) << ", " << volBestSim_dmp.getUnitsInDim(1) << ", " << volBestSim_dmp.getUnitsInDim(2) << std::endl
            << "\t- volSecBestSim_dmp : " << volSecBestSim_dmp.getUnitsInDim(0) << ", " << volSecBestSim_dmp.getUnitsInDim(1) << ", " << volSecBestSim_dmp.getUnitsInDim(2) << std::endl
            << "\t- scale: " << vol.Scale() << std::endl
            << "\t- volStepXY: " << vol.StepXY() << std::endl
            << "\t- volDimX: " << vol.DimX() << std::endl
            << "\t- volDimY: " << vol.DimY() << std::endl
            << "\t- volDimZ: " << vol.DimZ());

        {
            clock_t t1 = tic();

            ALICEVISION_LOG_DEBUG("ps_computeSimilarityVolume:" << std::endl
                << "\t- scale: " << vol.Scale() << std::endl
                << "\t- volStepXY: " << vol.StepXY() << std::endl
                << "\t- volDimX: " << vol.DimX() << std::endl
                << "\t- volDimY: " << vol.DimY());

            // last synchronous step
            // cudaDeviceSynchronize();
            vol.compute(
                volBestSim_dmp,
                volSecBestSim_dmp,
                rcam, rcWidth, rcHeight,
                tcam, tcWidth, tcHeight,
                tcs[tci],
                wsh,
                gammaC, gammaP,
                tci);

            ALICEVISION_LOG_DEBUG("ps_computeSimilarityVolume elapsed time: " << toc(t1) << " ms.");
        }
    }
}


/**
 * @param[inout] volume input similarity volume
 */
bool PlaneSweepingCuda::SGMoptimizeSimVolume(int rc,
                                             const CudaDeviceMemoryPitched<TSim, 3>& volSim_dmp,
                                             CudaDeviceMemoryPitched<TSim, 3>& volSimFiltered_dmp,
                                             int volDimX, int volDimY, int volDimZ,
                                             const std::string& filteringAxes,
                                             int scale, unsigned char P1, unsigned char P2)
{
    auto startTime = std::chrono::high_resolution_clock::now();

    ALICEVISION_LOG_DEBUG("SGM optimizing volume:" << std::endl
                          << "\t- volDimX: " << volDimX << std::endl
                          << "\t- volDimY: " << volDimY << std::endl
                          << "\t- volDimZ: " << volDimZ << std::endl
                          << "\t- filteringAxes: " << filteringAxes);

    int rc_cam_idx = addCam(rc, scale );

    ps_SGMoptimizeSimVolume(_cams[rc_cam_idx],
                            volSim_dmp,
                            volSimFiltered_dmp,
                            volDimX, volDimY, volDimZ,
                            filteringAxes,
                            _mp.verbose, P1, P2, scale,
                            _CUDADeviceNo, _nImgsInGPUAtTime);

    ALICEVISION_LOG_INFO("==== SGMoptimizeSimVolume done in : " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - startTime).count() << "ms.");

    return true;
}

void PlaneSweepingCuda::SGMretrieveBestDepth(DepthSimMap& bestDepth, CudaDeviceMemoryPitched<TSim, 3>& volSim_dmp, const StaticVector<float>& depths, const int rcCamId,
  int volDimX, int volDimY, int volDimZ, int scaleStep, bool interpolate)
{
  ALICEVISION_LOG_DEBUG("SGMretrieveBestDepth:" << std::endl
    << "\t- volDimX: " << volDimX << std::endl
    << "\t- volDimY: " << volDimY << std::endl
    << "\t- volDimZ: " << volDimZ);

  int rc_frame_cache_idx = addCam(rcCamId, 1 );

  int rc_cam_cache_idx = _hidden->getLocalCamId(rc_frame_cache_idx);

  CudaDeviceMemory<float> depths_d(depths.getData().data(), depths.size());

  CudaDeviceMemoryPitched<float, 2> bestDepth_dmp(CudaSize<2>(volDimX, volDimY));
  CudaDeviceMemoryPitched<float, 2> bestSim_dmp(CudaSize<2>(volDimX, volDimY));

  long t1 = clock();

  ps_SGMretrieveBestDepth(
    bestDepth_dmp,
    bestSim_dmp,
    rc_cam_cache_idx,
    depths_d,
    volSim_dmp,
    volDimX, volDimY, volDimZ,
    scaleStep,
    interpolate);

  /*
  {
      CudaTexture<float> bestDepth_tex(bestDepth_dmp);
      ps_medianFilter3(bestDepth_tex.textureObj, bestDepth_dmp);
  }
  */

  CudaHostMemoryHeap<float, 2> bestDepth_hmh(CudaSize<2>(volDimX, volDimY));
  bestDepth_hmh.copyFrom(bestDepth_dmp);
  bestDepth_dmp.deallocate();

  CudaHostMemoryHeap<float, 2> bestSim_hmh(CudaSize<2>(volDimX, volDimY));
  bestSim_hmh.copyFrom(bestSim_dmp);
  bestSim_dmp.deallocate();

  for (int y = 0; y < volDimY; ++y)
  {
    for (int x = 0; x < volDimX; ++x)
    {
      DepthSim& out = bestDepth._dsm[y * volDimX + x];
      out.depth = bestDepth_hmh(x, y);
      out.sim = bestSim_hmh(x, y);
    }
  }
  mvsUtils::printfElapsedTime(t1);
}

// make_float3(avail,total,used)
Point3d PlaneSweepingCuda::getDeviceMemoryInfo()
{
    size_t iavail;
    size_t itotal;
    cudaMemGetInfo(&iavail, &itotal);
    size_t iused = itotal - iavail;

    double avail = (double)iavail / (1024.0 * 1024.0);
    double total = (double)itotal / (1024.0 * 1024.0);
    double used = (double)iused / (1024.0 * 1024.0);

    return Point3d(avail, total, used);
}

bool PlaneSweepingCuda::fuseDepthSimMapsGaussianKernelVoting(int w, int h, StaticVector<DepthSim>& oDepthSimMap,
                                                               const StaticVector<StaticVector<DepthSim>*>& dataMaps,
                                                               int nSamplesHalf, int nDepthsToRefine, float sigma)
{
    long t1 = clock();

    // sweep
    std::vector<CudaHostMemoryHeap<float2, 2>*> dataMaps_hmh(dataMaps.size());
    for(int i = 0; i < dataMaps.size(); i++)
    {
        dataMaps_hmh[i] = new CudaHostMemoryHeap<float2, 2>(CudaSize<2>(w, h));
        for(int y = 0; y < h; y++)
        {
            for(int x = 0; x < w; x++)
            {
                float2& data_hmh = (*dataMaps_hmh[i])(x, y);
                const DepthSim& data = (*dataMaps[i])[y * w + x];
                data_hmh.x = data.depth;
                data_hmh.y = data.sim;
            }
        }
    }

    CudaHostMemoryHeap<float2, 2> oDepthSimMap_hmh(CudaSize<2>(w, h));

    ps_fuseDepthSimMapsGaussianKernelVoting(&oDepthSimMap_hmh, dataMaps_hmh, dataMaps.size(), nSamplesHalf,
                                            nDepthsToRefine, sigma, w, h, _mp.verbose);

    for(int y = 0; y < h; y++)
    {
        for(int x = 0; x < w; x++)
        {
            const float2& oDepthSim_hmh = oDepthSimMap_hmh(x, y);
            DepthSim& oDepthSim = oDepthSimMap[y * w + x];
            oDepthSim.depth = oDepthSim_hmh.x;
            oDepthSim.sim = oDepthSim_hmh.y;
        }
    }

    for(int i = 0; i < dataMaps.size(); i++)
    {
        delete dataMaps_hmh[i];
    }

    mvsUtils::printfElapsedTime(t1);

    return true;
}

bool PlaneSweepingCuda::optimizeDepthSimMapGradientDescent(StaticVector<DepthSim>& oDepthSimMap,
                                                           const StaticVector<DepthSim>& sgmDepthPixSizeMap,
                                                           const StaticVector<DepthSim>& refinedDepthSimMap,
                                                           int rc_global_id,
                                                           int nSamplesHalf, int nDepthsToRefine, float sigma,
                                                           int nIters, int yFrom, int hPart)
{
    ALICEVISION_LOG_DEBUG("optimizeDepthSimMapGradientDescent.");

    int scale = 1;
    int w = _mp.getWidth(rc_global_id);

    long t1 = clock();

    int rc_idx = addCam(rc_global_id, scale);

    ALICEVISION_LOG_DEBUG(__FUNCTION__ << " RC: " << rc_global_id << ", rc_cache_idx: " << rc_idx);

    // sweep
    CudaHostMemoryHeap<float2, 2> sgmDepthPixSizeMap_hmh(CudaSize<2>(w, hPart));
    CudaHostMemoryHeap<float2, 2> refinedDepthSimMap_hmh(CudaSize<2>(w, hPart));
    copy(sgmDepthPixSizeMap_hmh, sgmDepthPixSizeMap, yFrom);
    copy(refinedDepthSimMap_hmh, refinedDepthSimMap, yFrom);

    CudaHostMemoryHeap<float2, 2> oDepthSimMap_hmh(CudaSize<2>(w, hPart));

    ps_optimizeDepthSimMapGradientDescent(
            oDepthSimMap_hmh,
            sgmDepthPixSizeMap_hmh, refinedDepthSimMap_hmh,
            nSamplesHalf, nDepthsToRefine, nIters, sigma,
            _cams[rc_idx], w, hPart,
            scale - 1, _CUDADeviceNo, _nImgsInGPUAtTime,
            _mp.verbose, yFrom);

    copy(oDepthSimMap, oDepthSimMap_hmh, yFrom);

    mvsUtils::printfElapsedTime(t1);

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

/*********************************************************************************
 * FrameCacheEntry
 *********************************************************************************/

FrameCacheEntry::FrameCacheEntry( int cache_frame_id, int w, int h, int s )
    : _cache_frame_id( cache_frame_id )
    , _cache_cam_id( -1 )
    , _global_cam_id( -1 )
    , _width( w )
    , _height( h )
    , _scales( s )
    , _memBytes( 0 )
{
    CudaSize<2> sz( w, h );
    _host_frame = new CudaHostMemoryHeap<CudaRGBA, 2>( sz );
    _memBytes = ps_deviceAllocate( _pyramid, w, h, s );
}

FrameCacheEntry::~FrameCacheEntry( )
{
    ps_deviceDeallocate( _pyramid, _scales );
    delete _host_frame;
}

Pyramid& FrameCacheEntry::getPyramid()
{
    return _pyramid;
}

Pyramid* FrameCacheEntry::getPyramidPtr()
{
    return &_pyramid;
}

int FrameCacheEntry::getPyramidMem() const
{
    return _memBytes;
}

void FrameCacheEntry::fillFrame( int global_cam_id,
                                 mvsUtils::ImagesCache<ImageRGBAf>& imageCache,
                                 mvsUtils::MultiViewParams& mp,
                                 cudaStream_t stream )
{
    ALICEVISION_LOG_TRACE(__FUNCTION__ << ": camera:" << global_cam_id << " " << mp.getWidth(global_cam_id) << "x" << mp.getHeight(global_cam_id));

    /* Copy data for cached image "global_cam_id" into the host-side data buffer managed
     * by data structure "cam". */
    fillHostFrameFromImageCache( imageCache, _host_frame, global_cam_id, mp );

    /* Copy data from host-sided cache in "cam" onto the GPU and create
     * downscaled and Gauss-filtered versions on the GPU. */
    ps_device_fillPyramidFromHostFrame( _pyramid,
                         _host_frame,
                         _scales,
                         mp.getWidth(global_cam_id),
                         mp.getHeight(global_cam_id),
                         stream );
}

void FrameCacheEntry::fillHostFrameFromImageCache(
    mvsUtils::ImagesCache<ImageRGBAf>& ic,
    CudaHostMemoryHeap<CudaRGBA, 2>* hostFrame,
    int c,
    mvsUtils::MultiViewParams& mp )
{
    clock_t t1 = tic();
    mvsUtils::ImagesCache<ImageRGBAf>::ImgSharedPtr img = ic.getImg_sync( c );
    ALICEVISION_LOG_TRACE(__FUNCTION__ << ": " << c << " -a- Retrieve from ImagesCache elapsed time: " << toc(t1) << " ms.");
    t1 = tic();

    const int h = mp.getHeight(c);
    const int w = mp.getWidth(c);
    for(int y = 0; y < h; ++y)
    {
        for(int x = 0; x < w; ++x)
        {
            const ColorRGBAf& floatRGBA = img->at(x, y);
            CudaRGBA& pix_rgba = (*hostFrame)(x, y);
            pix_rgba.x = floatRGBA.r * 255.0f;
            pix_rgba.y = floatRGBA.g * 255.0f;
            pix_rgba.z = floatRGBA.b * 255.0f;
            pix_rgba.w = floatRGBA.a * 255.0f;
        }
    }
    ALICEVISION_LOG_DEBUG(__FUNCTION__ << ": " << c << " -b- Copy to HMH elapsed time: " << toc(t1) << " ms.");
}

void FrameCacheEntry::setLocalCamId( int cache_cam_id )
{
    _cache_cam_id = cache_cam_id;
}

int FrameCacheEntry::getLocalCamId( ) const
{
    return _cache_cam_id;
}

/*********************************************************************************
 * FrameCacheMemory
 *********************************************************************************/

FrameCacheMemory::FrameCacheMemory( int ImgsInGPUAtTime,
                                    int maxWidth, int maxHeight, int scales, int CUDAdeviceNo )
{
    int allBytes = 0;

    /* If not done before, initialize Gaussian filters in GPU constant mem.  */
    ps_create_gaussian_arr( CUDAdeviceNo, scales );

    pr_printfDeviceMemoryInfo();

    _v.resize( ImgsInGPUAtTime );

    for( int i=0; i<ImgsInGPUAtTime; i++ )
    {
        _v[i] = new FrameCacheEntry( i, maxWidth, maxHeight, scales );
        allBytes += _v[i]->getPyramidMem();
    }

    ALICEVISION_LOG_INFO( "FrameCache for GPU " << CUDAdeviceNo << ", " << scales << " scales, allocated " << allBytes << " on GPU" );

    pr_printfDeviceMemoryInfo();
}

FrameCacheMemory::~FrameCacheMemory( )
{
    for( auto ptr : _v )
    {
        delete ptr;
    }
}

void FrameCacheMemory::fillFrame( int cache_frame_id,
                                  int global_cam_id,
                                  mvsUtils::ImagesCache<ImageRGBAf>& imageCache,
                                  mvsUtils::MultiViewParams& mp,
                                  cudaStream_t stream )
{
    _v[cache_frame_id]->fillFrame( global_cam_id, imageCache, mp, stream );
}

void FrameCacheMemory::setLocalCamId( int cache_frame_id, int cache_cam_id )
{
    _v[cache_frame_id]->setLocalCamId( cache_cam_id );
}

int FrameCacheMemory::getLocalCamId( int cache_frame_id ) const
{
    return _v[cache_frame_id]->getLocalCamId( );
}

/*********************************************************************************
 * CamSelection
 *********************************************************************************/

bool operator==( const CamSelection& l, const CamSelection& r )
{
    return ( l.first == r.first && l.second == r.second );
}

bool operator<( const CamSelection& l, const CamSelection& r )
{
    return ( l.first < r.first || 
           ( l.first == r.first && l.second < r.second ) );
}

} // namespace depthMap
} // namespace aliceVision
