// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "PlaneSweepingCuda.hpp"
#include <aliceVision/depthMap/volumeIO.hpp>

#include <aliceVision/system/Logger.hpp>
#include <aliceVision/mvsData/Matrix3x3.hpp>
#include <aliceVision/mvsData/Matrix3x4.hpp>
#include <aliceVision/mvsData/OrientedPoint.hpp>
#include <aliceVision/mvsUtils/common.hpp>
#include <aliceVision/mvsUtils/fileIO.hpp>
#include <aliceVision/depthMap/cuda/planeSweeping/plane_sweeping_cuda.hpp>
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

static void cps_host_fillCameraData(mvsUtils::ImagesCache<ImageRGBAf>& ic, CameraStruct& cam, int c, mvsUtils::MultiViewParams& mp)
{
    ALICEVISION_LOG_DEBUG("cps_host_fillCameraData [" << c << "]: " << mp.getWidth(c) << "x" << mp.getHeight(c));
    clock_t t1 = tic();
    mvsUtils::ImagesCache<ImageRGBAf>::ImgSharedPtr img = ic.getImg_sync( c ); // TODO RGBA
    ALICEVISION_LOG_DEBUG("cps_host_fillCameraData: " << c << " -a- Retrieve from ImagesCache elapsed time: " << toc(t1) << " ms.");
    t1 = tic();

    const int h = mp.getHeight(c);
    const int w = mp.getWidth(c);
    for(int y = 0; y < h; ++y)
    {
        for(int x = 0; x < w; ++x)
        {
            const ColorRGBAf& floatRGBA = img->at(x, y);
            CudaRGBA& pix_rgba = (*cam.tex_rgba_hmh)(x, y);
            pix_rgba.x = floatRGBA.r * 255.0f;
            pix_rgba.y = floatRGBA.g * 255.0f;
            pix_rgba.z = floatRGBA.b * 255.0f;
            pix_rgba.w = floatRGBA.a * 255.0f;
        }
    }
    ALICEVISION_LOG_DEBUG("cps_host_fillCameraData: " << c << " -b- Copy to HMH elapsed time: " << toc(t1) << " ms.");
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



PlaneSweepingCuda::PlaneSweepingCuda( int CUDADeviceNo,
                                      mvsUtils::ImagesCache<ImageRGBAf>&     ic,
                                      mvsUtils::MultiViewParams& mp,
                                      int scales )
    : _scales( scales )
    , _CUDADeviceNo( CUDADeviceNo )
    , _ic( ic )
    , _mp(mp)
{
    ps_testCUDAdeviceNo( _CUDADeviceNo );

    cudaError_t err;

    /* The caller knows all camera that will become rc cameras, but it does not
     * pass that information to this function.
     * It knows the nearest cameras for each of those rc cameras, but it doesn't
     * pass that information, either.
     * So, the only task of this function is to allocate an amount of memory that
     * will hold CUDA memory for camera structs and bitmaps.
     */

    const int maxImageWidth = mp.getMaxImageWidth();
    const int maxImageHeight = mp.getMaxImageHeight();

    float oneimagemb = 4.0f * sizeof(float) * (((float)(maxImageWidth * maxImageHeight) / 1024.0f) / 1024.0f);
    for(int scale = 2; scale <= _scales; ++scale)
    {
        oneimagemb += 4.0 * sizeof(float) * (((float)((maxImageWidth / scale) * (maxImageHeight / scale)) / 1024.0) / 1024.0);
    }
    float maxmbGPU = 400.0f; // TODO FACA
    _nImgsInGPUAtTime = (int)(maxmbGPU / oneimagemb);
    _nImgsInGPUAtTime = std::max(2, std::min(mp.ncams, _nImgsInGPUAtTime));

    ALICEVISION_LOG_INFO("PlaneSweepingCuda:" << std::endl
                         << "\t- _nImgsInGPUAtTime: " << _nImgsInGPUAtTime << std::endl
                         << "\t- scales: " << _scales);

    if( _nImgsInGPUAtTime > MAX_CONCURRENT_IMAGES_IN_DEPTHMAP )
    {
        ALICEVISION_LOG_ERROR( "DepthMap has been compiled with a hard limit of "
                               << MAX_CONCURRENT_IMAGES_IN_DEPTHMAP
                               << " concurrent images. "<< std::endl
                               << "Recompilation required for larger values." << std::endl
                               << "Change define MAX_CONCURRENT_IMAGES_IN_DEPTHMAP "
                               << " but consider hardware limits for CUDA constant memory." );
        exit( -1 );

    }

    // allocate global on the device
    ps_deviceAllocate(_hidden_pyramids, _nImgsInGPUAtTime, maxImageWidth, maxImageHeight, _scales, _CUDADeviceNo);

    err = cudaMalloc( &_camsBasesDev, MAX_CONCURRENT_IMAGES_IN_DEPTHMAP*sizeof(CameraStructBase) );
    THROW_ON_CUDA_ERROR( err, "Could not allocate set of camera structs in device memory in " << __FILE__ << ":" << __LINE__ << ", " << cudaGetErrorString(err) );

    err = cudaMallocHost( &_camsBasesHst, MAX_CONCURRENT_IMAGES_IN_DEPTHMAP*sizeof(CameraStructBase) );
    THROW_ON_CUDA_ERROR( err, "Could not allocate set of camera structs in pinned host memory in " << __FILE__ << ":" << __LINE__ << ", " << cudaGetErrorString(err) );

    _camsBasesHstScale.resize( MAX_CONCURRENT_IMAGES_IN_DEPTHMAP, -1 );

    _cams     .resize(_nImgsInGPUAtTime);
    _camsRcs  .resize(_nImgsInGPUAtTime);
    _camsTimes.resize(_nImgsInGPUAtTime);

    for( int rc = 0; rc < _nImgsInGPUAtTime; ++rc )
    {
        _cams[rc].param_hst = &_camsBasesHst[rc];
        _cams[rc].param_dev = &_camsBasesDev[rc];
        _cams[rc].pyramid   = &_hidden_pyramids[rc];

        err = cudaStreamCreate( &_cams[rc].stream );
        if( err != cudaSuccess )
        {
            ALICEVISION_LOG_WARNING("Failed to create a CUDA stream object for async sweeping");
            _cams[rc].stream = 0;
        }
    }

    for(int rc = 0; rc < _nImgsInGPUAtTime; ++rc)
    {
        _cams[rc].camId = -1;
        _camsRcs[rc]   = -1;
        _camsTimes[rc] = 0;
    }
}

PlaneSweepingCuda::~PlaneSweepingCuda()
{
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // deallocate global on the device
    ps_deviceDeallocate(_hidden_pyramids, _CUDADeviceNo, _nImgsInGPUAtTime, _scales);

    cudaFree(     _camsBasesDev );
    cudaFreeHost( _camsBasesHst );

    for(int c = 0; c < _cams.size(); c++)
    {
        delete _cams[c].tex_rgba_hmh;

        cudaStreamDestroy( _cams[c].stream );
    }
}

void PlaneSweepingCuda::cameraToDevice( int rc, const StaticVector<int>& tcams )
{
    std::ostringstream ostr;

    ostr << "Called " << __FUNCTION__ << " with cameras" << std::endl
         << "    rc = " << rc << std::endl;
    for( auto it : tcams )
    {
        ostr << "    " << it << std::endl;
    }

    ALICEVISION_LOG_DEBUG( ostr.str() );
}

int PlaneSweepingCuda::addCam( int global_cam_id, int scale )
{
    // first is oldest
    int id = _camsRcs.indexOf(global_cam_id);
    if(id == -1)
    {
        // get oldest id
        id = _camsTimes.minValId();
        CameraStruct& cam = _cams[id];
        cam.camId = id;

        if(cam.tex_rgba_hmh == nullptr)
        {
            cam.tex_rgba_hmh =
                new CudaHostMemoryHeap<CudaRGBA, 2>(CudaSize<2>(_mp.getMaxImageWidth(), _mp.getMaxImageHeight()));
        }
        else
        {
            assert(cam.tex_rgba_hmh->getSize() == CudaSize<2>(_mp.getMaxImageWidth(), _mp.getMaxImageHeight()));
        }
        long t1 = clock();

        cps_host_fillCamera(_camsBasesHst[id], global_cam_id, _mp, scale);

        _camsBasesHstScale[id] = scale;

        /* Copy data for cached image "global_cam_id" into the host-side data buffer managed
         * by data structure "cam". */
        cps_host_fillCameraData(_ic, cam, global_cam_id, _mp);

        /* Copy data from host-sided cache in "cam" onto the GPU and create
         * downscaled and Gauss-filtered versions on the GPU. */
        ps_device_updateCam(cam,
                            _CUDADeviceNo,
                            _scales, _mp.getWidth(global_cam_id), _mp.getHeight(global_cam_id));

        mvsUtils::printfElapsedTime(t1, "Copy image (camera id="+std::to_string(global_cam_id)+") from CPU to GPU");

        _camsRcs[id] = global_cam_id;

        ps_loadCameraStructs( _camsBasesDev, _camsBasesHst, id );
    }
    else if( _camsBasesHstScale[id] == scale )
    {
        /* do nothing, the CameraStruct at position id is unchanged */
    }
    else
    {
        /*
         * It is not sensible to waste cycles on refilling the camera struct if the new one
         * is identical to the old one.
         */
        cps_host_fillCamera(_camsBasesHst[id], global_cam_id, _mp, scale);

        _camsBasesHstScale[id] = scale;

        // ps_device_updateCam((CameraStruct*)(*cams)[id], id, _scales);
        ALICEVISION_LOG_DEBUG("Reuse image (camera id=" + std::to_string(global_cam_id) + ") already on the GPU.");

        ps_loadCameraStructs( _camsBasesDev, _camsBasesHst, id );
    }
    _camsTimes[id] = clock();

    if( _cams[id].camId != id )
    {
        std::cerr << "BUG in " << __FILE__ << ":" << __LINE__ << " ?"
                  << " The camId member should be initialized with the return value of addCam()."
                  << std::endl;
        exit( -1 );
    }

    return id;
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
    // int w = _mp.getWidth(rc_global_id)/scale;
    int w = wPart;
    int h = _mp.getHeight(rc_global_id) / scale;

    long t1 = clock();

    ALICEVISION_LOG_DEBUG("\t- rc: " << rc_global_id << std::endl << "\t- tcams: " << tc_global_id);

    int rc_idx = addCam(rc_global_id, scale );
    int tc_idx = addCam(tc_global_id, scale );

    // sweep
    ps_refineRcDepthMap(out_simMap.getDataWritable().data(),
                        out_rcDepthMap.getDataWritable().data(), nStepsToRefine,
                        _cams[rc_idx],
                        _cams[tc_idx],
                        w, h,
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
                                             const StaticVector<int>& tc_global_list,
                                             int wsh, float gammaC, float gammaP,
                                             int scale)
{
    int volDimZ = rc_depths.size();
    ps_initSimilarityVolume(
      volBestSim_dmp,
      volSecBestSim_dmp,
      volDimX, volDimY, volDimZ);

    // copy the vector of depths to GPU
    CudaDeviceMemory<float> depths_d(rc_depths.data(), rc_depths.size());

#ifdef PLANE_SWEEPING_PRECOMPUTED_COLORS
    CudaDeviceMemoryPitched<float4, 3> volTcamColors_dmp(CudaSize<3>(volDimX, volDimY, volDimZ)); // TODO FACA: move out of the loop (max of depthsToSearch)
#endif

    int s = scale - 1;

    for(int tci = 0; tci < tc_global_list.size(); ++tci)
    {
        int tc_global_id = tc_global_list[tci];

        const int rcamCacheId = addCam(rc_global_id, scale );
        CameraStruct& rcam = _cams[rcamCacheId];

        const int tcamCacheId = addCam(tc_global_id, scale );
        CameraStruct& tcam = _cams[tcamCacheId];


        ALICEVISION_LOG_DEBUG("rc: " << rc_global_id << " tcams: " << tc_global_id);
        ALICEVISION_LOG_DEBUG("rcamCacheId: " << rcamCacheId << ", tcamCacheId: " << tcamCacheId);

        // CudaDeviceMemoryPitched<float4, 3> volTcamColors_dmp(CudaSize<3>(volDimX, volDimY, tcs[tci].getDepthsToSearch())); // TODO FACA: move out of the loop (max of depthsToSearch)

        {
            pr_printfDeviceMemoryInfo();
            ALICEVISION_LOG_DEBUG(__FUNCTION__ << ": total size of one similarity volume map in GPU memory: approx " << volBestSim_dmp.getBytesPadded() / (1024.0 * 1024.0) << " MB");
            ALICEVISION_LOG_DEBUG(__FUNCTION__ << ": UNPADDED total size of one similarity volume map in GPU memory: approx " << volBestSim_dmp.getBytesUnpadded() / (1024.0 * 1024.0) << " MB");
#ifdef PLANE_SWEEPING_PRECOMPUTED_COLORS
            ALICEVISION_LOG_DEBUG(__FUNCTION__ << ": total size of one color volume map in GPU memory: approx " << volTcamColors_dmp.getBytesPadded() / (1024.0 * 1024.0) << " MB");
            ALICEVISION_LOG_DEBUG(__FUNCTION__ << ": UNPADDED total size of one color volume map in GPU memory: approx " << volTcamColors_dmp.getBytesUnpadded() / (1024.0 * 1024.0) << " MB");
#endif
        }

        const int rcWidth = _mp.getWidth(rc_global_id);
        const int rcHeight = _mp.getHeight(rc_global_id);
        const int tcWidth = _mp.getWidth(tc_global_id);
        const int tcHeight = _mp.getHeight(tc_global_id);

        ALICEVISION_LOG_DEBUG("sweepPixelsToVolume:" << std::endl
            << "\t- tci: " << tci << std::endl
            << "\t- tcs[tci].getDepthToStart(): " << tcs[tci].getDepthToStart() << std::endl
            << "\t- tcs[tci].getDepthsToSearch(): " << tcs[tci].getDepthsToSearch() << std::endl
            << "\t- volBestSim_dmp : " << volBestSim_dmp.getUnitsInDim(0) << ", " << volBestSim_dmp.getUnitsInDim(1) << ", " << volBestSim_dmp.getUnitsInDim(2) << std::endl
            << "\t- volSecBestSim_dmp : " << volSecBestSim_dmp.getUnitsInDim(0) << ", " << volSecBestSim_dmp.getUnitsInDim(1) << ", " << volSecBestSim_dmp.getUnitsInDim(2) << std::endl
            << "\t- scale: " << scale << std::endl
            << "\t- volStepXY: " << volStepXY << std::endl
            << "\t- volDimX: " << volDimX << std::endl
            << "\t- volDimY: " << volDimY << std::endl
            << "\t- volDimZ: " << volDimZ);
#ifdef PLANE_SWEEPING_PRECOMPUTED_COLORS
        ALICEVISION_LOG_DEBUG("\t- volTcamColors_dmp : " << volTcamColors_dmp.getUnitsInDim(0) << ", " << volTcamColors_dmp.getUnitsInDim(1) << ", " << volTcamColors_dmp.getUnitsInDim(2));
#endif

#ifdef PLANE_SWEEPING_PRECOMPUTED_COLORS
        ps_initColorVolumeFromCamera(volTcamColors_dmp,
                                     rcam, tcam,
                                     (*tcam.pyramid)[s].tex,
                                     tcWidth, tcHeight,
                                     tcs[tci].getDepthToStart(), depths_d,
                                     volDimX, volDimY, tcs[tci].getDepthsToSearch(), volStepXY);

        /*
        if (true) // debug
        {
            CudaHostMemoryHeap<float4, 3> volTcamColors_h(volTcamColors_dmp.getSize());
            volTcamColors_h.copyFrom(volTcamColors_dmp);

            exportColorVolume(volTcamColors_h, rc_depths, tcs[tci].getDepthToStart(), tcs[tci].getDepthsToSearch(), _mp, rc, scale, volStepXY, _mp.getDepthMapsFolder() + std::to_string(_mp.getViewId(rc)) + "_" + std::to_string(_mp.getViewId(tc)) + "_vol_colors.abc");
        }*/
#endif

// #define PLANE_SWEEPING_PRECOMPUTED_COLORS_TEXTURE 1
#ifdef PLANE_SWEEPING_PRECOMPUTED_COLORS_TEXTURE
        cudaTextureObject_t volTcamColors_tex3D;
        {
            cudaTextureDesc  tex_desc;
            memset(&tex_desc, 0, sizeof(cudaTextureDesc));
            tex_desc.normalizedCoords = 0; // addressed (x,y,z) in [width,height,depth]
            tex_desc.addressMode[0] = cudaAddressModeClamp;
            tex_desc.addressMode[1] = cudaAddressModeClamp;
            tex_desc.addressMode[2] = cudaAddressModeClamp;
            tex_desc.readMode = cudaReadModeElementType;
            tex_desc.filterMode = cudaFilterModePoint;

            cudaResourceDesc res_desc;
            res_desc.resType = cudaResourceTypePitch2D;
            res_desc.res.pitch2D.desc = cudaCreateChannelDesc<float4>();
            res_desc.res.pitch2D.devPtr = volTcamColors_dmp.getBuffer();
            res_desc.res.pitch2D.width = volTcamColors_dmp.getSize()[0];
            res_desc.res.pitch2D.height = volTcamColors_dmp.getSize()[1];
            res_desc.res.pitch2D.pitchInBytes = volTcamColors_dmp.getPitch();

            // create texture object
            cudaError_t err = cudaCreateTextureObject(&volTcamColors_tex3D, &res_desc, &tex_desc, NULL);

            THROW_ON_CUDA_ERROR(err, "Could not create the 3D texture object in " << __FILE__ << ":" << __LINE__ << ", " << cudaGetErrorString(err));
        }
#endif

        // FACA WIP: for now, only create one cell for a TC
        std::vector<OneTC> cells;
        cells.push_back(tcs[tci]);
        
        {
            clock_t t1 = tic();

            ALICEVISION_LOG_DEBUG("ps_computeSimilarityVolume:" << std::endl
                << "\t- scale: " << scale << std::endl
                << "\t- volStepXY: " << volStepXY << std::endl
                << "\t- volDimX: " << volDimX << std::endl
                << "\t- volDimY: " << volDimY);

            // last synchronous step
            // cudaDeviceSynchronize();
#ifdef PLANE_SWEEPING_PRECOMPUTED_COLORS
            int s = scale - 1;
            ps_computeSimilarityVolume_precomputedColors(
                (*rcam.pyramid)[s].tex,
#ifdef PLANE_SWEEPING_PRECOMPUTED_COLORS_TEXTURE
                volTcamColors_tex3D,
#else
                volTcamColors_dmp,
#endif
                volBestSim_dmp,
                volSecBestSim_dmp,
                rcam, rcWidth, rcHeight,
                volStepXY, volDimX, volDimY,
                cells.front(),
                wsh, _nbestkernelSizeHalf,
                scale,
                _mp.verbose,
                gammaC, gammaP);
#else
            ps_computeSimilarityVolume(
                volBestSim_dmp,
                volSecBestSim_dmp,
                rcam, rcWidth, rcHeight,
                tcam, tcWidth, tcHeight,
                volStepXY, volDimX, volDimY,
                depths_d,
                cells,
                wsh,
                _nbestkernelSizeHalf,
                scale,
                _mp.verbose,
                gammaC, gammaP);
#endif

#ifdef PLANE_SWEEPING_PRECOMPUTED_COLORS
            ALICEVISION_LOG_DEBUG("ps_computeSimilarityVolume_precomputedColors elapsed time: " << toc(t1) << " ms.");
#else
            ALICEVISION_LOG_DEBUG("ps_computeSimilarityVolume without precomputedColors elapsed time: " << toc(t1) << " ms.");
#endif
        }
        // cudaDestroyTextureObject(volTcamColors_tex3D);
    }
    // cudaDeviceSynchronize();
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

  int rc_cam_idx = addCam(rcCamId, 1 );

  CudaDeviceMemory<float> depths_d(depths.getData().data(), depths.size());

  CudaDeviceMemoryPitched<float, 2> bestDepth_dmp(CudaSize<2>(volDimX, volDimY));
  CudaDeviceMemoryPitched<float, 2> bestSim_dmp(CudaSize<2>(volDimX, volDimY));

  long t1 = clock();
  ps_SGMretrieveBestDepth(
    bestDepth_dmp,
    bestSim_dmp,
    _cams[rc_cam_idx],
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
    int h = hPart;

    long t1 = clock();

    int rc_idx = addCam(rc_global_id, scale );

    ALICEVISION_LOG_DEBUG("rc: " << rc_global_id);

    CameraStruct& ttcam = _cams[rc_idx];

    // sweep
    CudaHostMemoryHeap<float2, 2> sgmDepthPixSizeMap_hmh(CudaSize<2>(w, h));
    CudaHostMemoryHeap<float2, 2> refinedDepthSimMap_hmh(CudaSize<2>(w, h));
    copy(sgmDepthPixSizeMap_hmh, sgmDepthPixSizeMap, yFrom);
    copy(refinedDepthSimMap_hmh, refinedDepthSimMap, yFrom);

    CudaHostMemoryHeap<float2, 2> oDepthSimMap_hmh(CudaSize<2>(w, h));

    ps_optimizeDepthSimMapGradientDescent(
            oDepthSimMap_hmh,
            sgmDepthPixSizeMap_hmh, refinedDepthSimMap_hmh,
            nSamplesHalf, nDepthsToRefine, nIters, sigma,
            _cams[rc_idx],
            w, h, scale - 1, _CUDADeviceNo, _nImgsInGPUAtTime,
            _mp.verbose, yFrom);

    copy(oDepthSimMap, oDepthSimMap_hmh, yFrom);

    mvsUtils::printfElapsedTime(t1);

    return true;
}

bool PlaneSweepingCuda::computeNormalMap(
    StaticVector<float>* depthMap, StaticVector<ColorRGBf>* normalMap, int rc,
    int scale, float igammaC, float igammaP, int wsh)
{
  const int w = _mp.getWidth(rc) / scale;
  const int h = _mp.getHeight(rc) / scale;

  const long t1 = clock();

  ALICEVISION_LOG_DEBUG("computeNormalMap rc: " << rc);

  CameraStruct camera;

  // Fill Camera Struct
  CudaDeviceMemoryPitched<CameraStructBase, 2> camsBasesDev(CudaSize<2>(1, 1));
  CudaHostMemoryHeap<CameraStructBase, 2>      camsBasesHst(CudaSize<2>(1, 1));
  camera.param_hst    = &camsBasesHst(0, 0);
  camera.param_dev    = &camsBasesDev(0, 0);
  camera.tex_rgba_hmh = 0; // texture remains invalid
  camera.pyramid      = 0; // downscaled images remain invalid
  camera.camId        = rc;
  cps_host_fillCamera(camsBasesHst(0, 0), rc, _mp, scale);
  camsBasesDev.copyFrom(camsBasesHst);


  CudaHostMemoryHeap<float3, 2> normalMap_hmh(CudaSize<2>(w, h));
  CudaHostMemoryHeap<float, 2> depthMap_hmh(CudaSize<2>(w, h));

  for (int i = 0; i < w * h; i++)
  {
    depthMap_hmh.getBuffer()[i] = (*depthMap)[i];
  }

  ps_computeNormalMap( normalMap_hmh, depthMap_hmh,
                       camera,
                       w, h, scale - 1,
                       _nImgsInGPUAtTime,
                       _scales, wsh, _mp.verbose, igammaC, igammaP);

  for (int i = 0; i < w * h; i++)
  {
    (*normalMap)[i].r = normalMap_hmh.getBuffer()[i].x;
    (*normalMap)[i].g = normalMap_hmh.getBuffer()[i].y;
    (*normalMap)[i].b = normalMap_hmh.getBuffer()[i].z;
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

int listCUDADevices(bool verbose)
{
    return ps_listCUDADevices(verbose);
}

} // namespace depthMap
} // namespace aliceVision
