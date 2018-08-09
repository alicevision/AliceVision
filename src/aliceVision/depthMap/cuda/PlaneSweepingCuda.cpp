// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "PlaneSweepingCuda.hpp"
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/mvsData/Matrix3x3.hpp>
#include <aliceVision/mvsData/Matrix3x4.hpp>
#include <aliceVision/mvsData/OrientedPoint.hpp>
#include <aliceVision/mvsData/SeedPoint.hpp>
#include <aliceVision/mvsUtils/common.hpp>
#include <aliceVision/mvsUtils/fileIO.hpp>
#include <aliceVision/depthMap/cuda/commonStructures.hpp>
#include <aliceVision/depthMap/cuda/planeSweeping/plane_sweeping_cuda.hpp>

#include <iostream>

namespace aliceVision {
namespace depthMap {

inline double getMaxGPUMemoryMB(int cudaDeviceNum, double defaultSize = 100.0)
{
    try
    {
        size_t avail;
        size_t total;
        if (cudaMemGetInfo(&avail, &total) != cudaSuccess)
            throw std::runtime_error("Cannot get memory information for CUDA gpu device " + std::to_string(cudaDeviceNum));
        return 0.9 * avail / (1024. * 1024.); // convert to MB and remove some margin
    }
    catch (const std::exception& e)
    {
        ALICEVISION_LOG_WARNING(e.what());
    }
    return defaultSize;
}

static void cps_fillCamera(cameraStruct* cam, int c, mvsUtils::MultiViewParams* mp, float** H, int scale)
{
    cam->scale = scale;

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
    Matrix3x3 K = scaleM * mp->KArr[c];

    Matrix3x3 iK = K.inverse();
    Matrix3x4 P = K * (mp->RArr[c] | (Point3d(0.0, 0.0, 0.0) - mp->RArr[c] * mp->CArr[c]));
    Matrix3x3 iP = mp->iRArr[c] * iK;

    cam->cam.C[0] = mp->CArr[c].x;
    cam->cam.C[1] = mp->CArr[c].y;
    cam->cam.C[2] = mp->CArr[c].z;

    cam->cam.P[0] = P.m11;
    cam->cam.P[1] = P.m21;
    cam->cam.P[2] = P.m31;
    cam->cam.P[3] = P.m12;
    cam->cam.P[4] = P.m22;
    cam->cam.P[5] = P.m32;
    cam->cam.P[6] = P.m13;
    cam->cam.P[7] = P.m23;
    cam->cam.P[8] = P.m33;
    cam->cam.P[9] = P.m14;
    cam->cam.P[10] = P.m24;
    cam->cam.P[11] = P.m34;

    cam->cam.iP[0] = iP.m11;
    cam->cam.iP[1] = iP.m21;
    cam->cam.iP[2] = iP.m31;
    cam->cam.iP[3] = iP.m12;
    cam->cam.iP[4] = iP.m22;
    cam->cam.iP[5] = iP.m32;
    cam->cam.iP[6] = iP.m13;
    cam->cam.iP[7] = iP.m23;
    cam->cam.iP[8] = iP.m33;

    cam->cam.R[0] = mp->RArr[c].m11;
    cam->cam.R[1] = mp->RArr[c].m21;
    cam->cam.R[2] = mp->RArr[c].m31;
    cam->cam.R[3] = mp->RArr[c].m12;
    cam->cam.R[4] = mp->RArr[c].m22;
    cam->cam.R[5] = mp->RArr[c].m32;
    cam->cam.R[6] = mp->RArr[c].m13;
    cam->cam.R[7] = mp->RArr[c].m23;
    cam->cam.R[8] = mp->RArr[c].m33;

    cam->cam.iR[0] = mp->iRArr[c].m11;
    cam->cam.iR[1] = mp->iRArr[c].m21;
    cam->cam.iR[2] = mp->iRArr[c].m31;
    cam->cam.iR[3] = mp->iRArr[c].m12;
    cam->cam.iR[4] = mp->iRArr[c].m22;
    cam->cam.iR[5] = mp->iRArr[c].m32;
    cam->cam.iR[6] = mp->iRArr[c].m13;
    cam->cam.iR[7] = mp->iRArr[c].m23;
    cam->cam.iR[8] = mp->iRArr[c].m33;

    cam->cam.K[0] = K.m11;
    cam->cam.K[1] = K.m21;
    cam->cam.K[2] = K.m31;
    cam->cam.K[3] = K.m12;
    cam->cam.K[4] = K.m22;
    cam->cam.K[5] = K.m32;
    cam->cam.K[6] = K.m13;
    cam->cam.K[7] = K.m23;
    cam->cam.K[8] = K.m33;

    cam->cam.iK[0] = iK.m11;
    cam->cam.iK[1] = iK.m21;
    cam->cam.iK[2] = iK.m31;
    cam->cam.iK[3] = iK.m12;
    cam->cam.iK[4] = iK.m22;
    cam->cam.iK[5] = iK.m32;
    cam->cam.iK[6] = iK.m13;
    cam->cam.iK[7] = iK.m23;
    cam->cam.iK[8] = iK.m33;

    if(cam->H != NULL)
    {
        delete cam->H;
        cam->H = NULL;
    }
    if(H != NULL)
    {
        cam->H = (*H);
    }

    ps_init_camera_vectors( cam->cam );
}

static void cps_fillCameraData( mvsUtils::ImagesCache* ic, cameraStruct* cam, int c, const mvsUtils::MultiViewParams* mp)
{
    ic->refreshData(c);

    if(cam->tex_rgba_hmh == nullptr)
        cam->tex_rgba_hmh = new CudaHostMemoryHeap<uchar4, 2>(CudaSize<2>(mp->getMaxImageWidth(), mp->getMaxImageHeight()));

    Pixel pix;
    for(pix.y = 0; pix.y < mp->getHeight(c); pix.y++)
    {
        for(pix.x = 0; pix.x < mp->getWidth(c); pix.x++)
        {
             uchar4& pix_rgba = ic->transposed ? (*cam->tex_rgba_hmh)(pix.x, pix.y) : (*cam->tex_rgba_hmh)(pix.y, pix.x);
             const rgb& pc = ic->getPixelValue(pix, c);
             pix_rgba.x = pc.r;
             pix_rgba.y = pc.g;
             pix_rgba.z = pc.b;
             pix_rgba.w = 0;
        }
    }
}

static void cps_updateCamH(cameraStruct* cam, float** H)
{
    if(cam->H != NULL)
    {
        delete cam->H;
        cam->H = NULL;
    }
    if(H != NULL)
    {
        cam->H = (*H);
    }
}

PlaneSweepingCuda::PlaneSweepingCuda(int _CUDADeviceNo, mvsUtils::ImagesCache* _ic, mvsUtils::MultiViewParams* _mp,
                                         mvsUtils::PreMatchCams* _pc, int _scales)
{
    CUDADeviceNo = _CUDADeviceNo;

    ic = _ic;
    scales = _scales;
    mp = _mp;
    pc = _pc;

    const int maxImageWidth = mp->getMaxImageWidth();
    const int maxImageHeight = mp->getMaxImageHeight();

    verbose = mp->verbose;

    double oneimagemb = 4.0 * (((double)(maxImageWidth * maxImageHeight) / 1024.0) / 1024.0);
    for(int scale = 2; scale <= scales; ++scale)
    {
        oneimagemb += 4.0 * (((double)((maxImageWidth / scale) * (maxImageHeight / scale)) / 1024.0) / 1024.0);
    }
    double maxmbGPU = getMaxGPUMemoryMB(CUDADeviceNo);
    nImgsInGPUAtTime = (int)(maxmbGPU / oneimagemb);
    nImgsInGPUAtTime = std::max(2, std::min(mp->ncams, nImgsInGPUAtTime));

    // TODO remove nbest ... now must be 1
    nbest = 1;
    nbestkernelSizeHalf = 1;

    doVizualizePartialDepthMaps = mp->_ini.get<bool>("grow.visualizePartialDepthMaps", false);
    useRcDepthsOrRcTcDepths = mp->_ini.get<bool>("grow.useRcDepthsOrRcTcDepths", false);

    minSegSize = mp->_ini.get<int>("fuse.minSegSize", 100);
    varianceWSH = mp->_ini.get<int>("global.varianceWSH", 4);

    subPixel = mp->_ini.get<bool>("global.subPixel", true);

    ALICEVISION_LOG_INFO("PlaneSweepingCuda:" << std::endl
        << "\t- max GPU Memory (MB): " << maxmbGPU << std::endl
        << "\t- nImgsInGPUAtTime: " << nImgsInGPUAtTime << std::endl
        << "\t- scales: " << scales << std::endl
        << "\t- subPixel: " << (subPixel ? "Yes" : "No") << std::endl
        << "\t- varianceWSH: " << varianceWSH);

    // allocate global on the device
    ps_deviceAllocate( nImgsInGPUAtTime, maxImageWidth, maxImageHeight, scales, CUDADeviceNo);

    _cams     .resize(nImgsInGPUAtTime);
    _camsRcs  .resize(nImgsInGPUAtTime);
    _camsTimes.resize(nImgsInGPUAtTime);

    for(int rc = 0; rc < nImgsInGPUAtTime; ++rc)
    {
        _cams[rc] = new cameraStruct;
        _cams[rc]->init();
        _camsRcs[rc] = -1;
        _camsTimes[rc] = clock();
    }
}

int PlaneSweepingCuda::addCam(int rc, float** H, int scale)
{
    // first is oldest
    int id = _camsRcs.indexOf(rc);
    if(id == -1)
    {
        ALICEVISION_LOG_INFO("Camera id " << rc << " was not in the cache, so it loads the image from disk.");
        // get oldest id
        int oldestId = _camsTimes.minValId();

        long t1 = clock();

        cps_fillCamera( _cams[oldestId], rc, mp, H, scale);
        cps_fillCameraData(ic, _cams[oldestId], rc, mp);
        // ps_deviceUpdateCam((CudaArray<uchar4, 2>**)ps_texs_arr, _cams[oldestId], oldestId,
        //                    CUDADeviceNo, nImgsInGPUAtTime, scales, mp->getMaxImageWidth(), mp->getMaxImageHeight(), varianceWSH);
        ps_deviceUpdateCam( _cams[oldestId], oldestId,
                            CUDADeviceNo, nImgsInGPUAtTime, scales,
                            mp->getMaxImageWidth(), mp->getMaxImageHeight(), varianceWSH );

        if(verbose)
            mvsUtils::printfElapsedTime(t1, "copy image from disk to GPU ");

        _camsRcs[oldestId] = rc;
        _camsTimes[oldestId] = clock();
        id = oldestId;
    }
    else
    {
        ALICEVISION_LOG_INFO("Camera id " << rc << " is in the cache.");
        cps_fillCamera( _cams[id], rc, mp, H, scale);
        // cps_fillCameraData( _cams[id], rc, mp, H, scales);
        // ps_deviceUpdateCam( _cams[id], id, scales);

        _camsTimes[id] = clock();
        cps_updateCamH( _cams[id], H);
    }
    return id;
}

PlaneSweepingCuda::~PlaneSweepingCuda(void)
{
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // deallocate global on the device
    ps_deviceDeallocate( CUDADeviceNo, nImgsInGPUAtTime, scales);

    for(int c = 0; c < _cams.size(); c++)
    {
        delete _cams[c]->tex_rgba_hmh;
        if( _cams[c]->H != NULL )
        {
            delete [] _cams[c]->H;
        }
        delete _cams[c];
    }

    mp = NULL;
}

void PlaneSweepingCuda::getMinMaxdepths(int rc, StaticVector<int>* tcams, float& minDepth, float& midDepth,
                                          float& maxDepth)
{
    StaticVector<SeedPoint>* seeds;
    mvsUtils::loadSeedsFromFile(&seeds, rc, mp, mvsUtils::EFileType::seeds);

    float minCamDist = (float)mp->_ini.get<double>("prematching.minCamDist", 0.0f);
    float maxCamDist = (float)mp->_ini.get<double>("prematching.maxCamDist", 15.0f);
    float maxDepthScale = (float)mp->_ini.get<double>("prematching.maxDepthScale", 1.5f);
    bool minMaxDepthDontUseSeeds = mp->_ini.get<bool>("prematching.minMaxDepthDontUseSeeds", false);

    if((seeds->empty()) || minMaxDepthDontUseSeeds)
    {
        minDepth = 0.0f;
        maxDepth = 0.0f;
        for(int c = 0; c < tcams->size(); c++)
        {
            int tc = (*tcams)[c];
            minDepth += (mp->CArr[rc] - mp->CArr[tc]).size() * minCamDist;
            maxDepth += (mp->CArr[rc] - mp->CArr[tc]).size() * maxCamDist;
        }
        minDepth /= (float)tcams->size();
        maxDepth /= (float)tcams->size();
        midDepth = (minDepth + maxDepth) / 2.0f;
    }
    else
    {
        OrientedPoint rcplane;
        rcplane.p = mp->CArr[rc];
        rcplane.n = mp->iRArr[rc] * Point3d(0.0, 0.0, 1.0);
        rcplane.n = rcplane.n.normalize();

        minDepth = std::numeric_limits<float>::max();
        maxDepth = -std::numeric_limits<float>::max();

        // StaticVector<sortedId> *sos = new StaticVector<sortedId>;
        // sos->reserve(seeds->size());
        // for (int i=0;i<seeds->size();i++) {
        //	sos->push_back(sortedId(i,pointPlaneDistance((*seeds)[i].op.p,rcplane.p,rcplane.n)));
        //};
        // qsort(&(*sos)[0],sos->size(),sizeof(sortedId),qsortCompareSortedIdAsc);
        // minDepth = (*sos)[(int)((float)sos->size()*0.1f)].value;
        // maxDepth = (*sos)[(int)((float)sos->size()*0.9f)].value;

        Point3d cg = Point3d(0.0f, 0.0f, 0.0f);
        for(int i = 0; i < seeds->size(); i++)
        {
            SeedPoint* sp = &(*seeds)[i];
            cg = cg + sp->op.p;
            float depth = pointPlaneDistance(sp->op.p, rcplane.p, rcplane.n);
            minDepth = std::min(minDepth, depth);
            maxDepth = std::max(maxDepth, depth);
        }
        cg = cg / (float)seeds->size();
        midDepth = pointPlaneDistance(cg, rcplane.p, rcplane.n);

        maxDepth = maxDepth * maxDepthScale;
    }

    delete seeds;
}

StaticVector<float>* PlaneSweepingCuda::getDepthsByPixelSize(int rc, float minDepth, float midDepth, float maxDepth,
                                                               int scale, int step, int maxDepthsHalf)
{
    float d = (float)step;

    OrientedPoint rcplane;
    rcplane.p = mp->CArr[rc];
    rcplane.n = mp->iRArr[rc] * Point3d(0.0, 0.0, 1.0);
    rcplane.n = rcplane.n.normalize();

    int ndepthsMidMax = 0;
    float maxdepth = midDepth;
    while((maxdepth < maxDepth) && (ndepthsMidMax < maxDepthsHalf))
    {
        Point3d p = rcplane.p + rcplane.n * maxdepth;
        float pixSize = mp->getCamPixelSize(p, rc, (float)scale * d);
        maxdepth += pixSize;
        ndepthsMidMax++;
    }

    int ndepthsMidMin = 0;
    float mindepth = midDepth;
    while((mindepth > minDepth) && (ndepthsMidMin < maxDepthsHalf * 2 - ndepthsMidMax))
    {
        Point3d p = rcplane.p + rcplane.n * mindepth;
        float pixSize = mp->getCamPixelSize(p, rc, (float)scale * d);
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
        pixSize = mp->getCamPixelSize(p, rc, (float)scale * d);
        depth += pixSize;
        ndepths++;
    }

    StaticVector<float>* out = new StaticVector<float>;
    out->reserve(ndepths);

    // fill
    depth = mindepth;
    pixSize = 1.0f;
    ndepths = 0;
    while((depth < maxdepth) && (pixSize > 0.0f) && (ndepths < 2 * maxDepthsHalf))
    {
        out->push_back(depth);
        Point3d p = rcplane.p + rcplane.n * depth;
        pixSize = mp->getCamPixelSize(p, rc, (float)scale * d);
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
    rcplane.p = mp->CArr[rc];
    rcplane.n = mp->iRArr[rc] * Point3d(0.0, 0.0, 1.0);
    rcplane.n = rcplane.n.normalize();

    Point2d rmid = Point2d((float)mp->getWidth(rc) / 2.0f, (float)mp->getHeight(rc) / 2.0f);
    Point2d pFromTar, pToTar; // segment of epipolar line of the principal point of the rc camera to the tc camera
    getTarEpipolarDirectedLine(&pFromTar, &pToTar, rmid, rc, tc, mp);

    int allDepths = static_cast<int>((pToTar - pFromTar).size());
    if(verbose == true)
    {
        ALICEVISION_LOG_DEBUG("allDepths: " << allDepths);
    }

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
        if(triangulateMatch(p, rmid, tpix, rc, tc, mp)) // triangulate principal point from rc with tpix
        {
            float depth = orientedPointPlaneDistance(p, rcplane.p, rcplane.n); // todo: can compute the distance to the camera (as it's the principal point it's the same)
            if( mp->isPixelInImage(tpix, tc)
                && (depth > 0.0f)
                && checkPair(p, rc, tc, mp, pc->minang, pc->maxang) )
            {
                cg = cg + tpix;
                cg3 = cg3 + p;
                ncg++;
            }
        }
    }
    if(ncg == 0)
    {
        return new StaticVector<float>;
    }
    cg = cg / (float)ncg;
    cg3 = cg3 / (float)ncg;
    allDepths = ncg;

    if(verbose == true)
    {
        ALICEVISION_LOG_DEBUG("All correct depths: " << allDepths);
    }

    Point2d midpoint = cg;
    if(midDepth > 0.0f)
    {
        Point3d midPt = rcplane.p + rcplane.n * midDepth;
        mp->getPixelFor3DPoint(&midpoint, midPt, tc);
    }

    // compute the direction
    float direction = 1.0f;
    {
        Point3d p;
        if(!triangulateMatch(p, rmid, midpoint, rc, tc, mp))
        {
            StaticVector<float>* out = new StaticVector<float>;
            return out;
        }

        float depth = orientedPointPlaneDistance(p, rcplane.p, rcplane.n);

        if(!triangulateMatch(p, rmid, midpoint + pixelVect, rc, tc, mp))
        {
            StaticVector<float>* out = new StaticVector<float>;
            return out;
        }

        float depthP1 = orientedPointPlaneDistance(p, rcplane.p, rcplane.n);
        if(depth > depthP1)
        {
            direction = -1.0f;
        }
    }

    StaticVector<float>* out1 = new StaticVector<float>;
    out1->reserve(2 * maxDepthsHalf);

    Point2d tpix = midpoint;
    float depthOld = -1.0f;
    int istep = 0;
    bool ok = true;

    // compute depths for all pixels from the middle point to on one side of the epipolar line
    while((out1->size() < maxDepthsHalf) && (mp->isPixelInImage(tpix, tc) == true) && (ok == true))
    {
        tpix = tpix + pixelVect * direction;

        Point3d refvect = mp->iCamArr[rc] * rmid;
        Point3d tarvect = mp->iCamArr[tc] * tpix;
        float rptpang = angleBetwV1andV2(refvect, tarvect);

        Point3d p;
        ok = triangulateMatch(p, rmid, tpix, rc, tc, mp);

        float depth = orientedPointPlaneDistance(p, rcplane.p, rcplane.n);
        if (mp->isPixelInImage(tpix, tc)
            && (depth > 0.0f) && (depth > depthOld)
            && checkPair(p, rc, tc, mp, pc->minang, pc->maxang)
            && (rptpang > pc->minang)  // WARNING if vects are near parallel thaen this results to strange angles ...
            && (rptpang < pc->maxang)) // this is the propper angle ... beacause is does not depend on the triangluated p
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

    StaticVector<float>* out2 = new StaticVector<float>;
    out2->reserve(2 * maxDepthsHalf);
    tpix = midpoint;
    istep = 0;
    ok = true;

    // compute depths for all pixels from the middle point to the other side of the epipolar line
    while((out2->size() < maxDepthsHalf) && (mp->isPixelInImage(tpix, tc) == true) && (ok == true))
    {
        Point3d refvect = mp->iCamArr[rc] * rmid;
        Point3d tarvect = mp->iCamArr[tc] * tpix;
        float rptpang = angleBetwV1andV2(refvect, tarvect);

        Point3d p;
        ok = triangulateMatch(p, rmid, tpix, rc, tc, mp);

        float depth = orientedPointPlaneDistance(p, rcplane.p, rcplane.n);
        if(mp->isPixelInImage(tpix, tc)
            && (depth > 0.0f) && (depth < depthOld) 
            && checkPair(p, rc, tc, mp, pc->minang, pc->maxang)
            && (rptpang > pc->minang)  // WARNING if vects are near parallel thaen this results to strange angles ...
            && (rptpang < pc->maxang)) // this is the propper angle ... beacause is does not depend on the triangluated p
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
    StaticVector<float>* out = new StaticVector<float>;
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
    if((*out)[0] > (*out)[out->size() - 1])
    {
        StaticVector<float>* outTmp = new StaticVector<float>;
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

    if(verbose == true)
    {
        ALICEVISION_LOG_DEBUG("used depths: " << out->size());
    }

    return out;
}

bool PlaneSweepingCuda::smoothDepthMap(StaticVector<float>* depthMap, int rc, int scale, float igammaC, float igammaP,
                                         int wsh)
{
    int w = mp->getWidth(rc) / scale;
    int h = mp->getHeight(rc) / scale;

    long t1 = clock();

    if(verbose)
        ALICEVISION_LOG_DEBUG("smoothDepthMap rc: " << rc);
    int camsids = addCam(rc, NULL, scale);

    cameraStruct ttcams;
    ttcams = *_cams[camsids];
    ttcams.camId = camsids;
    ttcams.rc = rc;

    CudaHostMemoryHeap<float, 2> depthMap_hmh(CudaSize<2>(w, h));

    for(int i = 0; i < w * h; i++)
    {
        depthMap_hmh.getBuffer()[i] = (*depthMap)[i];
    }

    // ps_smoothDepthMap((CudaArray<uchar4, 2>**)ps_texs_arr, &depthMap_hmh, ttcams, w, h, scale - 1, CUDADeviceNo,
    //                   nImgsInGPUAtTime, scales, wsh, verbose, igammaC, igammaP);
    ps_smoothDepthMap( &depthMap_hmh, ttcams, w, h, scale - 1, CUDADeviceNo,
                       // nImgsInGPUAtTime,
                       scales, wsh, verbose, igammaC, igammaP);

    for(int i = 0; i < w * h; i++)
    {
        (*depthMap)[i] = depthMap_hmh.getBuffer()[i];
    }

    if(verbose)
        mvsUtils::printfElapsedTime(t1);

    return true;
}

bool PlaneSweepingCuda::filterDepthMap(StaticVector<float>* depthMap, int rc, int scale, float igammaC,
                                         float minCostThr, int wsh)
{
    int w = mp->getWidth(rc) / scale;
    int h = mp->getHeight(rc) / scale;

    long t1 = clock();

    if(verbose)
        ALICEVISION_LOG_DEBUG("filterDepthMap rc: " << rc);
    int camsids = addCam(rc, NULL, scale);

    cameraStruct ttcams = *_cams[camsids];
    ttcams.camId = camsids;
    ttcams.rc = rc;

    CudaHostMemoryHeap<float, 2> depthMap_hmh(CudaSize<2>(w, h));

    for(int i = 0; i < w * h; i++)
    {
        depthMap_hmh.getBuffer()[i] = (*depthMap)[i];
    }

    // ps_filterDepthMap((CudaArray<uchar4, 2>**)ps_texs_arr, &depthMap_hmh, ttcams, w, h, scale - 1, CUDADeviceNo,
    //                   nImgsInGPUAtTime, scales, wsh, verbose, igammaC, minCostThr);
    ps_filterDepthMap( &depthMap_hmh, ttcams, w, h, scale - 1, CUDADeviceNo,
                       // nImgsInGPUAtTime,
                       scales, wsh, verbose, igammaC, minCostThr );

    for(int i = 0; i < w * h; i++)
    {
        (*depthMap)[i] = depthMap_hmh.getBuffer()[i];
    }

    if(verbose)
        mvsUtils::printfElapsedTime(t1);

    return true;
}

bool PlaneSweepingCuda::computeNormalMap(StaticVector<float>* depthMap, StaticVector<Color>* normalMap, int rc,
                                           int scale, float igammaC, float igammaP, int wsh)
{
    int w = mp->getWidth(rc) / scale;
    int h = mp->getHeight(rc) / scale;

    long t1 = clock();

    if(verbose)
        ALICEVISION_LOG_DEBUG("computeNormalMap rc: " << rc);

    int camsids = addCam(rc, NULL, scale);

    cameraStruct ttcams = *_cams[camsids];
    ttcams.camId = camsids;
    ttcams.rc = rc;

    CudaHostMemoryHeap<float3, 2> normalMap_hmh(CudaSize<2>(w, h));
    CudaHostMemoryHeap<float, 2> depthMap_hmh(CudaSize<2>(w, h));

    for(int i = 0; i < w * h; i++)
    {
        depthMap_hmh.getBuffer()[i] = (*depthMap)[i];
    }

    // ps_computeNormalMap((CudaArray<uchar4, 2>**)ps_texs_arr, &normalMap_hmh, &depthMap_hmh, ttcams, w, h, scale - 1,
    //                     CUDADeviceNo, nImgsInGPUAtTime, scales, wsh, verbose, igammaC, igammaP);
    ps_computeNormalMap( &normalMap_hmh, &depthMap_hmh, ttcams, w, h, scale - 1,
                         CUDADeviceNo,
                         // nImgsInGPUAtTime,
                         scales, wsh, verbose, igammaC, igammaP);

    for(int i = 0; i < w * h; i++)
    {
        (*normalMap)[i].r = normalMap_hmh.getBuffer()[i].x;
        (*normalMap)[i].g = normalMap_hmh.getBuffer()[i].y;
        (*normalMap)[i].b = normalMap_hmh.getBuffer()[i].z;
    }

    if(verbose)
        mvsUtils::printfElapsedTime(t1);

    return true;
}

void PlaneSweepingCuda::alignSourceDepthMapToTarget(StaticVector<float>* sourceDepthMap,
                                                      StaticVector<float>* targetDepthMap, int rc, int scale,
                                                      float igammaC, int wsh, float maxPixelSizeDist)
{
    int w = mp->getWidth(rc) / scale;
    int h = mp->getHeight(rc) / scale;

    long t1 = clock();

    if(verbose)
        ALICEVISION_LOG_DEBUG("alignSourceDepthMapToTarget rc: " << rc);

    int camsids = addCam(rc, NULL, scale);

    cameraStruct ttcams = *_cams[camsids];
    ttcams.camId = camsids;
    ttcams.rc = rc;

    CudaHostMemoryHeap<float, 2> sourceDepthMap_hmh(CudaSize<2>(w, h));
    CudaHostMemoryHeap<float, 2> targetDepthMap_hmh(CudaSize<2>(w, h));

    for(int i = 0; i < w * h; i++)
    {
        sourceDepthMap_hmh.getBuffer()[i] = (*sourceDepthMap)[i];
        targetDepthMap_hmh.getBuffer()[i] = (*targetDepthMap)[i];
    }

    // ps_alignSourceDepthMapToTarget((CudaArray<uchar4, 2>**)ps_texs_arr, &sourceDepthMap_hmh, &targetDepthMap_hmh,
    //                                ttcams, w, h, scale - 1, CUDADeviceNo, nImgsInGPUAtTime, scales, wsh, verbose,
    //                                igammaC, maxPixelSizeDist);
    ps_alignSourceDepthMapToTarget( &sourceDepthMap_hmh, &targetDepthMap_hmh,
                                    ttcams, w, h, scale - 1,
                                    CUDADeviceNo,
                                    // nImgsInGPUAtTime,
                                    scales, wsh, verbose,
                                    igammaC, maxPixelSizeDist);

    for(int i = 0; i < w * h; i++)
    {
        (*sourceDepthMap)[i] = sourceDepthMap_hmh.getBuffer()[i];
    }

    if(verbose)
        mvsUtils::printfElapsedTime(t1);
}

bool PlaneSweepingCuda::refineDepthMapReproject(StaticVector<float>* depthMap, StaticVector<float>* simMap, int rc,
                                                  int tc, int wsh, float gammaC, float gammaP, float simThr, int niters,
                                                  bool moveByTcOrRc)
{
    int scale = 1;
    int w = mp->getWidth(rc);
    int h = mp->getHeight(rc);

    long t1 = clock();

    StaticVector<int> camsids;
    camsids.reserve(2);
    camsids.push_back(addCam(rc, NULL, scale));

    if(verbose)
        ALICEVISION_LOG_DEBUG("\t- rc: " << rc << std::endl << "\t- tcams: " << tc);

    camsids.push_back(addCam(tc, NULL, scale));

    cameraStruct** ttcams = new cameraStruct*[camsids.size()];
    for(int i = 0; i < camsids.size(); i++)
    {
        ttcams[i] = _cams[camsids[i]];
        ttcams[i]->camId = camsids[i];
        if(i == 0)
        {
            ttcams[i]->rc = rc;
        }
        else
        {
            ttcams[i]->rc = tc;
        }
    }

    // sweep
    CudaHostMemoryHeap<float, 2> osim_hmh(CudaSize<2>(w, h));
    CudaHostMemoryHeap<float, 2> odpt_hmh(CudaSize<2>(w, h));
    CudaHostMemoryHeap<float, 2> depthMap_hmh(CudaSize<2>(w, h));
    for(int i = 0; i < w * h; i++)
    {
        depthMap_hmh.getBuffer()[i] = (*depthMap)[i];
    }

    CudaHostMemoryHeap<uchar4, 2> otimg_hmh(CudaSize<2>(w, h));

    // ps_refineDepthMapReproject((CudaArray<uchar4, 2>**)ps_texs_arr, &otimg_hmh, &osim_hmh, &odpt_hmh, depthMap_hmh,
    //                            ttcams, camsids.size(), w, h, scale - 1, CUDADeviceNo, nImgsInGPUAtTime, scales,
    //                            verbose, wsh, gammaC, gammaP, simThr, niters, moveByTcOrRc);
    ps_refineDepthMapReproject( &otimg_hmh, &osim_hmh, &odpt_hmh, depthMap_hmh,
                                ttcams, camsids.size(), w, h, scale - 1,
                                CUDADeviceNo, nImgsInGPUAtTime, scales,
                                verbose, wsh, gammaC, gammaP, simThr, niters, moveByTcOrRc );

    for(int i = 0; i < w * h; i++)
    {
        float depth = odpt_hmh.getBuffer()[i];
        (*depthMap)[i] = depth;
        // if (depth>0.0f) {
        //	(*simMap)[i] = -1.0f;
        //};
        (*simMap)[i] = osim_hmh.getBuffer()[i];
    }

    for(int i = 0; i < camsids.size(); i++)
    {
        ttcams[i] = NULL;
    }
    delete[] ttcams;

    if(verbose)
        mvsUtils::printfElapsedTime(t1);

    return true;
}

/*
computeRcTcPhotoErrMapReproject
    ps_computeRcTcPhotoErrMapReproject
*/

bool PlaneSweepingCuda::computeSimMapForRcTcDepthMap(StaticVector<float>* oSimMap, StaticVector<float>* rcTcDepthMap,
                                                       int rc, int tc, int wsh, float gammaC, float gammaP,
                                                       float epipShift)
{
    int scale = 1;
    int w = mp->getWidth(rc);
    int h = mp->getHeight(rc);

    long t1 = clock();

    StaticVector<int> camsids;
    camsids.reserve(2);
    camsids.push_back(addCam(rc, NULL, scale));

    if(verbose)
        ALICEVISION_LOG_DEBUG("\t- rc: " << rc << std::endl << "\t- tcams: " << tc);

    camsids.push_back(addCam(tc, NULL, scale));

    cameraStruct** ttcams = new cameraStruct*[camsids.size()];
    for(int i = 0; i < camsids.size(); i++)
    {
        ttcams[i] = _cams[camsids[i]];
        ttcams[i]->camId = camsids[i];
        if(i == 0)
        {
            ttcams[i]->rc = rc;
        }
        else
        {
            ttcams[i]->rc = tc;
        }
    }

    // sweep
    CudaHostMemoryHeap<float, 2> simMap_hmh(CudaSize<2>(w, h));
    CudaHostMemoryHeap<float, 2> rcTcDepthMap_hmh(CudaSize<2>(w, h));
    for(int i = 0; i < w * h; i++)
    {
        rcTcDepthMap_hmh.getBuffer()[i] = (*rcTcDepthMap)[i];
    }

    // ps_computeSimMapForRcTcDepthMap((CudaArray<uchar4, 2>**)ps_texs_arr, &simMap_hmh, rcTcDepthMap_hmh, ttcams,
    //                                 camsids.size(), w, h, scale - 1, CUDADeviceNo, nImgsInGPUAtTime, scales, verbose,
    //                                 wsh, gammaC, gammaP, epipShift);
    ps_computeSimMapForRcTcDepthMap( &simMap_hmh, rcTcDepthMap_hmh, ttcams,
                                     camsids.size(), w, h, scale - 1,
                                     CUDADeviceNo, nImgsInGPUAtTime, scales, verbose,
                                     wsh, gammaC, gammaP, epipShift );

    for(int j = 0; j < w * h; j++)
    {
        (*oSimMap)[j] = simMap_hmh.getBuffer()[j];
    }

    for(int i = 0; i < camsids.size(); i++)
    {
        ttcams[i] = NULL;
    }
    delete[] ttcams;

    if(verbose)
        mvsUtils::printfElapsedTime(t1);

    return true;
}

bool PlaneSweepingCuda::refineRcTcDepthMap(bool useTcOrRcPixSize, int nStepsToRefine, StaticVector<float>* simMap,
                                             StaticVector<float>* rcDepthMap, int rc, int tc, int scale, int wsh,
                                             float gammaC, float gammaP, float epipShift, int xFrom, int wPart)
{
    // int w = mp->getWidth(rc)/scale;
    int w = wPart;
    int h = mp->getHeight(rc) / scale;

    long t1 = clock();

    StaticVector<int> camsids;
    camsids.reserve(2);
    camsids.push_back(addCam(rc, NULL, scale));

    if(verbose)
        ALICEVISION_LOG_DEBUG("\t- rc: " << rc << std::endl << "\t- tcams: " << tc);

    camsids.push_back(addCam(tc, NULL, scale));

    cameraStruct** ttcams = new cameraStruct*[camsids.size()];
    for(int i = 0; i < camsids.size(); i++)
    {
        ttcams[i] = _cams[camsids[i]];
        ttcams[i]->camId = camsids[i];
        if(i == 0)
        {
            ttcams[i]->rc = rc;
        }
        else
        {
            ttcams[i]->rc = tc;
        }
    }

    // sweep
    // ps_refineRcDepthMap(
    //     (CudaArray<uchar4, 2>**)ps_texs_arr,
    //     simMap->getDataWritable().data(),
    //     rcDepthMap->getDataWritable().data(),
    //     nStepsToRefine,
    //     ttcams, camsids.size(), w, h, mp->getWidth(rc) / scale,
    //     mp->getHeight(rc) / scale, scale - 1, CUDADeviceNo, nImgsInGPUAtTime, scales, verbose, wsh,
    //     gammaC, gammaP, epipShift, useTcOrRcPixSize, xFrom);
    ps_refineRcDepthMap(
        simMap->getDataWritable().data(),
        rcDepthMap->getDataWritable().data(),
        nStepsToRefine,
        ttcams, camsids.size(), w, h, mp->getWidth(rc) / scale,
        mp->getHeight(rc) / scale, scale - 1, CUDADeviceNo, nImgsInGPUAtTime, scales, verbose, wsh,
        gammaC, gammaP, epipShift, useTcOrRcPixSize, xFrom);

    /*
    CudaHostMemoryHeap<float, 3> tmpSimVolume_hmh(CudaSize<3>(201, 201, nStepsToRefine));

    ps_refineRcTcDepthMapSGM(
            &tmpSimVolume_hmh,
            &simMap_hmh,
            &rcDepthMap_hmh,
            nStepsToRefine,
            rcDepthMap_hmh,
            ttcams, camsids.size(),
            w, h,
            scale-1, scales,
            verbose, wsh, gammaC, gammaP, epipShift,
            0.0001f, 1.0f
    );
    */

    for(int i = 0; i < camsids.size(); i++)
    {
        ttcams[i] = NULL;
    }
    delete[] ttcams;

    if(verbose)
        mvsUtils::printfElapsedTime(t1);

    return true;
}

float PlaneSweepingCuda::sweepPixelsToVolume(int nDepthsToSearch, StaticVector<unsigned char>* volume, int volDimX,
                                               int volDimY, int volDimZ, int volStepXY, int volLUX, int volLUY,
                                               int volLUZ, StaticVector<float>* depths, int rc, int wsh, float gammaC,
                                               float gammaP, StaticVector<Voxel>* pixels, int scale, int step,
                                               StaticVector<int>* tcams, float epipShift) {
    if(verbose)
        ALICEVISION_LOG_DEBUG("sweepPixelsVolume:" << std::endl
                              << "\t- scale: " << scale << std::endl
                              << "\t- step: " << step << std::endl
                              << "\t- npixels: " << pixels->size() << std::endl
                              << "\t- volStepXY: " << volStepXY << std::endl
                              << "\t- volDimX: " << volDimX << std::endl
                              << "\t- volDimY: " << volDimY << std::endl
                              << "\t- volDimZ: " << volDimZ);

    int w = mp->getWidth(rc) / scale;
    int h = mp->getHeight(rc) / scale;

    long t1 = clock();

    if((tcams->size() == 0) || (pixels->size() == 0)) {
        return -1.0f;
    }

    StaticVector<int> camsids;
    camsids.reserve(tcams->size() + 1);
    camsids.push_back(addCam(rc, NULL, scale));
    if(verbose)
        ALICEVISION_LOG_DEBUG("rc: " << rc << std::endl << "tcams: ");

    for(int c = 0; c < tcams->size(); ++c)
    {
        int tc = (*tcams)[c];
        if(verbose)
            ALICEVISION_LOG_DEBUG("\t- " << tc);
        camsids.push_back(addCam(tc, NULL, scale));
    }

    cameraStruct **ttcams = new cameraStruct *[camsids.size()];
    for(int i = 0; i < camsids.size(); i++) {
        ttcams[i] = _cams[camsids[i]];
        ttcams[i]->camId = camsids[i];
        if (i == 0) {
            ttcams[i]->rc = rc;
        } else {
            ttcams[i]->rc = (*tcams)[i - 1];
        }
    }

    // hard-coded for CC 3.0 and 3.5
    // int slicesAtTime = std::min(pixels->size(), 65000 );
    // dynamically extracted slice size from cudaDeviceProp structure
    int slicesAtTime = std::min( pixels->size(), ps_getTexture2DLinear() );
    // hard-coded value
    // int slicesAtTime = std::min(pixels->size(), 4096*16); //TODO
    // int slicesAtTime = 480/scale;
    // int slicesAtTime = pixels->size();

    int npixs = pixels->size();
    int ntimes = npixs / slicesAtTime + 1;
    ALICEVISION_LOG_INFO("Split processing in N blocks (ntimes): " << ntimes);
    CudaHostMemoryHeap<int, 2> volPixs_hmh_x(CudaSize<2>(slicesAtTime, ntimes));
    CudaHostMemoryHeap<int, 2> volPixs_hmh_y(CudaSize<2>(slicesAtTime, ntimes));
    CudaHostMemoryHeap<int, 2> volPixs_hmh_z(CudaSize<2>(slicesAtTime, ntimes));

    int *volPixs_x = volPixs_hmh_x.getBuffer();
    int *volPixs_y = volPixs_hmh_y.getBuffer();
    int *volPixs_z = volPixs_hmh_z.getBuffer();
    const Voxel *_pixels = pixels->getData().data();

    for(int y = 0; y < ntimes; ++y)
    {
        for(int x = 0; x < slicesAtTime; ++x)
        {
            int index = y * slicesAtTime + x;
            if(index >= npixs)
                break;
            const Voxel &pixel = _pixels[index];
            volPixs_x[index] = pixel.x;
            volPixs_y[index] = pixel.y;
            volPixs_z[index] = pixel.z;
            // volPixs_w[index] = 1;
        }
    }

    CudaHostMemoryHeap<float, 2> depths_hmh(CudaSize<2>(depths->size(), 1));

    for(int x = 0; x < depths->size(); x++)
    {
        depths_hmh(x, 0) = (*depths)[x];
    }

    // sweep
    // float volumeMBinGPUMem = ps_planeSweepingGPUPixelsVolume(
    //     (CudaArray<uchar4, 2>**)ps_texs_arr, volume->getDataWritable().data(), ttcams, camsids.size(), w, h, volStepXY, volDimX, volDimY,
    //     volDimZ, volLUX, volLUY, volLUZ, volPixs_hmh, depths_hmh, nDepthsToSearch, slicesAtTime, ntimes, npixs, wsh,
    //     nbestkernelSizeHalf, depths->size(), scale - 1, CUDADeviceNo, nImgsInGPUAtTime, scales, verbose, false, nbest,
    //     true, gammaC, gammaP, subPixel, epipShift);
    float volumeMBinGPUMem = ps_planeSweepingGPUPixelsVolume(
        volume->getDataWritable().data(), ttcams, camsids.size(), w, h, volStepXY, volDimX, volDimY,
        volDimZ, volLUX, volLUY, volLUZ,
	volPixs_hmh_x,
	volPixs_hmh_y,
	volPixs_hmh_z,
	depths_hmh, nDepthsToSearch, slicesAtTime, ntimes, npixs, wsh,
        nbestkernelSizeHalf, depths->size(), scale - 1,
        CUDADeviceNo, nImgsInGPUAtTime, scales, verbose, false, nbest,
        true, gammaC, gammaP, subPixel, epipShift);

    for(int i = 0; i < camsids.size(); i++)
    {
        ttcams[i] = NULL;
    }
    delete[] ttcams;

    if(verbose)
        mvsUtils::printfElapsedTime(t1);

    return volumeMBinGPUMem;
}

/**
 * @param[inout] volume input similarity volume (after Z reduction)
 */
bool PlaneSweepingCuda::SGMoptimizeSimVolume(int rc, StaticVector<unsigned char>* volume, 
                                               int volDimX, int volDimY, int volDimZ, 
                                               int volStepXY, int volLUX, int volLUY, int scale,
                                               unsigned char P1, unsigned char P2)
{
    if(verbose)
        ALICEVISION_LOG_DEBUG("SGM optimizing volume:" << std::endl
                              << "\t- volDimX: " << volDimX << std::endl
                              << "\t- volDimY: " << volDimY << std::endl
                              << "\t- volDimZ: " << volDimZ);

    long t1 = clock();

    // ps_SGMoptimizeSimVolume((CudaArray<uchar4, 2>**)ps_texs_arr, _cams[addCam(rc, NULL, scale)],
    //                         volume->getDataWritable().data(), volDimX, volDimY, volDimZ, volStepXY, volLUX, volLUY, verbose, P1, P2, scale - 1, // TODO: move the '- 1' inside the function
    //                         CUDADeviceNo, nImgsInGPUAtTime, scales);
    ps_SGMoptimizeSimVolume( _cams[addCam(rc, NULL, scale)],
                             volume->getDataWritable().data(),
                             volDimX, volDimY, volDimZ, volStepXY, volLUX, volLUY,
                             verbose, P1, P2, scale - 1, // TODO: move the '- 1' inside the function
                             CUDADeviceNo, nImgsInGPUAtTime, scales );

    if(verbose)
        mvsUtils::printfElapsedTime(t1);

    return true;
}

// make_float3(avail,total,used)
Point3d PlaneSweepingCuda::getDeviceMemoryInfo()
{
    float3 dmif3 = ps_getDeviceMemoryInfo();
    return Point3d(dmif3.x, dmif3.y, dmif3.z);
}

/*
// bool PlaneSweepingCuda::transposeVolume(StaticVector<unsigned char>* volume, const voxel& dimIn, const voxel& dimTrn,
//                                           voxel& dimOut)
// {
//     if(verbose)
//         printf("\n transposing volume volDimX %i volDimY %i volDimZ %i \n", dimIn.x, dimIn.y, dimIn.z);
//     long t1 = clock();
// 
//     CudaHostMemoryHeap<unsigned char, 3> ivol_hmh(CudaSize<3>(dimIn.x, dimIn.y, dimIn.z));
// 
// #pragma omp parallel for
//     for(int z = 0; z < dimIn.z; z++)
//     {
//         for(int y = 0; y < dimIn.y; y++)
//         {
//             for(int x = 0; x < dimIn.x; x++)
//             {
//                 ivol_hmh.getBuffer()[z * dimIn.x * dimIn.y + y * dimIn.x + x] =
//                     (*volume)[z * dimIn.x * dimIn.y + y * dimIn.x + x];
//             }
//         }
//     }
// 
//     dimOut = voxel(dimIn.m[dimTrn.x], dimIn.m[dimTrn.y], dimIn.m[dimTrn.z]);
// 
//     CudaHostMemoryHeap<unsigned char, 3> ovol_hmh(CudaSize<3>(dimOut.x, dimOut.y, dimOut.z));
// 
//     ps_transposeVolume(&ovol_hmh, &ivol_hmh, dimIn.x, dimIn.y, dimIn.z, dimTrn.x, dimTrn.y, dimTrn.z, verbose);
// 
// #pragma omp parallel for
//     for(int z = 0; z < dimOut.z; z++)
//     {
//         for(int y = 0; y < dimOut.y; y++)
//         {
//             for(int x = 0; x < dimOut.x; x++)
//             {
//                 (*volume)[z * dimOut.x * dimOut.y + y * dimOut.x + x] =
//                     ovol_hmh.getBuffer()[z * dimOut.x * dimOut.y + y * dimOut.x + x];
//             }
//         }
//     }
// 
//     if(verbose)
//         printfElapsedTime(t1);
// 
//     return true;
// }
// 
// bool PlaneSweepingCuda::computeRcVolumeForRcTcsDepthSimMaps(
//     StaticVector<unsigned int>* volume, StaticVector<StaticVector<Point2d>*>* rcTcsDepthSimMaps, int volDimX,
//     int volDimY, int volDimZ, int volStepXY, StaticVector<float>* depths, int scale, int step,
//     StaticVector<int>* rtcams, StaticVector<Point2d>* rtCamsMinMaxFpDepths, const float maxTcRcPixSizeInVoxRatio,
//     bool considerNegativeDepthAsInfinity)
// {
//     if(verbose)
//         printf(
//             "\n computeRcVolumeForTcDepthMap scale %i step %i volStepXY %i volDimX %i volDimY %i volDimZ %i cams %i\n",
//             scale, step, volStepXY, volDimX, volDimY, volDimZ, rtcams->size());
// 
//     int w = mp->getMaxImageWidth() / scale;
//     int h = mp->getMaxImageHeight() / scale;
// 
//     long t1 = clock();
// 
//     if(rtcams->size() == 0)
//     {
//         return false;
//     }
// 
//     cameraStruct** ttcams = new cameraStruct*[rtcams->size()];
//     for(int i = 0; i < rtcams->size(); i++)
//     {
//         cameraStruct* tcam = new cameraStruct;
//         tcam->init();
//         cps_fillCamera(tcam, (*rtcams)[i], mp, NULL, scale);
//         ttcams[i] = tcam;
//     }
// 
//     CudaHostMemoryHeap<float, 2> depths_hmh(CudaSize<2>(depths->size(), 1));
// #pragma omp parallel for
//     for(int i = 0; i < depths->size(); i++)
//     {
//         depths_hmh.getBuffer()[i] = (*depths)[i];
//     }
// 
//     CudaHostMemoryHeap<float2, 2>** rcTcsDepthSimMaps_hmh = new CudaHostMemoryHeap<float2, 2>*[rtcams->size()];
//     for(int c = 0; c < rtcams->size(); c++)
//     {
//         rcTcsDepthSimMaps_hmh[c] = new CudaHostMemoryHeap<float2, 2>(CudaSize<2>(w / volStepXY, h / volStepXY));
// #pragma omp parallel for
//         for(int i = 0; i < (w / volStepXY) * (h / volStepXY); i++)
//         {
//             rcTcsDepthSimMaps_hmh[c]->getBuffer()[i].x = (*(*rcTcsDepthSimMaps)[c])[i].x;
//             rcTcsDepthSimMaps_hmh[c]->getBuffer()[i].y = (*(*rcTcsDepthSimMaps)[c])[i].y;
//         }
//     }
// 
//     int nZparts = 4;
//     int volDimZpart = (int)ceil((float)volDimZ / (float)nZparts);
//     CudaHostMemoryHeap<unsigned int, 3>** vols_hmh = new CudaHostMemoryHeap<unsigned int, 3>*[nZparts];
//     for(int zPart = 0; zPart < nZparts; zPart++)
//     {
//         vols_hmh[zPart] = new CudaHostMemoryHeap<unsigned int, 3>(CudaSize<3>(volDimX, volDimY, volDimZpart));
//     }
// 
//     float2* camsMinMaxFpDepths = new float2[rtcams->size()];
//     for(int c = 0; c < rtcams->size(); c++)
//     {
//         camsMinMaxFpDepths[c].x = (*rtCamsMinMaxFpDepths)[c].x;
//         camsMinMaxFpDepths[c].y = (*rtCamsMinMaxFpDepths)[c].y;
//     }
// 
//     // sweep
//     ps_computeRcVolumeForTcDepthSimMaps(vols_hmh, nZparts, volDimZpart, ttcams, rtcams->size(), camsMinMaxFpDepths, w,
//                                         h, volStepXY, volDimX, volDimY, volDimZ, depths_hmh, depths->size(), scale - 1,
//                                         CUDADeviceNo, nImgsInGPUAtTime, scales, rcTcsDepthSimMaps_hmh, verbose,
//                                         maxTcRcPixSizeInVoxRatio, considerNegativeDepthAsInfinity);
// 
//     for(int zPart = 0; zPart < nZparts; zPart++)
//     {
// #pragma omp parallel for
//         for(int z = 0; z < std::min(volDimZ - zPart * volDimZpart, volDimZpart); z++)
//         {
//             for(int y = 0; y < volDimY; y++)
//             {
//                 for(int x = 0; x < volDimX; x++)
//                 {
//                     (*volume)[(zPart * volDimZpart + z) * volDimX * volDimY + y * volDimX + x] =
//                         vols_hmh[zPart]->getBuffer()[z * volDimX * volDimY + y * volDimX + x];
//                 }
//             }
//         }
//     }
// 
//     for(int c = 0; c < rtcams->size(); c++)
//     {
//         delete rcTcsDepthSimMaps_hmh[c];
//     }
//     delete[] rcTcsDepthSimMaps_hmh;
// 
//     for(int zPart = 0; zPart < nZparts; zPart++)
//     {
//         delete vols_hmh[zPart];
//     }
//     delete[] vols_hmh;
// 
//     for(int i = 0; i < rtcams->size(); i++)
//     {
//         ttcams[i] = NULL;
//     }
//     delete[] ttcams;
// 
//     delete[] camsMinMaxFpDepths;
// 
//     if(verbose)
//         printfElapsedTime(t1);
// 
//     return true;
// }
// *
// bool PlaneSweepingCuda::filterRcIdDepthMapByTcDepthMap(StaticVector<unsigned short>* rcIdDepthMap,
//                                                          StaticVector<float>* tcDepthMap, int volDimX, int volDimY,
//                                                          int volDimZ, int volStepXY, StaticVector<float>* depths,
//                                                          int scale, int step, StaticVector<int>* rtcams, int distLimit)
// {
//     if(verbose)
//         printf("\n filterRcIdDepthMapByTcDepthMap scale %i step %i volStepXY %i volDimX %i volDimY %i volDimZ %i\n",
//                scale, step, volStepXY, volDimX, volDimY, volDimZ);
// 
//     int w = mp->getWidth(rc) / scale;
//     int h = mp->getHeight(rc) / scale;
// 
//     long t1 = clock();
// 
//     if(rtcams->size() <= 1)
//     {
//         return false;
//     }
// 
//     // WE DONT NEED IMAGES
//     cameraStruct** ttcams = new cameraStruct*[rtcams->size()];
//     for(int i = 0; i < rtcams->size(); i++)
//     {
//         cameraStruct* tcam = new cameraStruct;
//         tcam->init();
//         cps_fillCamera(tcam, (*rtcams)[i], mp, NULL, scale);
//         ttcams[i] = tcam;
//     }
// 
//     CudaHostMemoryHeap<float, 2> tcDepthMap_hmh(CudaSize<2>(w / volStepXY, h / volStepXY));
// 
// #pragma omp parallel for
//     for(int i = 0; i < (w / volStepXY) * (h / volStepXY); i++)
//     {
//         tcDepthMap_hmh.getBuffer()[i] = (*tcDepthMap)[i];
//     }
// 
//     CudaHostMemoryHeap<float, 2> depths_hmh(CudaSize<2>(depths->size(), 1));
// 
// #pragma omp parallel for
//     for(int i = 0; i < depths->size(); i++)
//     {
//         depths_hmh.getBuffer()[i] = (*depths)[i];
//         // printf("%f\n",depths_hmh.getBuffer()[i]);
//     }
// 
//     CudaHostMemoryHeap<unsigned short, 2> rcIdDepthMap_hmh(CudaSize<2>(volDimX, volDimY));
// 
// #pragma omp parallel for
//     for(int y = 0; y < volDimY; y++)
//     {
//         for(int x = 0; x < volDimX; x++)
//         {
//             rcIdDepthMap_hmh.getBuffer()[y * volDimX + x] = (*rcIdDepthMap)[y * volDimX + x];
//         }
//     }
// 
//     // sweep
//     ps_filterRcIdDepthMapByTcDepthMap(&rcIdDepthMap_hmh, ttcams, rtcams->size(), w, h, volStepXY, volDimX, volDimY,
//                                       volDimZ, depths_hmh, depths->size(), scale - 1, CUDADeviceNo, nImgsInGPUAtTime,
//                                       scales, tcDepthMap_hmh, verbose, distLimit);
// 
// #pragma omp parallel for
//     for(int y = 0; y < volDimY; y++)
//     {
//         for(int x = 0; x < volDimX; x++)
//         {
//             (*rcIdDepthMap)[y * volDimX + x] = rcIdDepthMap_hmh.getBuffer()[y * volDimX + x];
//         }
//     }
// 
//     for(int i = 0; i < rtcams->size(); i++)
//     {
//         ttcams[i] = NULL;
//     }
//     delete[] ttcams;
// 
//     if(verbose)
//         printfElapsedTime(t1);
// 
//     return true;
// }
// *
// bool PlaneSweepingCuda::filterRcIdDepthMapByTcDepthMaps(StaticVector<unsigned short>* nModalsMap,
//                                                           StaticVector<unsigned short>* rcIdDepthMap,
//                                                           StaticVector<StaticVector<float>*>* tcDepthMaps, int volDimX,
//                                                           int volDimY, int volDimZ, int volStepXY,
//                                                           StaticVector<float>* depths, int scale, int step,
//                                                           StaticVector<int>* rtcams, int distLimit)
// {
//     if(verbose)
//         printf("\n filterRcIdDepthMapByTcDepthMaps scale %i step %i volStepXY %i volDimX %i volDimY %i volDimZ %i cams "
//                "%i\n",
//                scale, step, volStepXY, volDimX, volDimY, volDimZ, rtcams->size());
// 
//     int w = mp->getMaxImageWidth() / scale;
//     int h = mp->getMaxImageHeight() / scale;
// 
//     long t1 = clock();
// 
//     if(rtcams->size() <= 1)
//     {
//         return false;
//     }
// 
//     // WE DONT NEED IMAGES
//     cameraStruct** ttcams = new cameraStruct*[rtcams->size()];
//     for(int i = 0; i < rtcams->size(); i++)
//     {
//         cameraStruct* tcam = new cameraStruct;
//         tcam->init();
//         cps_fillCamera(tcam, (*rtcams)[i], mp, NULL, scale);
//         ttcams[i] = tcam;
//     }
// 
//     CudaHostMemoryHeap<float, 2> depths_hmh(CudaSize<2>(depths->size(), 1));
// #pragma omp parallel for
//     for(int i = 0; i < depths->size(); i++)
//     {
//         depths_hmh.getBuffer()[i] = (*depths)[i];
//     }
// 
//     CudaHostMemoryHeap<float, 2>** tcDepthMaps_hmh = new CudaHostMemoryHeap<float, 2>*[rtcams->size() - 1];
//     for(int c = 0; c < rtcams->size() - 1; c++)
//     {
//         tcDepthMaps_hmh[c] = new CudaHostMemoryHeap<float, 2>(CudaSize<2>(volDimX, volDimY));
// #pragma omp parallel for
//         for(int i = 0; i < volDimX * volDimY; i++)
//         {
//             tcDepthMaps_hmh[c]->getBuffer()[i] = (*(*tcDepthMaps)[c])[i];
//         }
//     }
// 
//     CudaHostMemoryHeap<unsigned short, 2> rcIdDepthMap_hmh(CudaSize<2>(volDimX, volDimY));
// #pragma omp parallel for
//     for(int y = 0; y < volDimY; y++)
//     {
//         for(int x = 0; x < volDimX; x++)
//         {
//             rcIdDepthMap_hmh.getBuffer()[y * volDimX + x] = (*rcIdDepthMap)[y * volDimX + x];
//         }
//     }
// 
//     CudaHostMemoryHeap<unsigned short, 2> nModalsMap_hmh(CudaSize<2>(volDimX, volDimY));
// 
//     ps_filterRcIdDepthMapByTcDepthMaps(&nModalsMap_hmh, &rcIdDepthMap_hmh, ttcams, rtcams->size(), w, h, volStepXY,
//                                        volDimX, volDimY, volDimZ, depths_hmh, depths->size(), scale - 1, CUDADeviceNo,
//                                        nImgsInGPUAtTime, scales, tcDepthMaps_hmh, verbose, distLimit);
// 
// #pragma omp parallel for
//     for(int y = 0; y < volDimY; y++)
//     {
//         for(int x = 0; x < volDimX; x++)
//         {
//             (*nModalsMap)[y * volDimX + x] = nModalsMap_hmh.getBuffer()[y * volDimX + x];
//         }
//     }
// 
//     for(int c = 0; c < rtcams->size() - 1; c++)
//     {
//         delete tcDepthMaps_hmh[c];
//     }
//     delete[] tcDepthMaps_hmh;
// 
//     for(int i = 0; i < rtcams->size(); i++)
//     {
//         ttcams[i] = NULL;
//     }
//     delete[] ttcams;
// 
//     if(verbose)
//         printfElapsedTime(t1);
// 
//     return true;
// }
// 
// bool PlaneSweepingCuda::SGGCoptimizeSimVolume(StaticVector<unsigned short>* ftidMap,
//                                                 StaticVector<unsigned int>* ivolume, int _volDimX, int volDimY,
//                                                 int volDimZ, int xFrom, int xTo, int K)
// {
//     int volDimX = xTo - xFrom + 1;
// 
//     if(verbose)
//         printf("\n SGGC optimizing volume volDimX %i volDimY %i volDimZ %i K %i \n", volDimX, volDimY, volDimZ, K);
// 
//     long t1 = clock();
// 
//     CudaHostMemoryHeap<unsigned int, 3> vol_hmh(CudaSize<3>(volDimX, volDimY, volDimZ));
// 
// #pragma omp parallel for
//     for(int z = 0; z < volDimZ; z++)
//     {
//         for(int y = 0; y < volDimY; y++)
//         {
//             for(int x = xFrom; x <= xTo; x++)
//             {
//                 vol_hmh.getBuffer()[z * volDimX * volDimY + y * volDimX + (x - xFrom)] =
//                     (*ivolume)[z * _volDimX * volDimY + y * _volDimX + x];
//             }
//         }
//     }
// 
//     CudaHostMemoryHeap<unsigned int, 2> ftid_hmh(CudaSize<2>(volDimX, volDimY));
// 
//     if(K == 0)
//     {
//         ps_GC_aggregatePathVolume(&ftid_hmh, // f-irst t-label id
//                                   vol_hmh, volDimX, volDimY, volDimZ);
//     }
//     else
//     {
//         ps_GC_K_aggregatePathVolume(&ftid_hmh, // f-irst t-label id
//                                     vol_hmh, volDimX, volDimY, volDimZ, K);
//     }
// 
// #pragma omp parallel for
//     for(int y = 0; y < volDimY; y++)
//     {
//         for(int x = 0; x < volDimX; x++)
//         {
//             (*ftidMap)[y * _volDimX + (x + xFrom)] = ftid_hmh.getBuffer()[y * volDimX + x];
//         }
//     }
// 
//     if(verbose)
//         printfElapsedTime(t1);
// 
//     return true;
// }
*/

bool PlaneSweepingCuda::fuseDepthSimMapsGaussianKernelVoting(int w, int h, StaticVector<DepthSim>* oDepthSimMap,
                                                               const StaticVector<StaticVector<DepthSim>*>* dataMaps,
                                                               int nSamplesHalf, int nDepthsToRefine, float sigma)
{
    long t1 = clock();

    // sweep
    CudaHostMemoryHeap<float2, 2>** dataMaps_hmh = new CudaHostMemoryHeap<float2, 2>*[dataMaps->size()];
    for(int i = 0; i < dataMaps->size(); i++)
    {
        dataMaps_hmh[i] = new CudaHostMemoryHeap<float2, 2>(CudaSize<2>(w, h));
        for(int y = 0; y < h; y++)
        {
            for(int x = 0; x < w; x++)
            {
                float2& data_hmh = (*dataMaps_hmh[i])(x, y);
                const DepthSim& data = (*(*dataMaps)[i])[y * w + x];
                data_hmh.x = data.depth;
                data_hmh.y = data.sim;
            }
        }
    }

    CudaHostMemoryHeap<float2, 2> oDepthSimMap_hmh(CudaSize<2>(w, h));

    ps_fuseDepthSimMapsGaussianKernelVoting(&oDepthSimMap_hmh, dataMaps_hmh, dataMaps->size(), nSamplesHalf,
                                            nDepthsToRefine, sigma, w, h, verbose);

    for(int y = 0; y < h; y++)
    {
        for(int x = 0; x < w; x++)
        {
            const float2& oDepthSim_hmh = oDepthSimMap_hmh(x, y);
            DepthSim& oDepthSim = (*oDepthSimMap)[y * w + x];
            oDepthSim.depth = oDepthSim_hmh.x;
            oDepthSim.sim = oDepthSim_hmh.y;
        }
    }

    for(int i = 0; i < dataMaps->size(); i++)
    {
        delete dataMaps_hmh[i];
    }
    delete[] dataMaps_hmh;

    if(verbose)
        mvsUtils::printfElapsedTime(t1);

    return true;
}

bool PlaneSweepingCuda::optimizeDepthSimMapGradientDescent(StaticVector<DepthSim>* oDepthSimMap,
                                                             StaticVector<StaticVector<DepthSim>*>* dataMaps, int rc,
                                                             int nSamplesHalf, int nDepthsToRefine, float sigma,
                                                             int nIters, int yFrom, int hPart)
{
    if(mp->verbose)
        ALICEVISION_LOG_DEBUG("optimizeDepthSimMapGradientDescent.");

    int scale = 1;
    int w = mp->getWidth(rc);
    int h = hPart;

    long t1 = clock();

    int camsids = addCam(rc, NULL, scale);
    if(verbose)
        printf("rc: %i, ", rc);

    cameraStruct ttcams = *_cams[camsids];
    ttcams.camId = camsids;
    ttcams.rc    = rc;

    // sweep
    CudaHostMemoryHeap<float2, 2>** dataMaps_hmh = new CudaHostMemoryHeap<float2, 2>*[dataMaps->size()];
    for(int i = 0; i < dataMaps->size(); i++)
    {
        dataMaps_hmh[i] = new CudaHostMemoryHeap<float2, 2>(CudaSize<2>(w, h));
        for(int y = 0; y < h; y++)
        {
            for(int x = 0; x < w; x++)
            {
                int jO = (y + yFrom) * w + x;
                float2& h_data = (*dataMaps_hmh[i])(x, y);
                const DepthSim& data = (*(*dataMaps)[i])[jO];
                h_data.x = data.depth;
                h_data.y = data.sim;
            }
        }
    }

    CudaHostMemoryHeap<float2, 2> oDepthSimMap_hmh(CudaSize<2>(w, h));

    ps_optimizeDepthSimMapGradientDescent( &oDepthSimMap_hmh, dataMaps_hmh,
                                           dataMaps->size(), nSamplesHalf, nDepthsToRefine, nIters,
                                           sigma,
                                           ttcams,
                                           // *ttcams[0],
                                           // camsids.size(),
                                            w, h, scale - 1,
                                           CUDADeviceNo,
                                           // nImgsInGPUAtTime,
                                           scales,
                                           verbose, yFrom );

    for(int y = 0; y < h; y++)
    {
        for(int x = 0; x < w; x++)
        {
            int jO = (y + yFrom) * w + x;
            DepthSim& oDepthSim = (*oDepthSimMap)[jO];
            const float2& h_oDepthSim = oDepthSimMap_hmh(x, y);

            oDepthSim.depth = h_oDepthSim.x;
            oDepthSim.sim = h_oDepthSim.y;
        }
    }

    for(int i = 0; i < dataMaps->size(); i++)
    {
        delete dataMaps_hmh[i];
    }
    delete[] dataMaps_hmh;

    if(verbose)
        mvsUtils::printfElapsedTime(t1);

    return true;
}

#if 0
// bool PlaneSweepingCuda::computeDP1Volume(StaticVector<int>* ovolume, StaticVector<unsigned int>* ivolume,
//                                            int _volDimX, int volDimY, int volDimZ, int xFrom, int xTo)
// {
//     int volDimX = xTo - xFrom + 1;
// 
//     if(verbose)
//         printf("\n computerDP1Volume volDimX %i volDimY %i volDimZ %i \n", volDimX, volDimY, volDimZ);
// 
//     long t1 = clock();
// 
//     CudaHostMemoryHeap<unsigned int, 3> ivol_hmh(CudaSize<3>(volDimX, volDimY, volDimZ));
// 
// #pragma omp parallel for
//     for(int z = 0; z < volDimZ; z++)
//     {
//         for(int y = 0; y < volDimY; y++)
//         {
//             for(int x = xFrom; x <= xTo; x++)
//             {
//                 ivol_hmh.getBuffer()[z * volDimX * volDimY + y * volDimX + (x - xFrom)] =
//                     (*ivolume)[z * _volDimX * volDimY + y * _volDimX + x];
//             }
//         }
//     }
// 
//     CudaHostMemoryHeap<int, 3> ovol_hmh(CudaSize<3>(volDimX, volDimY, volDimZ));
// 
//     ps_computeDP1Volume(&ovol_hmh, &ivol_hmh, volDimX, volDimY, volDimZ, verbose);
// 
// #pragma omp parallel for
//     for(int z = 0; z < volDimZ; z++)
//     {
//         for(int y = 0; y < volDimY; y++)
//         {
//             for(int x = xFrom; x <= xTo; x++)
//             {
//                 (*ovolume)[z * _volDimX * volDimY + y * _volDimX + x] =
//                     ovol_hmh.getBuffer()[z * volDimX * volDimY + y * volDimX + (x - xFrom)];
//             }
//         }
//     }
// 
//     if(verbose)
//         printfElapsedTime(t1);
// 
//     return true;
// }
#endif

/*
// bool PlaneSweepingCuda::computeSimMapReprojectByDepthMapMovedByStep(StaticVector<float>* osimMap,
//                                                                       StaticVector<float>* iodepthMap, int rc, int tc,
//                                                                       int _wsh, float _gammaC, float _gammaP,
//                                                                       bool moveByTcOrRc, float moveStep)
// {
//     if(verbose)
//         printf("computeSimMapReprojectByDepthMapMovedByStep rc: %i\n", rc);
// 
//     int scale = 1;
//     int w = mp->getWidth(rc);
//     int h = mp->getHeight(rc);
// 
//     long t1 = clock();
// 
//     StaticVector<int> camsids;
//     camsids.reserve(2);
//     camsids.push_back(addCam(rc, NULL, scale));
//     if(verbose)
//         printf("rc: %i, ", rc);
//     if(verbose)
//         printf("tcams: ");
//     if(verbose)
//         printf("%i ", tc);
//     camsids.push_back(addCam(tc, NULL, scale));
//     if(verbose)
//         printf("\n");
// 
//     cameraStruct** ttcams = new cameraStruct*[camsids.size()];
//     for(int i = 0; i < camsids.size(); i++)
//     {
//         ttcams[i] = _cams[camsids[i]];
//         ttcams[i]->camId = camsids[i];
//         if(i == 0)
//         {
//             ttcams[i]->rc = rc;
//         }
//         else
//         {
//             ttcams[i]->rc = tc;
//         }
//     }
// 
//     // sweep
//     CudaHostMemoryHeap<float, 2> osimMap_hmh(CudaSize<2>(w, h));
//     CudaHostMemoryHeap<float, 2> iodepthMap_hmh(CudaSize<2>(w, h));
//     for(int i = 0; i < w * h; i++)
//     {
//         iodepthMap_hmh.getBuffer()[i] = (*iodepthMap)[i];
//     }
// 
//     ps_computeSimMapReprojectByDepthMapMovedByStep(
//         (CudaArray<uchar4, 2>**)ps_texs_arr, &osimMap_hmh, &iodepthMap_hmh, ttcams, camsids.size(), w, h, scale - 1,
//         CUDADeviceNo, nImgsInGPUAtTime, scales, verbose, _wsh, _gammaC, _gammaP, moveByTcOrRc, moveStep);
// 
//     for(int i = 0; i < w * h; i++)
//     {
//         (*osimMap)[i] = osimMap_hmh.getBuffer()[i];
//         (*iodepthMap)[i] = iodepthMap_hmh.getBuffer()[i];
//     }
// 
//     for(int i = 0; i < camsids.size(); i++)
//     {
//         ttcams[i] = NULL;
//     }
//     delete[] ttcams;
// 
//     if(verbose)
//         printfElapsedTime(t1);
// 
//     return true;
// }
// 
// bool PlaneSweepingCuda::computeRcTcdepthMap(StaticVector<float>* iRcDepthMap_oRcTcDepthMap,
//                                               StaticVector<float>* tcDdepthMap, int rc, int tc, float pixSizeRatioThr)
// {
//     if(verbose)
//         printf("computeRcTcdepthMap rc: %i\n", rc);
// 
//     int scale = 1;
//     int w = mp->getWidth(rc);
//     int h = mp->getHeight(rc);
// 
//     long t1 = clock();
// 
//     // WE DONT NEED IMAGES
//     cameraStruct** ttcams = new cameraStruct*[2];
//     {
//         cameraStruct* tcam = new cameraStruct;
//         tcam->init();
//         cps_fillCamera(tcam, rc, mp, NULL, scale);
//         ttcams[0] = tcam;
//         tcam = new cameraStruct;
//         tcam->init();
//         cps_fillCamera(tcam, tc, mp, NULL, scale);
//         ttcams[1] = tcam;
//     }
// 
//     CudaHostMemoryHeap<float, 2> iRcDepthMap_oRcTcDepthMap_hmh(CudaSize<2>(w, h));
//     CudaHostMemoryHeap<float, 2> tcDepthMap_hmh(CudaSize<2>(w, h));
//     for(int i = 0; i < w * h; i++)
//     {
//         iRcDepthMap_oRcTcDepthMap_hmh.getBuffer()[i] = (*iRcDepthMap_oRcTcDepthMap)[i];
//         tcDepthMap_hmh.getBuffer()[i] = (*tcDdepthMap)[i];
//     }
// 
//     ps_computeRcTcDepthMap(iRcDepthMap_oRcTcDepthMap_hmh, tcDepthMap_hmh, pixSizeRatioThr, ttcams, 2, w, h, scale - 1,
//                            CUDADeviceNo, nImgsInGPUAtTime, scales, verbose);
// 
//     for(int i = 0; i < w * h; i++)
//     {
//         (*iRcDepthMap_oRcTcDepthMap)[i] = iRcDepthMap_oRcTcDepthMap_hmh.getBuffer()[i];
//     }
// 
//     for(int i = 0; i < 2; i++)
//     {
//         ttcams[i] = NULL;
//     }
//     delete[] ttcams;
// 
//     if(verbose)
//         printfElapsedTime(t1);
// 
//     return true;
// }
*/

bool PlaneSweepingCuda::getSilhoueteMap(StaticVectorBool* oMap, int scale, int step, const rgb maskColor, int rc)
{
    if(verbose)
        ALICEVISION_LOG_DEBUG("getSilhoueteeMap: rc: " << rc);

    int w = mp->getWidth(rc) / scale;
    int h = mp->getHeight(rc) / scale;

    long t1 = clock();

    int camId = addCam(rc, NULL, scale);

    uchar4 maskColorRgb;
    maskColorRgb.x = maskColor.r;
    maskColorRgb.y = maskColor.g;
    maskColorRgb.z = maskColor.b;
    maskColorRgb.w = 1.0f;

    CudaHostMemoryHeap<bool, 2> omap_hmh(CudaSize<2>(w / step, h / step));

    ps_getSilhoueteMap( &omap_hmh, w, h, scale - 1,
                        CUDADeviceNo,
                        nImgsInGPUAtTime, scales, step, camId, maskColorRgb, verbose );

    for(int i = 0; i < (w / step) * (h / step); i++)
    {
        (*oMap)[i] = omap_hmh.getBuffer()[i];
    }

    if(verbose)
        mvsUtils::printfElapsedTime(t1);

    return true;
}

int listCUDADevices(bool verbose)
{
    return ps_listCUDADevices(verbose);
}

} // namespace depthMap
} // namespace aliceVision
