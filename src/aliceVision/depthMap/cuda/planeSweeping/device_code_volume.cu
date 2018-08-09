// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "aliceVision/depthMap/cuda/planeSweeping/device_code_volume.cuh"

#include "aliceVision/depthMap/cuda/planeSweeping/device_code.cuh"
#include "aliceVision/depthMap/cuda/planeSweeping/device_utils.cuh"

#include "aliceVision/depthMap/cuda/deviceCommon/device_matrix.cuh"
#include "aliceVision/depthMap/cuda/deviceCommon/device_color.cuh"
#include "aliceVision/depthMap/cuda/deviceCommon/device_patch_es.cuh"

namespace aliceVision {
namespace depthMap {

static __device__ void volume_computePatch( cudaTextureObject_t depthsTex, patch& ptch, int depthid, int2& pix)
{
    float3 p;
    float pixSize;

    float fpPlaneDepth = tex2D<float>(depthsTex, depthid, 0);
    p = get3DPointForPixelAndFrontoParellePlaneRC(pix, fpPlaneDepth);
    pixSize = computePixSize(p);

    ptch.p = p;
    ptch.d = pixSize;
    computeRotCSEpip(ptch, p);
}

__global__ void volume_slice_kernel(
    cudaTextureObject_t r4tex,
    cudaTextureObject_t t4tex,
    cudaTextureObject_t depthsTex,
    cudaTextureObject_t volPixs_x_Tex,
    cudaTextureObject_t volPixs_y_Tex,
    cudaTextureObject_t volPixs_z_Tex,
    unsigned char* slice, int slice_p,
    // float3* slicePts, int slicePts_p,
    int nsearchdepths, int ndepths, int slicesAtTime, int width, int height, int wsh,
    int t, int npixs, const float gammaC, const float gammaP, const float epipShift )
{
    int sdptid = blockIdx.x * blockDim.x + threadIdx.x;
    int pixid = blockIdx.y * blockDim.y + threadIdx.y;

    if((sdptid < nsearchdepths) && (pixid < slicesAtTime) && (slicesAtTime * t + pixid < npixs))
    {
        int volPix_x = tex2D<int>(volPixs_x_Tex, pixid, t);
        int volPix_y = tex2D<int>(volPixs_y_Tex, pixid, t);
        int volPix_z = tex2D<int>(volPixs_z_Tex, pixid, t);
        int2 pix = make_int2(volPix_x, volPix_y);
        int depthid = sdptid + volPix_z;

        if(depthid < ndepths)
        {
            patch ptcho;
            volume_computePatch( depthsTex, ptcho, depthid, pix);

            float fsim = compNCCby3DptsYK( r4tex, t4tex, ptcho, wsh, width, height, gammaC, gammaP, epipShift);
            // unsigned char sim = (unsigned char)(((fsim+1.0f)/2.0f)*255.0f);

            float fminVal = -1.0f;
            float fmaxVal = 1.0f;
            fsim = (fsim - fminVal) / (fmaxVal - fminVal);
            fsim = fminf(1.0f, fmaxf(0.0f, fsim));
            unsigned char sim = (unsigned char)(fsim * 255.0f);

            // coalescent
            /*int sliceid = pixid * slice_p + sdptid;
            slice[sliceid] = sim;*/
            *get2DBufferAt(slice, slice_p, sdptid, pixid) = sim;
        }
    }
}

__global__ void volume_saveSliceToVolume_kernel(
    cudaTextureObject_t volPixsTex_x,
    cudaTextureObject_t volPixsTex_y,
    cudaTextureObject_t volPixsTex_z,
    unsigned char* volume, int volume_s, int volume_p, unsigned char* slice,
    int slice_p, int nsearchdepths, int ndepths, int slicesAtTime,
    int width, int height, int t, int npixs, int volStepXY, int volDimX,
    int volDimY, int volDimZ, int volLUX, int volLUY, int volLUZ )
{
    int sdptid = blockIdx.x * blockDim.x + threadIdx.x;
    int pixid = blockIdx.y * blockDim.y + threadIdx.y;

    if((sdptid < nsearchdepths) && (pixid < slicesAtTime) && (slicesAtTime * t + pixid < npixs))
    {
        int volPix_x = tex2D<int>(volPixsTex_x, pixid, t);
        int volPix_y = tex2D<int>(volPixsTex_y, pixid, t);
        int volPix_z = tex2D<int>(volPixsTex_z, pixid, t);
        int2 pix = make_int2(volPix_x, volPix_y);
        int depthid = sdptid + volPix_z;

        if(depthid < ndepths)
        {
            unsigned char sim = *get2DBufferAt(slice, slice_p, sdptid, pixid);

            int vx = (pix.x - volLUX) / volStepXY;
            int vy = (pix.y - volLUY) / volStepXY;
            // int vz = sdptid;//depthid;
            int vz = depthid - volLUZ;
            if((vx >= 0) && (vx < volDimX) && (vy >= 0) && (vy < volDimY) && (vz >= 0) && (vz < volDimZ))
            {
                unsigned char* volsim = get3DBufferAt(volume, volume_s, volume_p, vx, vy, vz);
                *volsim = min(sim, *volsim);
            }
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

__global__ void volume_transposeAddAvgVolume_kernel(unsigned char* volumeT, int volumeT_s, int volumeT_p,
                                                    const unsigned char* volume, int volume_s, int volume_p, int volDimX,
                                                    int volDimY, int volDimZ, int dimTrnX, int dimTrnY, int dimTrnZ,
                                                    int z, int lastN)
{
    int vx = blockIdx.x * blockDim.x + threadIdx.x;
    int vy = blockIdx.y * blockDim.y + threadIdx.y;
    int vz = z;

    if((vx >= 0) && (vx < volDimX) && (vy >= 0) && (vy < volDimY) && (vz >= 0) && (vz < volDimZ))
    {
        int v[3];
        v[0] = vx;
        v[1] = vy;
        v[2] = vz;

        int dimsTrn[3];
        dimsTrn[0] = dimTrnX;
        dimsTrn[1] = dimTrnY;
        dimsTrn[2] = dimTrnZ;

        int vTx = v[dimsTrn[0]];
        int vTy = v[dimsTrn[1]];
        int vTz = v[dimsTrn[2]];

        unsigned char* oldVal_ptr = get3DBufferAt(volumeT, volumeT_s, volumeT_p, vTx, vTy, vTz);
        unsigned char newVal = *get3DBufferAt(volume, volume_s, volume_p, vx, vy, vz);
        float val = (*oldVal_ptr * (float)lastN + (float)newVal) / (float)(lastN + 1);

        *oldVal_ptr = (unsigned char)(fminf(255.0f, val));
    }
}

__global__ void volume_updateMinXSlice_kernel(unsigned char* volume, int volume_s, int volume_p,
                                              unsigned char* xySliceBestSim, int xySliceBestSim_p, int* xySliceBestZ,
                                              int xySliceBestZ_p, int volDimX, int volDimY, int volDimZ, int vz)
{
    int vx = blockIdx.x * blockDim.x + threadIdx.x;
    int vy = blockIdx.y * blockDim.y + threadIdx.y;

    if((vx >= 0) && (vx < volDimX) && (vy >= 0) && (vy < volDimY) && (vz >= 0) && (vz < volDimZ))
    {
        unsigned char sim = *get3DBufferAt(volume, volume_s, volume_p, vx, vy, vz);
        unsigned char* actSim_ptr = get2DBufferAt(xySliceBestSim, xySliceBestSim_p, vx, vy);
        if((sim < *actSim_ptr) || (vz == 0))
        {
            *actSim_ptr = sim;
            *get2DBufferAt(xySliceBestZ, xySliceBestZ_p, vx, vy) = vz;
        }
    }
}

#if 0
// __global__ void volume_computeBestXSlice_kernel(
//     cudaTextureObject_t sliceTexUChar,
//     unsigned char* xsliceBestInColCst,
//     int volDimX, int volDimY )
// {
//     int vx = blockIdx.x * blockDim.x + threadIdx.x;
// 
//     if((vx >= 0) && (vx < volDimX))
//     {
//         unsigned char bestCst = tex2D<unsigned char>(sliceTexUChar, vx, 0);
//         for(int vy = 0; vy < volDimY; vy++)
//         {
//             unsigned char cst = tex2D<unsigned char>(sliceTexUChar, vx, vy);
//             bestCst = cst < bestCst ? cst : bestCst;
//         }
//         xsliceBestInColCst[vx] = bestCst;
//     }
// }
#endif

#if 0
// __global__ void volume_agregateCostVolumeAtZ_kernel(unsigned char* volume, int volume_s, int volume_p,
//                                                     unsigned char* xsliceBestInColCst, int volDimX, int volDimY,
//                                                     int volDimZ, int vz, unsigned char P1, unsigned char P2,
//                                                     bool transfer)
// {
//     int vx = blockIdx.x * blockDim.x + threadIdx.x;
//     int vy = blockIdx.y * blockDim.y + threadIdx.y;
// 
//     if((vx >= 0) && (vx < volDimX) && (vy >= 0) && (vy < volDimY) && (vz >= 0) && (vz < volDimZ))
//     {
//         unsigned char* sim_ptr = get3DBufferAt(volume, volume_s, volume_p, vx, vy, vz);
//         unsigned char sim = *sim_ptr;
//         unsigned char pathCost = (transfer == true) ? sim : 255;
// 
//         if((vz >= 1) && (vy >= 1) && (vy < volDimY - 1))
//         {
//             unsigned char bestCostM = xsliceBestInColCst[vx];
//             unsigned char pathCostMDM1 = volume[(vz - 1) * volume_s + (vy - 1) * volume_p + vx];
//             unsigned char pathCostMD = volume[(vz - 1) * volume_s + (vy + 0) * volume_p + vx];
//             unsigned char pathCostMDP1 = volume[(vz - 1) * volume_s + (vy + 1) * volume_p + vx];
//             pathCost = (unsigned char)(fminf(
//                 255.0f, (float)sim + fminf(fminf(fminf((float)pathCostMD, (float)pathCostMDM1 + (float)P1),
//                                                  (float)pathCostMDP1 + (float)P1),
//                                            (float)bestCostM + (float)P2) -
//                             (float)bestCostM));
//         }
// 
//         *sim_ptr = pathCost;
//     }
// }
#endif

__global__ void volume_computeBestXSliceUInt_kernel(
    cudaTextureObject_t sliceTexUInt,
    unsigned int* xsliceBestInColCst, int volDimX, int volDimY )
{
    int vx = blockIdx.x * blockDim.x + threadIdx.x;

    if((vx >= 0) && (vx < volDimX))
    {
        unsigned int bestCst = tex2D<unsigned int>(sliceTexUInt, vx, 0);
        for(int vy = 0; vy < volDimY; vy++)
        {
            unsigned int cst = tex2D<unsigned int>(sliceTexUInt, vx, vy);
            bestCst = cst < bestCst ? cst : bestCst;
        }
        xsliceBestInColCst[vx] = bestCst;
    }
}

/**
 * @param[inout] xySliceForZ input similarity plane
 * @param[in] xySliceForZM1
 * @param[in] xSliceBestInColCst
 * @param[out] volSimT output similarity volume
 */
__global__ void volume_agregateCostVolumeAtZinSlices_kernel(
    cudaTextureObject_t r4tex,
    unsigned int* xySliceForZ, int xySliceForZ_p,
    const unsigned int* xySliceForZM1, int xySliceForZM1_p,
    const unsigned int* xSliceBestInColSimForZM1,
    unsigned char* volSimT, int volSimT_s, int volSimT_p, 
    int volDimX, int volDimY, int volDimZ, 
    int vz, unsigned int _P1, unsigned int _P2,
    bool transfer, int volLUX, int volLUY,
    int dimTrnX, bool doInvZ)
{
    int vx = blockIdx.x * blockDim.x + threadIdx.x;
    int vy = blockIdx.y * blockDim.y + threadIdx.y;

    if((vx >= 0) && (vx < volDimX) && (vy >= 0) && (vy < volDimY) && (vz >= 0) && (vz < volDimZ))
    {
        unsigned int* sim_yx = get2DBufferAt(xySliceForZ, xySliceForZ_p, vx, vy);
        unsigned int sim = *sim_yx;
        unsigned int pathCost = transfer ? sim : 255;

        if((vz >= 1) && (vy >= 1) && (vy < volDimY - 1))
        {
            int z = doInvZ ? volDimZ - vz : vz;
            int z1 = doInvZ ? z + 1 : z - 1; // M1
            int imX0 = volLUX + (dimTrnX == 0) ? vx : z; // current
            int imY0 = volLUY + (dimTrnX == 0) ?  z : vx;
            int imX1 = volLUX + (dimTrnX == 0) ? vx : z1; // M1
            int imY1 = volLUY + (dimTrnX == 0) ? z1 : vx;
            float4 gcr0 = 255.0f * tex2D<float4>(r4tex, (float)imX0 + 0.5f, (float)imY0 + 0.5f);
            float4 gcr1 = 255.0f * tex2D<float4>(r4tex, (float)imX1 + 0.5f, (float)imY1 + 0.5f);
            float deltaC = Euclidean3(gcr0, gcr1);
            // unsigned int P1 = (unsigned int)sigmoid(5.0f,20.0f,60.0f,10.0f,deltaC);
            unsigned int P1 = _P1;
            // 15.0 + (255.0 - 15.0) * (1.0 / (1.0 + exp(10.0 * ((x - 20.) / 80.))))
            unsigned int P2 = (unsigned int)sigmoid(15.0f, 255.0f, 80.0f, 20.0f, deltaC);
            // unsigned int P2 = _P2;

            unsigned int bestCostInColM1 = xSliceBestInColSimForZM1[vx];
            unsigned int pathCostMDM1 = *get2DBufferAt(xySliceForZM1, xySliceForZM1_p, vx, vy - 1); // M1: minus 1 over depths
            unsigned int pathCostMD   = *get2DBufferAt(xySliceForZM1, xySliceForZM1_p, vx, vy); 
            unsigned int pathCostMDP1 = *get2DBufferAt(xySliceForZM1, xySliceForZM1_p, vx, vy + 1); // P1: plus 1 over depths
            // pathCost = (unsigned char)(fminf(255.0f,(float)sim +
            // fminf(fminf(fminf((float)pathCostMD,(float)pathCostMDM1+(float)P1),(float)pathCostMDP1+(float)P1),(float)bestCostM+(float)P2)
            unsigned int minCost = min(pathCostMD, pathCostMDM1    + P1);
            minCost              = min(minCost,    pathCostMDP1    + P1);
            minCost              = min(minCost,    bestCostInColM1 + P2);

            // if 'pathCostMD' is the minimal value of the depth
            pathCost = sim + minCost - bestCostInColM1;
        }
        unsigned char* volume_zyx = get3DBufferAt(volSimT, volSimT_s, volSimT_p, vx, vy, vz);
        *volume_zyx = (unsigned char)(min(255, pathCost));
        *sim_yx = pathCost;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#if 0
// __global__ void volume_updateRcVolumeForTcDepthMap_kernel(
//     cudaTextureObject_t sliceTexFloat2,
//     unsigned int* volume, int volume_s, int volume_p,
//     const int volDimX, const int volDimY, const int volDimZ,
//     const int vz, const int volStepXY, const int tcDepthMapStep,
//     const int width, const int height, const float fpPlaneDepth,
//     const float stepInDepth, const int zPart,
//     const int vilDimZGlob, const float maxTcRcPixSizeInVoxRatio,
//     const bool considerNegativeDepthAsInfinity,
//     const float2 tcMinMaxFpDepth )
// {
//     int vx = blockIdx.x * blockDim.x + threadIdx.x;
//     int vy = blockIdx.y * blockDim.y + threadIdx.y;
// 
//     if((vx >= 0) && (vx < volDimX) && (vy >= 0) && (vy < volDimY) && (vz >= 0) && (volDimZ * zPart + vz < vilDimZGlob))
//     {
//         int2 pixi = make_int2(vx * volStepXY, vy * volStepXY);
//         // float2 pixf = make_float2(vx*volStepXY+0.5f,vy*volStepXY+0.5f);
//         float3 p = get3DPointForPixelAndFrontoParellePlaneRC(pixi, fpPlaneDepth);
// 
//         float depthTcP = size(sg_s_tC - p);
//         float fpDepthTcP = frontoParellePlaneTCDepthFor3DPoint(p);
// 
//         float2 tpixf;
//         getPixelFor3DPointTC(tpixf, p);
//         int2 tpix = make_int2((int)(tpixf.x + 0.5f), (int)(tpixf.y + 0.5f));
//         // int2 tpix = make_int2((int)(tpixf.x),(int)(tpixf.y));
//         int2 tpixMap =
//             make_int2((int)(tpixf.x / (float)tcDepthMapStep + 0.5f), (int)(tpixf.y / (float)tcDepthMapStep + 0.5f));
// 
//         float rcPixSize = computeRcPixSize(p);
//         float tcPixSize = computeTcPixSize(p);
// 
//         if(tcPixSize < rcPixSize * maxTcRcPixSizeInVoxRatio)
//         {
//             float depthTc = 0.0f;
//             float simWeightTc = 1.0f;
//             float2 depthSimTc = tex2D<float2>(sliceTexFloat2, tpixMap.x, tpixMap.y);
//             depthTc = depthSimTc.x;
//             // simWeightTc = sigmoid(0.1f,1.0f,1.0f,-0.5f,depthSimTc.y);
//             simWeightTc = depthSimTc.y;
// 
//             unsigned int Tval = 0;
//             unsigned int Vval = (((fpDepthTcP > tcMinMaxFpDepth.x) && (fpDepthTcP < tcMinMaxFpDepth.y) &&
//                                   (tpix.x >= 0) && (tpix.x < width) && (tpix.y >= 0) && (tpix.y < height))
//                                      ? 255
//                                      : 0);
// 
//             if((considerNegativeDepthAsInfinity == false) || (depthTc > 0.0f))
//             {
//                 int distid = (int)(rintf((depthTcP - depthTc) / stepInDepth));
//                 int distidabs = abs(distid);
// 
//                 unsigned char udistCostT = (unsigned char)(255.0f * (float)simWeightTc);
//                 unsigned char udistCostVis =
//                     (unsigned char)((float)distFcnConst3[min(2, distidabs)] * (float)simWeightTc);
//                 Tval = (unsigned int)((distid >= 0) ? ((distidabs <= 2) ? udistCostT : 0) : 0);
//                 Vval = (unsigned int)((distid < 0) ? udistCostVis : 0);
// 
//                 Tval = (((fpDepthTcP > tcMinMaxFpDepth.x) && (fpDepthTcP < tcMinMaxFpDepth.y) && (depthTc > 0.0f) &&
//                          (tpix.x >= 0) && (tpix.x < width) && (tpix.y >= 0) && (tpix.y < height))
//                             ? Tval
//                             : 0);
//                 Vval = (((fpDepthTcP > tcMinMaxFpDepth.x) && (fpDepthTcP < tcMinMaxFpDepth.y) && (depthTc > 0.0f) &&
//                          (tpix.x >= 0) && (tpix.x < width) && (tpix.y >= 0) && (tpix.y < height))
//                             ? Vval
//                             : 0);
//             }
// 
//             unsigned int* volume_zyx = get3DBufferAt(volume, volume_s, volume_p, vx, vy, vz);
//             unsigned int val = *volume_zyx;
//             unsigned int TvalOld = val >> 16;
//             unsigned int VvalOld = val & 0xFFFF;
//             Tval = min(65534, Tval + TvalOld);
//             Vval = min(65534, Vval + VvalOld);
//             val = (Tval << 16) | (Vval & 0xFFFF);
//             
//             *volume_zyx = val;
//         }
//     }
// }
#endif

#if 0
// __global__ void volume_updateRcVolumeForTcDepthMap2_kernel(
//     cudaTextureObject_t sliceTexFloat2,
//     unsigned int* volume, int volume_s, int volume_p,
//     const int volDimX, const int volDimY, const int volDimZ,
//     const int vz, const int volStepXY, const int tcDepthMapStep,
//     const int width, const int height, const float fpPlaneDepth,
//     const float stepInDepth, const int zPart,
//     const int vilDimZGlob, const float maxTcRcPixSizeInVoxRatio,
//     const bool considerNegativeDepthAsInfinity,
//     const float2 tcMinMaxFpDepth )
// {
//     int vx = blockIdx.x * blockDim.x + threadIdx.x;
//     int vy = blockIdx.y * blockDim.y + threadIdx.y;
// 
//     if((vx >= 0) && (vx < volDimX) && (vy >= 0) && (vy < volDimY) && (vz >= 0) && (volDimZ * zPart + vz < vilDimZGlob))
//     {
// 
//         int2 pixi = make_int2(vx * volStepXY, vy * volStepXY);
//         float3 p = get3DPointForPixelAndFrontoParellePlaneRC(pixi, fpPlaneDepth);
//         float depthTcP = size(sg_s_tC - p);
//         float fpDepthTcP = frontoParellePlaneTCDepthFor3DPoint(p);
//         float2 tpixf;
//         getPixelFor3DPointTC(tpixf, p);
//         int2 tpix = make_int2((int)(tpixf.x + 0.5f), (int)(tpixf.y + 0.5f));
//         int2 tpixMap =
//             make_int2((int)(tpixf.x / (float)tcDepthMapStep + 0.5f), (int)(tpixf.y / (float)tcDepthMapStep + 0.5f));
// 
//         unsigned int* volume_zyx = get3DBufferAt(volume, volume_s, volume_p, vx, vy, vz);
//         unsigned int val = *volume_zyx;
//         unsigned int TvalOld = val >> 16;
//         unsigned int VvalOld = val & 0xFFFF;
// 
//         unsigned int TvalSum = 0;
//         unsigned int VvalSum = 0;
// 
//         int k = 0;
//         for(int xp = -k; xp <= k; xp++)
//         {
//             for(int yp = -k; yp <= k; yp++)
//             {
//                 int2 tpixMapAct = make_int2(tpixMap.x + xp, tpixMap.y + yp);
//                 float depthTc = 0.0f;
//                 float simWeightTc = 1.0f;
// 
//                 float2 depthSimTc = tex2D<float2>(sliceTexFloat2, tpixMapAct.x, tpixMapAct.y);
//                 depthTc = depthSimTc.x;
//                 // simWeightTc = sigmoid(0.1f,1.0f,1.0f,-0.5f,depthSimTc.y);
//                 simWeightTc = depthSimTc.y;
//                 float3 tp =
//                     get3DPointForPixelAndDepthFromTC(make_float2(tpixMapAct.x + 0.5f, tpixMapAct.y + 0.5f), depthTc);
//                 float rcPixSize = computeRcPixSize(tp);
//                 float tcPixSize = computeTcPixSize(tp);
// 
//                 if((tcPixSize < rcPixSize * 1.2f) && (rcPixSize < tcPixSize * 1.2f))
//                 {
//                     unsigned int Tval = 0;
//                     unsigned int Vval = (((fpDepthTcP > tcMinMaxFpDepth.x) && (fpDepthTcP < tcMinMaxFpDepth.y) &&
//                                           (tpix.x >= 0) && (tpix.x < width) && (tpix.y >= 0) && (tpix.y < height))
//                                              ? 255
//                                              : 0);
// 
//                     if((considerNegativeDepthAsInfinity == false) || (depthTc > 0.0f))
//                     {
//                         int distid = (int)(rintf((depthTcP - depthTc) / stepInDepth));
//                         int distidabs = abs(distid);
// 
//                         unsigned char udistCostT = (unsigned char)(255.0f * (float)simWeightTc);
//                         unsigned char udistCostVis =
//                             (unsigned char)((float)distFcnConst3[min(2, distidabs)] * (float)simWeightTc);
//                         Tval = (unsigned int)((distid >= 0) ? ((distidabs <= 2) ? udistCostT : 0) : 0);
//                         Vval = (unsigned int)((distid < 0) ? udistCostVis : 0);
// 
//                         Tval = (((fpDepthTcP > tcMinMaxFpDepth.x) && (fpDepthTcP < tcMinMaxFpDepth.y) &&
//                                  (depthTc > 0.0f) && (tpix.x >= 0) && (tpix.x < width) && (tpix.y >= 0) &&
//                                  (tpix.y < height))
//                                     ? Tval
//                                     : 0);
//                         Vval = (((fpDepthTcP > tcMinMaxFpDepth.x) && (fpDepthTcP < tcMinMaxFpDepth.y) &&
//                                  (depthTc > 0.0f) && (tpix.x >= 0) && (tpix.x < width) && (tpix.y >= 0) &&
//                                  (tpix.y < height))
//                                     ? Vval
//                                     : 0);
//                     }
// 
//                     TvalSum += Tval;
//                     VvalSum += Vval;
//                 }
//             }
//         }
// 
//         TvalSum = min(65534, TvalSum + TvalOld);
//         VvalSum = min(65534, VvalSum + VvalOld);
//         val = (TvalSum << 16) | (VvalSum & 0xFFFF);
//         *volume_zyx = val;
//     }
// }
#endif

#if 0
// __global__ void volume_update_nModalsMap_kernel_id0(
//     unsigned short* nModalsMap, int nModalsMap_p,
//     int volDimX, int volDimY )
// {
//     int vx = blockIdx.x * blockDim.x + threadIdx.x;
//     int vy = blockIdx.y * blockDim.y + threadIdx.y;
// 
//     if((vx >= 0) && (vx < volDimX) && (vy >= 0) && (vy < volDimY))
//     {
//         unsigned short val = 0;
//         unsigned short* nModalsMap_yx = get2DBufferAt(nModalsMap, nModalsMap_p, vx, vy);
//         *nModalsMap_yx = val;
//     }
// }
#endif

#if 0
// __global__ void volume_update_nModalsMap_kernel(
//     cudaTextureObject_t depthsTex,
//     cudaTextureObject_t sliceTex,
//     unsigned short* nModalsMap, int nModalsMap_p,
//     unsigned short* rcIdDepthMap, int rcIdDepthMap_p, int volDimX,
//     int volDimY, int volDimZ, int volStepXY, int tcDepthMapStep, int width,
//     int height, int distLimit, int id )
// {
//     int vx = blockIdx.x * blockDim.x + threadIdx.x;
//     int vy = blockIdx.y * blockDim.y + threadIdx.y;
// 
//     if((vx >= 0) && (vx < volDimX) && (vy >= 0) && (vy < volDimY))
//     {
//         unsigned short val = 0;
//         unsigned short* nModalsMap_yx = get2DBufferAt(nModalsMap, nModalsMap_p, vx, vy);
//         if(id > 0)
//         {
//             val = *nModalsMap_yx;
// 
//             int2 pix = make_int2(vx * volStepXY, vy * volStepXY);
//             int vz = *get2DBufferAt(rcIdDepthMap, rcIdDepthMap_p, vx, vy);
// 
//             if(vz > 0)
//             {
//                 float fpPlaneDepth  = tex2D<float>(depthsTex, vz, 0);
//                 float fpPlaneDepthP = tex2D<float>(depthsTex, vz - 1, 0);
//                 float step = fabsf(fpPlaneDepthP - fpPlaneDepth);
// 
//                 float3 p = get3DPointForPixelAndFrontoParellePlaneRC(pix, fpPlaneDepth);
//                 float2 tpixf;
//                 getPixelFor3DPointTC(tpixf, p);
//                 int2 tpix = make_int2((int)(tpixf.x + 0.5f), (int)(tpixf.y + 0.5f));
// 
//                 float depthTc = tex2D<float>(sliceTex, tpix.x / tcDepthMapStep, tpix.y / tcDepthMapStep);
//                 float depthTcP = size(sg_s_tC - p);
//                 int distid = (int)(fabsf(depthTc - depthTcP) / step + 0.5f);
// 
//                 if((depthTc > 0.0f)
//                     && (tpix.x >= 0) && (tpix.x < width) && (tpix.y >= 0) && (tpix.y < height)
//                     && (distid < distLimit))
//                     val++;
//             }
//         }
//         *nModalsMap_yx = val;
//     }
// }
#endif

#if 0
// __global__ void volume_filterRcIdDepthMapByTcDepthMap_kernel(
//     cudaTextureObject_t depthsTex,
//     cudaTextureObject_t sliceTex,
//     unsigned short* rcIdDepthMap, int rcIdDepthMap_p,
//     int volDimX, int volDimY, int volDimZ, int volStepXY,
//     int tcDepthMapStep, int width, int height, int distLimit )
// {
//     int vx = blockIdx.x * blockDim.x + threadIdx.x;
//     int vy = blockIdx.y * blockDim.y + threadIdx.y;
// 
//     if((vx >= 0) && (vx < volDimX) && (vy >= 0) && (vy < volDimY))
//     {
//         int2 pix = make_int2(vx * volStepXY, vy * volStepXY);
//         unsigned short* rcIdDepthMap_yx = get2DBufferAt(rcIdDepthMap, rcIdDepthMap_p, vx, vy);
//         int vz = (int)*rcIdDepthMap_yx;
//         if(vz > 0)
//         {
//             float fpPlaneDepth = tex2D<float>(depthsTex, vz, 0);
//             float fpPlaneDepthP = tex2D<float>(depthsTex, vz - 1, 0);
//             float step = fabsf(fpPlaneDepthP - fpPlaneDepth);
// 
//             float3 p = get3DPointForPixelAndFrontoParellePlaneRC(pix, fpPlaneDepth);
//             float2 tpixf;
//             getPixelFor3DPointTC(tpixf, p);
//             int2 tpix = make_int2((int)(tpixf.x + 0.5f), (int)(tpixf.y + 0.5f));
// 
//             float depthTc = tex2D<float>(sliceTex, tpix.x / tcDepthMapStep, tpix.y / tcDepthMapStep);
//             float depthTcP = size(sg_s_tC - p);
//             int distid = (int)(fabsf(depthTc - depthTcP) / step + 0.5f);
// 
//             * rcIdDepthMap_yx =
//                 (((depthTc > 0.0f) && (tpix.x >= 0) && (tpix.x < width) && (tpix.y >= 0) && (tpix.y < height))
//                      ? ((distid < distLimit) ? 1 : 0)
//                      : 0);
//         }
//     }
// }
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

__global__ void update_GC_volumeXYSliceAtZInt4_kernel(int* xySlice_x, int xySlice_x_p,
	                                              int* xySlice_y, int xySlice_y_p,
						      int* xySlice_z, int xySlice_z_p,
						      int* xySlice_w, int xySlice_w_p,
						      unsigned int* volume, int volume_s,
                                                      int volume_p, int volDimX, int volDimY, int volDimZ, int vz)
{
    int vx = blockIdx.x * blockDim.x + threadIdx.x;
    int vy = blockIdx.y * blockDim.y + threadIdx.y;
    if((vx >= 0) && (vx < volDimX) && (vy >= 0) && (vy < volDimY) && (vz >= 0) && (vz < volDimZ))
    {
	int* xySlice_yx_x = get2DBufferAt(xySlice_x, xySlice_x_p, vx, vy);
	int* xySlice_yx_y = get2DBufferAt(xySlice_y, xySlice_y_p, vx, vy);
	int* xySlice_yx_z = get2DBufferAt(xySlice_z, xySlice_z_p, vx, vy);
	int* xySlice_yx_w = get2DBufferAt(xySlice_w, xySlice_w_p, vx, vy);

        int4 M1 = make_int4( *xySlice_yx_x,  *xySlice_yx_y,  *xySlice_yx_z,  *xySlice_yx_w );
        int R = M1.x;
        int dQ = M1.y;
        int first = M1.z;
        int wasReached = M1.w;

        if(wasReached == 0)
        {
            unsigned int* volume_zyx = get3DBufferAt(volume, volume_s, volume_p, vx, vy, vz);
            unsigned int val = *volume_zyx;
            int Ti = (int)(val >> 16);    // T - edge
            int Si = (int)(val & 0xFFFF); // visibility

            //			val = volume[(vz-1)*volume_s+vy*volume_p+vx];
            // int TiM1 = (int)(val >> 16); //T - edge
            // int SiM1 = (int)(val & 0xFFFF); //visibility

            int EiM1 = Si;

            if(dQ < 0)
            {
                wasReached = 1;
            }
            else
            {
                if(dQ > EiM1)
                {
                    first = vz;
                    dQ = EiM1;
                }
                dQ = dQ + (Si - Ti);
            }

            if(vz == volDimZ - 1)
            {
                if(dQ < 0)
                {
                    wasReached = 1;
                }
                if(wasReached == 0)
                {
                    first = volDimZ - 1;
                }
            }

            *xySlice_yx_x = R;
            *xySlice_yx_y = dQ;
            *xySlice_yx_z = first;
            *xySlice_yx_w = wasReached;
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#if 0
// /* This kernel becomes useless because we need only the remaining signed - unsigned typecast */
// __global__ void volume_GC_K_getVolumeXYSliceInt4ToUintDimZ_kernel(unsigned int* oxySlice, int oxySlice_p,
//                                                                   int* ixySlice_z, int ixySlice_z_p,
// 								  int volDimX, int volDimY)
// {
//     int vx = blockIdx.x * blockDim.x + threadIdx.x;
//     int vy = blockIdx.y * blockDim.y + threadIdx.y;
//     if((vx >= 0) && (vx < volDimX) && (vy >= 0) && (vy < volDimY))
//     {
//         int* ixySlice_yx_z = get2DBufferAt(ixySlice_z, ixySlice_z_p, vx, vy);
//         unsigned int* oxySlice_yx = get2DBufferAt(oxySlice, oxySlice_p, vx, vy);
// 
//         *oxySlice_yx = (unsigned int)(*ixySlice_yx_z);
//     }
// }
#endif

#if 0
// __global__ void volume_GC_K_initXYSliceInt4_kernel(int* xySlice_x, int xySlice_x_p,
//                                                    int* xySlice_y, int xySlice_y_p,
//                                                    int* xySlice_z, int xySlice_z_p,
//                                                    int* xySlice_w, int xySlice_w_p,
// 						   unsigned int* volume, int volume_s,
//                                                    int volume_p, int volDimX, int volDimY, int vz)
// {
//     int vx = blockIdx.x * blockDim.x + threadIdx.x;
//     int vy = blockIdx.y * blockDim.y + threadIdx.y;
//     if((vx >= 0) && (vx < volDimX) && (vy >= 0) && (vy < volDimY))
//     {
//         int* xySlice_yx_x = get2DBufferAt(xySlice_x, xySlice_x_p, vx, vy);
//         int* xySlice_yx_y = get2DBufferAt(xySlice_y, xySlice_y_p, vx, vy);
//         int* xySlice_yx_z = get2DBufferAt(xySlice_z, xySlice_z_p, vx, vy);
//         int* xySlice_yx_w = get2DBufferAt(xySlice_w, xySlice_w_p, vx, vy);
//         if(vz == 0)
//         {
//             int R = 0;
//             int Q = 10000000;
//             int first = 0;
//             int wasReached = 0;
// 
//             *xySlice_yx_x = R;
//             *xySlice_yx_y = Q;
//             *xySlice_yx_z = first;
//             *xySlice_yx_w = wasReached;
//         }
//         else
//         {
//             int R              = *xySlice_yx_x;
//             int Q              = *xySlice_yx_y;
//             int first          = *xySlice_yx_z;
//             int wasReachedStop = *xySlice_yx_w;
// 
//             unsigned int* volume_zyx = get3DBufferAt(volume, volume_s, volume_p, vx, vy, vz);
//             unsigned int val = *volume_zyx;
//             int Ti = (int)(val >> 16);    // T - edge
//             int Si = (int)(val & 0xFFFF); // visibility
// 
//             R = R + (Si - Ti);
// 
//             *xySlice_yx_x = R;
//             *xySlice_yx_y = Q;
//             *xySlice_yx_z = first;
//             *xySlice_yx_w = wasReachedStop;
//         }
//     }
// }
#endif

#if 0
// __global__ void update_GC_K_volumeXYSliceAtZInt4_kernel(int* xySlice_x, int xySlice_x_p,
//                                                         int* xySlice_y, int xySlice_y_p,
//                                                         int* xySlice_z, int xySlice_z_p,
//                                                         int* xySlice_w, int xySlice_w_p,
//                                                         unsigned int* volume,
//                                                         int volume_s, int volume_p, int volDimX, int volDimY,
//                                                         int volDimZ, int vz, int K)
// {
//     int vx = blockIdx.x * blockDim.x + threadIdx.x;
//     int vy = blockIdx.y * blockDim.y + threadIdx.y;
//     if((vx >= 0) && (vx < volDimX) && (vy >= 0) && (vy < volDimY) && (vz >= 0) && (vz < volDimZ))
//     {
//         int* xySlice_yx_x = get2DBufferAt(xySlice_x, xySlice_x_p, vx, vy);
//         int* xySlice_yx_y = get2DBufferAt(xySlice_y, xySlice_y_p, vx, vy);
//         int* xySlice_yx_z = get2DBufferAt(xySlice_z, xySlice_z_p, vx, vy);
//         int* xySlice_yx_w = get2DBufferAt(xySlice_w, xySlice_w_p, vx, vy);
// 
//         int R              = *xySlice_yx_x;
//         int Q              = *xySlice_yx_y;
//         int first          = *xySlice_yx_z;
//         int wasReachedStop = *xySlice_yx_w;
// 
//         int wasReached = 0;
//         int stop = 0;
//         if(wasReachedStop < 0)
//         {
//             wasReached = 1;
//         }
//         else
//         {
//             stop = wasReachedStop;
//         }
// 
//         /*
//         if ((R>100000000)||(Q>100000000))
//         {
//                 first = 0;
//                 wasReached = 1;
//         }
//         */
// 
//         if((wasReached == 0) && (stop > 0) && (Q + R >= 0))
//         {
//             wasReached = 1;
//         }
// 
//         if((wasReached == 0) && (Q + R < 0))
//         {
//             stop = stop + 1;
//             if(stop == K)
//             {
//                 wasReached = 1;
//             }
//         }
// 
//         if(wasReached == 0)
//         {
//             int val = *get3DBufferAt(volume, volume_s, volume_p, vx, vy, vz);
// //            int Si = (int)(val & 0xFFFF); // visibility
// 
//             val = *get3DBufferAt(volume, volume_s, volume_p, vx, vy, vz + 1);
//             int Ti1 = (int)(val >> 16);    // T - edge
//             int Si1 = (int)(val & 0xFFFF); // visibility
// 
//             val = *get3DBufferAt(volume, volume_s, volume_p, vx, vy, vz + K);
//             int TiK = (int)(val >> 16);    // T - edge
//             int SiK = (int)(val & 0xFFFF); // visibility
// 
//             // int Ei  = Si;
//             int Ei = Si1;
//             // int Ei  = 0;
// 
//             /*
//             if (K>=3) {
//                     val = volume[(vz+3)*volume_s+vy*volume_p+vx];
//                     int Ti3 = (int)(val >> 16); //T - edge
//                     int Si3 = (int)(val & 0xFFFF); //visibility
//                     Ei  = Si3;
//             }
//             */
// 
//             if(Ei < Q)
//             {
//                 Q = Ei;
//                 first = vz + 1;
//             }
// 
//             int d1 = Si1 - Ti1;
//             Q = Q + d1;
//             R = R - d1 + (SiK - TiK);
// 
//             if((vz == volDimZ - K - 1) && (Q + R < 0))
//             {
//                 wasReached = 1;
//             }
// 
//             if((vz == volDimZ - K - 1) && (wasReached == 0))
//             {
//                 first = vz + 1;
//             }
//         }
// 
//         if(wasReached == 0)
//         {
//             wasReachedStop = stop;
//         }
//         else
//         {
//             wasReachedStop = -1;
//         }
// 
//         *xySlice_yx_x = R;
//         *xySlice_yx_y = Q;
//         *xySlice_yx_z = first;
//         *xySlice_yx_w = wasReachedStop;
//     }
// }
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#if 0
// __global__ void volume_compute_rDP1_kernel(int2* xySlice, int xySlice_p, int* ovolume, int ovolume_s, int ovolume_p,
//                                            unsigned int* volume, int volume_s, int volume_p, int volDimX, int volDimY,
//                                            int volDimZ, int vz)
// {
//     int vx = blockIdx.x * blockDim.x + threadIdx.x;
//     int vy = blockIdx.y * blockDim.y + threadIdx.y;
// 
//     if((vx >= 0) && (vx < volDimX) && (vy >= 0) && (vy < volDimY) && (vz >= 0) && (vz < volDimZ))
//     {
//         unsigned int* volume_zyx = get3DBufferAt(volume, volume_s, volume_p, vx, vy, vz);
//         unsigned int val = *volume_zyx;
// 
//         int* ovolume_zyx = get3DBufferAt(ovolume, ovolume_s, ovolume_p, vx, vy, vz);
// 
//         int2* xySlice_yx = get2DBufferAt(xySlice, xySlice_p, vx, vy); 
// 
//         int Ti = (int)(val >> 16);    // T - edge
//         int Si = (int)(val & 0xFFFF); // visibility
//         int EiM1 = Si;
// 
//         int Ei = EiM1;
//         if(vz < volDimZ - 1)
//         {
//             volume_zyx = get3DBufferAt(volume, volume_s, volume_p, vx, vy, vz + 1);
//             val = *volume_zyx;
//             int Si1 = (int)(val & 0xFFFF); // visibility
//             Ei = Si1;
//         }
// 
//         if(vz == volDimZ - 1)
//         {
//             int Q0 = 0;
//             int dQ = Si - Ti;
//             *xySlice_yx = make_int2(Q0, dQ);
//             *ovolume_zyx = Q0 + dQ + EiM1;
//         }
// 
//         if((vz >= 0) && (vz <= volDimZ - 2))
//         {
//             int Q0 = xySlice_yx->x;
//             int dQ = xySlice_yx->y;
// 
//             if(dQ + Ei < 0)
//             {
//                 Q0 = Q0 + dQ + Ei;
//                 dQ = -Ei;
//             }
//             else
//             {
//                 if(dQ > 0)
//                 {
//                     dQ = 0;
//                 }
//             }
//             dQ = dQ + (Si - Ti);
// 
//             if(vz > 0)
//             {
//                 *ovolume_zyx = Q0 + dQ + EiM1;
//             }
// 
//             *xySlice_yx = make_int2(Q0, dQ); // Ei = S(i+1)
// 
//             if(vz == 0)
//             {
//                 *ovolume_zyx = Q0 + dQ;
//             }
//         }
//     }
// }
#endif

__global__ void volume_compute_DP1_kernel(int2* xySlice, int xySlice_p, int* ovolume, int ovolume_s, int ovolume_p,
                                          unsigned int* volume, int volume_s, int volume_p, int volDimX, int volDimY,
                                          int volDimZ, int vz)
{
    int vx = blockIdx.x * blockDim.x + threadIdx.x;
    int vy = blockIdx.y * blockDim.y + threadIdx.y;

    if((vx >= 0) && (vx < volDimX) && (vy >= 0) && (vy < volDimY) && (vz >= 0) && (vz < volDimZ))
    {
        unsigned int* volume_zyx = get3DBufferAt(volume, volume_s, volume_p, vx, vy, vz);
        unsigned int val = *volume_zyx;

        int* ovolume_zyx = get3DBufferAt(ovolume, ovolume_s, ovolume_p, vx, vy, vz);;

        int2* xySlice_yx = get2DBufferAt(xySlice, xySlice_p, vx, vy); 

        int Ti = (int)(val >> 16);    // T - edge
        int Si = (int)(val & 0xFFFF); // visibility
        int EiM1 = Si;

        int Ei = EiM1;
        if(vz < volDimZ - 1)
        {
            volume_zyx = get3DBufferAt(volume, volume_s, volume_p, vx, vy, vz + 1);
            val = *volume_zyx;
            int Si1 = (int)(val & 0xFFFF); // visibility
            Ei = Si1;
        }

        if(vz == 0)
        {
            int Q0 = 0;
            int dQ = Si - Ti;
            *xySlice_yx = make_int2(Q0, dQ);
            *ovolume_zyx = Q0 + dQ + EiM1;
        }

        if((vz > 0) && (vz <= volDimZ - 1))
        {
            int Q0 = xySlice_yx->x;
            int dQ = xySlice_yx->y;

            if(dQ + Ei < 0)
            {
                Q0 = Q0 + dQ + Ei;
                dQ = -Ei;
            }
            else
            {
                if(dQ > 0)
                {
                    dQ = 0;
                }
            }
            dQ = dQ + (Si - Ti);

            if(vz < volDimZ - 1)
            {
                *ovolume_zyx = Q0 + dQ + EiM1;
            }

            *xySlice_yx = make_int2(Q0, dQ); // Ei = S(i+1)

            if(vz == volDimZ - 1)
            {
                *ovolume_zyx = Q0 + dQ;
            }
        }
    }
}

#if 0
// __global__ void volume_compute_rDP1_volume_minMaxMap_kernel(int2* xySlice, int xySlice_p, int* volume, int volume_s,
//                                                             int volume_p, int volDimX, int volDimY, int volDimZ, int z,
//                                                             int zPart, int volDimZGlob)
// {
//     int vx = blockIdx.x * blockDim.x + threadIdx.x;
//     int vy = blockIdx.y * blockDim.y + threadIdx.y;
// 
//     int vzPart = z;
//     int vzGlob = volDimZ * zPart + z;
// 
//     if((vx >= 0) && (vx < volDimX) && (vy >= 0) && (vy < volDimY) && (vzGlob >= 0) && (vzGlob < volDimZGlob) &&
//        (vzPart < volDimZ))
//     {
//         int* volume_zyx = get3DBufferAt(volume, volume_s, volume_p, vx, vy, vzPart);
//         int val = *volume_zyx;
// 
//         int2* xySlice_yx = get2DBufferAt(xySlice, xySlice_p, vx, vy); 
// 
//         if(vzGlob == 0)
//         {
//             *xySlice_yx = make_int2(val, val);
//         }
//         else
//         {
//             int minVal = xySlice_yx->x;
//             int maxVal = xySlice_yx->y;
//             *xySlice_yx = make_int2(min(minVal, val), max(maxVal, val));
//         }
//     }
// }
#endif

#if 0
// __global__ void volume_normalize_rDP1_volume_by_minMaxMap_kernel(int2* xySlice, int xySlice_p, int* volume,
//                                                                  int volume_s, int volume_p, int volDimX, int volDimY,
//                                                                  int volDimZ, int z, int zPart, int volDimZGlob)
// {
//     int vx = blockIdx.x * blockDim.x + threadIdx.x;
//     int vy = blockIdx.y * blockDim.y + threadIdx.y;
// 
//     int vzPart = z;
//     int vzGlob = volDimZ * zPart + z;
// 
//     if((vx >= 0) && (vx < volDimX) && (vy >= 0) && (vy < volDimY) && (vzGlob >= 0) && (vzGlob < volDimZGlob) &&
//        (vzPart < volDimZ))
//     {
//         int2* xySlice_yx = get2DBufferAt(xySlice, xySlice_p, vx, vy); 
// 
//         int minVal = xySlice_yx->x;
//         int maxVal = xySlice_yx->y;
//         int* volume_zyx = get3DBufferAt(volume, volume_s, volume_p, vx, vy, vzPart);
//         int val = *volume_zyx;
// 
//         if(maxVal - minVal > 0)
//         {
//             // val = (int)(255.0f*((float)(val-minVal)/(float)(maxVal-minVal)));
//             val = (int)(255.0f * ((float)(val - minVal) / (float)(maxVal - minVal)));
//         }
// 
//         *volume_zyx = val;
//     }
// }
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#if 0
// __global__ void volume_filter_VisTVolume_kernel(
//     cudaTextureObject_t sliceTexUInt,
//     unsigned int* ovolume, int ovolume_s, int ovolume_p,
//     int volDimX, int volDimY, int volDimZ,
//     int vz, int K )
// {
//     int vx = blockIdx.x * blockDim.x + threadIdx.x;
//     int vy = blockIdx.y * blockDim.y + threadIdx.y;
// 
//     if((vx >= 0) && (vx < volDimX) && (vy >= 0) && (vy < volDimY) && (vz >= 0) && (vz < volDimZ))
//     {
//         unsigned int* ovolume_zyx = get3DBufferAt(ovolume, ovolume_s, ovolume_p, vx, vy, vz);
//         unsigned int val = *ovolume_zyx;
//         unsigned int TvalOld = (unsigned int)(val >> 16);    // T - edge
//         unsigned int VvalOld = (unsigned int)(val & 0xFFFF); // visibility
// 
//         unsigned int Tval = TvalOld;
//         unsigned int Vval = VvalOld;
// 
//         // unsigned int bestCst = tex2D<unsigned int>(sliceTexUInt,vx,0);
//         // int wsh = abs(K);
//         int wsh = 0;
//         for(int xp = vx - wsh; xp <= vx + wsh; xp++)
//         {
//             for(int yp = vy - wsh; yp <= vy + wsh; yp++)
//             {
//                 unsigned int valN = tex2D<unsigned int>(sliceTexUInt, xp, yp);
//                 unsigned int VvalN = (unsigned int)(valN & 0xFFFF); // visibility
// 
//                 if(K > 0)
//                 {
//                     Vval = max(Vval, VvalN);
//                 }
// 
//                 // if (K>0) {
//                 //	Tval = max(Tval,TvalN);
//                 //}
//             }
//         }
// 
//         val = (Tval << 16) | (Vval & 0xFFFF);
//         *ovolume_zyx = val;
//     }
// }
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#if 0
// __global__ void volume_filter_enforceTWeightInVolume_kernel(
//     cudaTextureObject_t sliceTexUInt,
//     unsigned int* ovolume, int ovolume_s, int ovolume_p,
//     int volDimX, int volDimY, int volDimZ, int vz, int K)
// {
//     int vx = blockIdx.x * blockDim.x + threadIdx.x;
//     int vy = blockIdx.y * blockDim.y + threadIdx.y;
// 
//     if((vx >= 0) && (vx < volDimX) && (vy >= 0) && (vy < volDimY) && (vz >= 0) && (vz < volDimZ))
//     {
//         unsigned int* ovolume_zyx = get3DBufferAt(ovolume, ovolume_s, ovolume_p, vx, vy, vz);
//         unsigned int val = *ovolume_zyx;
//         unsigned int TvalOld = (unsigned int)(val >> 16);    // T - edge
//         unsigned int VvalOld = (unsigned int)(val & 0xFFFF); // visibility
// 
//         unsigned int VvalFront = 0;
//         unsigned int VvalBack = 0;
// 
//         for(int xp = vx - K; xp <= vx + K; xp++)
//         {
//             unsigned int valN = tex2D<unsigned int>(sliceTexUInt, xp, vy);
//             // unsigned int TvalN = (unsigned  int)(valN >> 16); //T - edge
//             unsigned int VvalN = (unsigned int)(valN & 0xFFFF); // visibility
//             if(xp < vx)
//             {
//                 VvalFront += VvalN;
//             }
//             if(xp > vx)
//             {
//                 VvalBack += VvalN;
//             }
//         }
// 
//         float fVvalFront = (float)VvalFront / (float)K;
//         float fVvalBack = (float)VvalBack / (float)K;
// 
//         if((fVvalFront - fVvalBack > 500.0f) && (fVvalBack / fVvalFront) < 0.5f)
//         {
//             TvalOld = min(65534, TvalOld + 10000);
//         }
// 
//         val = (TvalOld << 16) | (VvalOld & 0xFFFF);
//         *ovolume_zyx = val;
//     }
// }
#endif

} // namespace depthMap
} // namespace aliceVision
