// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

namespace aliceVision {
namespace depthMap {

inline __device__ void volume_computePatch( const cameraStructBase* rc_cam_s,
                                            const cameraStructBase* tc_cam_s,
                                            patch& ptch,
                                            const float fpPlaneDepth, const int2& pix )
{
    float3 p;
    float pixSize;

    p = get3DPointForPixelAndFrontoParellePlaneRC( rc_cam_s, pix, fpPlaneDepth); // no texture use
    pixSize = computePixSize( rc_cam_s, p ); // no texture use

    ptch.p = p;
    ptch.d = pixSize;
    computeRotCSEpip( rc_cam_s, tc_cam_s, ptch, p ); // no texture use
}

__global__ void volume_slice_kernel(
                                    cudaTextureObject_t rc_tex,
                                    cudaTextureObject_t tc_tex,
                                    const cameraStructBase* rc_cam_s,
                                    const cameraStructBase* tc_cam_s,
                                    float* depths_dev,
                                    const int startDimZ,
                                    int width, int height, int wsh,
                                    const float gammaC, const float gammaP, const float epipShift,
                                    float* volume, int volume_s, int volume_p,
                                    int volStepXY,
                                    int volDimX, int volDimY )
{
    const int vx = blockIdx.x * blockDim.x + threadIdx.x;
    const int vy = blockIdx.y * blockDim.y + threadIdx.y;
    const int vz = blockIdx.z;

    if( vx >= volDimX ) return;
    if( vy >= volDimY ) return;

    const int x = vx * volStepXY;
    const int y = vy * volStepXY;

    const float fpPlaneDepth = depths_dev[startDimZ+vz];

    if( x >= width  ) return;
    if( y >= height ) return;

    // const unsigned char occluded = tex2D(r4tex, x+0.5f, y+0.5f).w;
    const float occluded = tex2D<float4>(rc_tex, x+0.5f, y+0.5f).w;
    if( occluded > 0.75f ) return;

    const int2 pix = make_int2( x, y );

    patch ptcho;
    volume_computePatch( rc_cam_s, tc_cam_s, ptcho, fpPlaneDepth, pix); // no texture use

    float fsim = compNCCby3DptsYK( rc_tex, tc_tex,
                                   rc_cam_s, tc_cam_s,
                                   ptcho, wsh,
                                   width, height,
                                   gammaC, gammaP,
                                   epipShift);

    const float fminVal = -1.0f;
    const float fmaxVal = 1.0f;
    fsim = (fsim - fminVal) / (fmaxVal - fminVal);
    fsim = fminf(1.0f, fmaxf(0.0f, fsim));
    // int sim = (unsigned char)(fsim * 255.0f); // upcast to int due to atomicMin

    *get3DBufferAt(volume, volume_s, volume_p, vx, vy, vz) = fsim;
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

template <typename T>
__global__ void volume_transposeVolume_kernel(T* volumeT, int volumeT_s, int volumeT_p, 
                                              const T* volume, int volume_s, int volume_p, 
                                              int volDimX, int volDimY, int volDimZ, 
                                              int dimTrnX, int dimTrnY, int dimTrnZ, 
                                              int z)
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

        T* oldVal_ptr = get3DBufferAt(volumeT, volumeT_s, volumeT_p, vTx, vTy, vTz);
        T newVal = *get3DBufferAt(volume, volume_s, volume_p, vx, vy, vz);
        *oldVal_ptr = newVal;
    }
}

template <typename T>
__global__ void volume_shiftZVolumeTempl_kernel(T* volume, int volume_s, int volume_p, int volDimX, int volDimY,
                                                int volDimZ, int vz)
{
    int vx = blockIdx.x * blockDim.x + threadIdx.x;
    int vy = blockIdx.y * blockDim.y + threadIdx.y;

    if((vx >= 0) && (vx < volDimX) && (vy >= 0) && (vy < volDimY) && (vz >= 0) && (vz < volDimZ))
    {
        T* v1_ptr = get3DBufferAt(volume, volume_s, volume_p, vx, vy, vz);
        T* v2_ptr = get3DBufferAt(volume, volume_s, volume_p, vx, vy, volDimZ - 1 - vz);
        T v1 = *v1_ptr;
        T v2 = *v2_ptr;
        *v1_ptr = v2;
        *v2_ptr = v1;
    }
}

template <typename T>
__global__ void volume_initVolume_kernel(T* volume, int volume_s, int volume_p, int volDimX, int volDimY, int volDimZ,
                                         int vz, T cst)
{
    int vx = blockIdx.x * blockDim.x + threadIdx.x;
    int vy = blockIdx.y * blockDim.y + threadIdx.y;

    if((vx >= 0) && (vx < volDimX) && (vy >= 0) && (vy < volDimY) && (vz >= 0) && (vz < volDimZ))
    {
        T* volume_zyx = get3DBufferAt(volume, volume_s, volume_p, vx, vy, vz);
        *volume_zyx = cst;
    }
}

__global__ void volume_updateMinXSlice_kernel(unsigned char* volume, int volume_s, int volume_p,
                                              unsigned char* xySliceBestSim, int xySliceBestSim_p,
                                              int* xySliceBestZ, int xySliceBestZ_p,
                                              int volDimX, int volDimY, int volDimZ, int vz)
{
    int vx = blockIdx.x * blockDim.x + threadIdx.x;
    int vy = blockIdx.y * blockDim.y + threadIdx.y;

    if( ( vx >= volDimX ) || ( vy >= volDimY ) || ( vz >= volDimZ ) || ( vz < 0 ) ) return;

    unsigned char sim = *get3DBufferAt(volume, volume_s, volume_p, vx, vy, vz);
    BufPtr<unsigned char> xySliceBest( xySliceBestSim, xySliceBestSim_p );
    unsigned char actSim_ptr = xySliceBest.at(vx, vy);
    if((sim < actSim_ptr) || (vz == 0))
    {
        xySliceBest                              .at(vx,vy) = sim;
        BufPtr<int>(xySliceBestZ, xySliceBestZ_p).at(vx,vy) = vz;
    }
}

template <typename T1, typename T2>
__global__ void volume_getVolumeXYSliceAtZ_kernel(T1* xySlice, int xySlice_p, T2* volume, int volume_s, int volume_p,
                                                  int volDimX, int volDimY, int volDimZ, int vz)
{
    int vx = blockIdx.x * blockDim.x + threadIdx.x;
    int vy = blockIdx.y * blockDim.y + threadIdx.y;

    if((vx >= 0) && (vx < volDimX) && (vy >= 0) && (vy < volDimY) && (vz >= 0) && (vz < volDimZ))
    {
        T2* volume_zyx = get3DBufferAt(volume, volume_s, volume_p, vx, vy, vz);
        T1* xySlice_yx = get2DBufferAt(xySlice, xySlice_p, vx, vy);
        *xySlice_yx = (T1)(*volume_zyx);
    }
}

__global__ void volume_computeBestXSlice_kernel(unsigned char* xsliceBestInColCst, int volDimX, int volDimY)
{
    int vx = blockIdx.x * blockDim.x + threadIdx.x;

    if((vx >= 0) && (vx < volDimX))
    {
        unsigned char bestCst = tex2D(sliceTexUChar, vx, 0);
        for(int vy = 0; vy < volDimY; vy++)
        {
            unsigned char cst = tex2D(sliceTexUChar, vx, vy);
            bestCst = cst < bestCst ? cst : bestCst;
        }
        xsliceBestInColCst[vx] = bestCst;
    }
}

__global__ void volume_agregateCostVolumeAtZ_kernel(unsigned char* volume, int volume_s, int volume_p,
                                                    unsigned char* xsliceBestInColCst, int volDimX, int volDimY,
                                                    int volDimZ, int vz, unsigned char P1, unsigned char P2,
                                                    bool transfer)
{
    int vx = blockIdx.x * blockDim.x + threadIdx.x;
    int vy = blockIdx.y * blockDim.y + threadIdx.y;

    if((vx >= 0) && (vx < volDimX) && (vy >= 0) && (vy < volDimY) && (vz >= 0) && (vz < volDimZ))
    {
        unsigned char* sim_ptr = get3DBufferAt(volume, volume_s, volume_p, vx, vy, vz);
        unsigned char sim = *sim_ptr;
        unsigned char pathCost = (transfer == true) ? sim : 255;

        if((vz >= 1) && (vy >= 1) && (vy < volDimY - 1))
        {
            unsigned char bestCostM = xsliceBestInColCst[vx];
            unsigned char pathCostMDM1 = volume[(vz - 1) * volume_s + (vy - 1) * volume_p + vx];
            unsigned char pathCostMD = volume[(vz - 1) * volume_s + (vy + 0) * volume_p + vx];
            unsigned char pathCostMDP1 = volume[(vz - 1) * volume_s + (vy + 1) * volume_p + vx];
            pathCost = (unsigned char)(fminf(
                255.0f, (float)sim + fminf(fminf(fminf((float)pathCostMD, (float)pathCostMDM1 + (float)P1),
                                                 (float)pathCostMDP1 + (float)P1),
                                           (float)bestCostM + (float)P2) -
                            (float)bestCostM));
        }

        *sim_ptr = pathCost;
    }
}

__global__ void volume_computeBestXSliceUInt_kernel(unsigned int* xsliceBestInColCst, int volDimX, int volDimY)
{
    int vx = blockIdx.x * blockDim.x + threadIdx.x;

    if((vx >= 0) && (vx < volDimX))
    {
        unsigned int bestCst = tex2D(sliceTexUInt, vx, 0);
        for(int vy = 0; vy < volDimY; vy++)
        {
            unsigned int cst = tex2D(sliceTexUInt, vx, vy);
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
__global__ void volume_agregateCostVolumeAtZinSlices_kernel(unsigned int* xySliceForZ, int xySliceForZ_p,
                                                            const unsigned int* xySliceForZM1, int xySliceForZM1_p,
                                                            const unsigned int* xSliceBestInColSimForZM1,
                                                            unsigned char* volSimT, int volSimT_s, int volSimT_p, 
                                                            int volDimX, int volDimY, int volDimZ, 
                                                            int vz, unsigned int _P1, unsigned int _P2,
                                                            bool transfer,
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
            int imX0 = (dimTrnX == 0) ? vx : z; // current
            int imY0 = (dimTrnX == 0) ?  z : vx;
            int imX1 = (dimTrnX == 0) ? vx : z1; // M1
            int imY1 = (dimTrnX == 0) ? z1 : vx;
            float4 gcr0 = 255.0f * tex2D(r4tex, (float)imX0 + 0.5f, (float)imY0 + 0.5f);
            float4 gcr1 = 255.0f * tex2D(r4tex, (float)imX1 + 0.5f, (float)imY1 + 0.5f);
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

__global__ void volume_updateRcVolumeForTcDepthMap_kernel(unsigned int* volume, int volume_s, int volume_p,
                                                          const int volDimX, const int volDimY, const int volDimZ,
                                                          const int vz, const int volStepXY, const int tcDepthMapStep,
                                                          const int width, const int height, const float fpPlaneDepth,
                                                          const float stepInDepth, const int zPart,
                                                          const int vilDimZGlob, const float maxTcRcPixSizeInVoxRatio,
                                                          const bool considerNegativeDepthAsInfinity,
                                                          const float2 tcMinMaxFpDepth, const bool useSimilarity)
{
    int vx = blockIdx.x * blockDim.x + threadIdx.x;
    int vy = blockIdx.y * blockDim.y + threadIdx.y;

    if((vx >= 0) && (vx < volDimX) && (vy >= 0) && (vy < volDimY) && (vz >= 0) && (volDimZ * zPart + vz < vilDimZGlob))
    {
        int2 pixi = make_int2(vx * volStepXY, vy * volStepXY);
        // float2 pixf = make_float2(vx*volStepXY+0.5f,vy*volStepXY+0.5f);

        // get3DPointForPixelAndFrontoParellePlaneRC uses __constant__ sg_s_r.*
        float3 p = get3DPointForPixelAndFrontoParellePlaneRC(pixi, fpPlaneDepth);

        float depthTcP = size(sg_s_t.C - p);
        float fpDepthTcP = frontoParellePlaneTCDepthFor3DPoint(p);

        float2 tpixf;
        getPixelFor3DPointTC(tpixf, p);
        int2 tpix = make_int2((int)(tpixf.x + 0.5f), (int)(tpixf.y + 0.5f));
        // int2 tpix = make_int2((int)(tpixf.x),(int)(tpixf.y));
        int2 tpixMap =
            make_int2((int)(tpixf.x / (float)tcDepthMapStep + 0.5f), (int)(tpixf.y / (float)tcDepthMapStep + 0.5f));

        float rcPixSize = computeRcPixSize(p);
        float tcPixSize = computeTcPixSize(p);

        if(tcPixSize < rcPixSize * maxTcRcPixSizeInVoxRatio)
        {
            float depthTc = 0.0f;
            float simWeightTc = 1.0f;
            if(useSimilarity == false)
            {
                depthTc = tex2D(sliceTex, tpixMap.x, tpixMap.y);
            }
            else
            {
                float2 depthSimTc = tex2D(sliceTexFloat2, tpixMap.x, tpixMap.y);
                depthTc = depthSimTc.x;
                // simWeightTc = sigmoid(0.1f,1.0f,1.0f,-0.5f,depthSimTc.y);
                simWeightTc = depthSimTc.y;
            }

            unsigned int Tval = 0;
            unsigned int Vval = (((fpDepthTcP > tcMinMaxFpDepth.x) && (fpDepthTcP < tcMinMaxFpDepth.y) &&
                                  (tpix.x >= 0) && (tpix.x < width) && (tpix.y >= 0) && (tpix.y < height))
                                     ? 255
                                     : 0);

            if((considerNegativeDepthAsInfinity == false) || (depthTc > 0.0f))
            {
                int distid = (int)(rintf((depthTcP - depthTc) / stepInDepth));
                int distidabs = abs(distid);

                unsigned char udistCostT = (unsigned char)(255.0f * (float)simWeightTc);
                unsigned char udistCostVis =
                    (unsigned char)((float)distFcnConst3[min(2, distidabs)] * (float)simWeightTc);
                Tval = (unsigned int)((distid >= 0) ? ((distidabs <= 2) ? udistCostT : 0) : 0);
                Vval = (unsigned int)((distid < 0) ? udistCostVis : 0);

                Tval = (((fpDepthTcP > tcMinMaxFpDepth.x) && (fpDepthTcP < tcMinMaxFpDepth.y) && (depthTc > 0.0f) &&
                         (tpix.x >= 0) && (tpix.x < width) && (tpix.y >= 0) && (tpix.y < height))
                            ? Tval
                            : 0);
                Vval = (((fpDepthTcP > tcMinMaxFpDepth.x) && (fpDepthTcP < tcMinMaxFpDepth.y) && (depthTc > 0.0f) &&
                         (tpix.x >= 0) && (tpix.x < width) && (tpix.y >= 0) && (tpix.y < height))
                            ? Vval
                            : 0);
            }

            unsigned int* volume_zyx = get3DBufferAt(volume, volume_s, volume_p, vx, vy, vz);
            unsigned int val = *volume_zyx;
            unsigned int TvalOld = val >> 16;
            unsigned int VvalOld = val & 0xFFFF;
            Tval = min(65534, Tval + TvalOld);
            Vval = min(65534, Vval + VvalOld);
            val = (Tval << 16) | (Vval & 0xFFFF);
            
            *volume_zyx = val;
        }
    }
}

__global__ void volume_updateRcVolumeForTcDepthMap2_kernel(unsigned int* volume, int volume_s, int volume_p,
                                                           const int volDimX, const int volDimY, const int volDimZ,
                                                           const int vz, const int volStepXY, const int tcDepthMapStep,
                                                           const int width, const int height, const float fpPlaneDepth,
                                                           const float stepInDepth, const int zPart,
                                                           const int vilDimZGlob, const float maxTcRcPixSizeInVoxRatio,
                                                           const bool considerNegativeDepthAsInfinity,
                                                           const float2 tcMinMaxFpDepth, const bool useSimilarity)
{
    int vx = blockIdx.x * blockDim.x + threadIdx.x;
    int vy = blockIdx.y * blockDim.y + threadIdx.y;

    if((vx >= 0) && (vx < volDimX) && (vy >= 0) && (vy < volDimY) && (vz >= 0) && (volDimZ * zPart + vz < vilDimZGlob))
    {

        int2 pixi = make_int2(vx * volStepXY, vy * volStepXY);
        float3 p = get3DPointForPixelAndFrontoParellePlaneRC(pixi, fpPlaneDepth);
        float depthTcP = size(sg_s_t.C - p);
        float fpDepthTcP = frontoParellePlaneTCDepthFor3DPoint(p);
        float2 tpixf;

        // getPixelFor3DPointTC uses __constant__ sg_s_t.*
        getPixelFor3DPointTC(tpixf, p);
        int2 tpix = make_int2((int)(tpixf.x + 0.5f), (int)(tpixf.y + 0.5f));
        int2 tpixMap =
            make_int2((int)(tpixf.x / (float)tcDepthMapStep + 0.5f), (int)(tpixf.y / (float)tcDepthMapStep + 0.5f));

        unsigned int* volume_zyx = get3DBufferAt(volume, volume_s, volume_p, vx, vy, vz);
        unsigned int val = *volume_zyx;
        unsigned int TvalOld = val >> 16;
        unsigned int VvalOld = val & 0xFFFF;

        unsigned int TvalSum = 0;
        unsigned int VvalSum = 0;

        int k = 0;
        for(int xp = -k; xp <= k; xp++)
        {
            for(int yp = -k; yp <= k; yp++)
            {
                int2 tpixMapAct = make_int2(tpixMap.x + xp, tpixMap.y + yp);
                float depthTc = 0.0f;
                float simWeightTc = 1.0f;

                if(useSimilarity == false)
                {
                    depthTc = tex2D(sliceTex, tpixMapAct.x, tpixMapAct.y);
                }
                else
                {
                    float2 depthSimTc = tex2D(sliceTexFloat2, tpixMapAct.x, tpixMapAct.y);
                    depthTc = depthSimTc.x;
                    // simWeightTc = sigmoid(0.1f,1.0f,1.0f,-0.5f,depthSimTc.y);
                    simWeightTc = depthSimTc.y;
                };
                float3 tp =
                    get3DPointForPixelAndDepthFromTC(make_float2(tpixMapAct.x + 0.5f, tpixMapAct.y + 0.5f), depthTc);
                float rcPixSize = computeRcPixSize(tp);
                float tcPixSize = computeTcPixSize(tp);

                if((tcPixSize < rcPixSize * 1.2f) && (rcPixSize < tcPixSize * 1.2f))
                {
                    unsigned int Tval = 0;
                    unsigned int Vval = (((fpDepthTcP > tcMinMaxFpDepth.x) && (fpDepthTcP < tcMinMaxFpDepth.y) &&
                                          (tpix.x >= 0) && (tpix.x < width) && (tpix.y >= 0) && (tpix.y < height))
                                             ? 255
                                             : 0);

                    if((considerNegativeDepthAsInfinity == false) || (depthTc > 0.0f))
                    {
                        int distid = (int)(rintf((depthTcP - depthTc) / stepInDepth));
                        int distidabs = abs(distid);

                        unsigned char udistCostT = (unsigned char)(255.0f * (float)simWeightTc);
                        unsigned char udistCostVis =
                            (unsigned char)((float)distFcnConst3[min(2, distidabs)] * (float)simWeightTc);
                        Tval = (unsigned int)((distid >= 0) ? ((distidabs <= 2) ? udistCostT : 0) : 0);
                        Vval = (unsigned int)((distid < 0) ? udistCostVis : 0);

                        Tval = (((fpDepthTcP > tcMinMaxFpDepth.x) && (fpDepthTcP < tcMinMaxFpDepth.y) &&
                                 (depthTc > 0.0f) && (tpix.x >= 0) && (tpix.x < width) && (tpix.y >= 0) &&
                                 (tpix.y < height))
                                    ? Tval
                                    : 0);
                        Vval = (((fpDepthTcP > tcMinMaxFpDepth.x) && (fpDepthTcP < tcMinMaxFpDepth.y) &&
                                 (depthTc > 0.0f) && (tpix.x >= 0) && (tpix.x < width) && (tpix.y >= 0) &&
                                 (tpix.y < height))
                                    ? Vval
                                    : 0);
                    }

                    TvalSum += Tval;
                    VvalSum += Vval;
                }
            }
        }

        TvalSum = min(65534, TvalSum + TvalOld);
        VvalSum = min(65534, VvalSum + VvalOld);
        val = (TvalSum << 16) | (VvalSum & 0xFFFF);
        *volume_zyx = val;
    }
}

__global__ void volume_update_nModalsMap_kernel(unsigned short* nModalsMap, int nModalsMap_p,
                                                unsigned short* rcIdDepthMap, int rcIdDepthMap_p, int volDimX,
                                                int volDimY, int volDimZ, int volStepXY, int tcDepthMapStep, int width,
                                                int height, int distLimit, int id)
{
    int vx = blockIdx.x * blockDim.x + threadIdx.x;
    int vy = blockIdx.y * blockDim.y + threadIdx.y;

    if((vx >= 0) && (vx < volDimX) && (vy >= 0) && (vy < volDimY))
    {
        unsigned short val = 0;
        unsigned short* nModalsMap_yx = get2DBufferAt(nModalsMap, nModalsMap_p, vx, vy);
        if(id > 0)
        {
            val = *nModalsMap_yx;

            int2 pix = make_int2(vx * volStepXY, vy * volStepXY);
            int vz = *get2DBufferAt(rcIdDepthMap, rcIdDepthMap_p, vx, vy);

            if(vz > 0)
            {
                float fpPlaneDepth = tex2D(depthsTex, vz, 0);
                float fpPlaneDepthP = tex2D(depthsTex, vz - 1, 0);
                float step = fabsf(fpPlaneDepthP - fpPlaneDepth);

                float3 p = get3DPointForPixelAndFrontoParellePlaneRC(pix, fpPlaneDepth);
                float2 tpixf;
                // getPixelFor3DPointTC uses __constant__ sg_s_t.*
                getPixelFor3DPointTC(tpixf, p);
                int2 tpix = make_int2((int)(tpixf.x + 0.5f), (int)(tpixf.y + 0.5f));

                float depthTc = tex2D(sliceTex, tpix.x / tcDepthMapStep, tpix.y / tcDepthMapStep);
                float depthTcP = size(sg_s_t.C - p);
                int distid = (int)(fabsf(depthTc - depthTcP) / step + 0.5f);

                if((depthTc > 0.0f)
                    && (tpix.x >= 0) && (tpix.x < width) && (tpix.y >= 0) && (tpix.y < height)
                    && (distid < distLimit))
                    val++;
            }
        }
        *nModalsMap_yx = val;
    }
}

__global__ void volume_filterRcIdDepthMapByTcDepthMap_kernel(unsigned short* rcIdDepthMap, int rcIdDepthMap_p,
                                                             int volDimX, int volDimY, int volDimZ, int volStepXY,
                                                             int tcDepthMapStep, int width, int height, int distLimit)
{
    int vx = blockIdx.x * blockDim.x + threadIdx.x;
    int vy = blockIdx.y * blockDim.y + threadIdx.y;

    if((vx >= 0) && (vx < volDimX) && (vy >= 0) && (vy < volDimY))
    {
        int2 pix = make_int2(vx * volStepXY, vy * volStepXY);
        unsigned short* rcIdDepthMap_yx = get2DBufferAt(rcIdDepthMap, rcIdDepthMap_p, vx, vy);
        int vz = (int)*rcIdDepthMap_yx;
        if(vz > 0)
        {
            float fpPlaneDepth = tex2D(depthsTex, vz, 0);
            float fpPlaneDepthP = tex2D(depthsTex, vz - 1, 0);
            float step = fabsf(fpPlaneDepthP - fpPlaneDepth);

            float3 p = get3DPointForPixelAndFrontoParellePlaneRC(pix, fpPlaneDepth);
            float2 tpixf;
            getPixelFor3DPointTC(tpixf, p);
            int2 tpix = make_int2((int)(tpixf.x + 0.5f), (int)(tpixf.y + 0.5f));

            float depthTc = tex2D(sliceTex, tpix.x / tcDepthMapStep, tpix.y / tcDepthMapStep);
            float depthTcP = size(sg_s_t.C - p);
            int distid = (int)(fabsf(depthTc - depthTcP) / step + 0.5f);

            * rcIdDepthMap_yx =
                (((depthTc > 0.0f) && (tpix.x >= 0) && (tpix.x < width) && (tpix.y >= 0) && (tpix.y < height))
                     ? ((distid < distLimit) ? 1 : 0)
                     : 0);
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

__global__ void update_GC_volumeXYSliceAtZInt4_kernel(int4* xySlice, int xySlice_p, unsigned int* volume, int volume_s,
                                                      int volume_p, int volDimX, int volDimY, int volDimZ, int vz)
{
    int vx = blockIdx.x * blockDim.x + threadIdx.x;
    int vy = blockIdx.y * blockDim.y + threadIdx.y;
    if((vx >= 0) && (vx < volDimX) && (vy >= 0) && (vy < volDimY) && (vz >= 0) && (vz < volDimZ))
    {
        int4* xySlice_yx = get2DBufferAt(xySlice, xySlice_p, vx, vy);
        int4 M1 = *xySlice_yx;
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

            *xySlice_yx = make_int4(R, dQ, first, wasReached);
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

__global__ void volume_GC_K_getVolumeXYSliceInt4ToUintDimZ_kernel(unsigned int* oxySlice, int oxySlice_p,
                                                                  int4* ixySlice, int ixySlice_p, int volDimX,
                                                                  int volDimY)
{
    int vx = blockIdx.x * blockDim.x + threadIdx.x;
    int vy = blockIdx.y * blockDim.y + threadIdx.y;
    if((vx >= 0) && (vx < volDimX) && (vy >= 0) && (vy < volDimY))
    {
        int4* ixySlice_yx = get2DBufferAt(ixySlice, ixySlice_p, vx, vy);
        unsigned int* oxySlice_yx = get2DBufferAt(oxySlice, oxySlice_p, vx, vy);

        *oxySlice_yx = (unsigned int)ixySlice_yx->z;
    }
}

__global__ void volume_GC_K_initXYSliceInt4_kernel(int4* xySlice, int xySlice_p, unsigned int* volume, int volume_s,
                                                   int volume_p, int volDimX, int volDimY, int vz)
{
    int vx = blockIdx.x * blockDim.x + threadIdx.x;
    int vy = blockIdx.y * blockDim.y + threadIdx.y;
    if((vx >= 0) && (vx < volDimX) && (vy >= 0) && (vy < volDimY))
    {
        int4* xySlice_yx = get2DBufferAt(xySlice, xySlice_p, vx, vy);
        if(vz == 0)
        {
            int R = 0;
            int Q = 10000000;
            int first = 0;
            int wasReached = 0;

            *xySlice_yx = make_int4(R, Q, first, wasReached);
        }
        else
        {
            int4 M1 = *xySlice_yx;
            int R = M1.x;
            int Q = M1.y;
            int first = M1.z;
            int wasReachedStop = M1.w;

            unsigned int* volume_zyx = get3DBufferAt(volume, volume_s, volume_p, vx, vy, vz);
            unsigned int val = *volume_zyx;
            int Ti = (int)(val >> 16);    // T - edge
            int Si = (int)(val & 0xFFFF); // visibility

            R = R + (Si - Ti);

            *xySlice_yx = make_int4(R, Q, first, wasReachedStop);
        }
    }
}

__global__ void update_GC_K_volumeXYSliceAtZInt4_kernel(int4* xySlice, int xySlice_p, unsigned int* volume,
                                                        int volume_s, int volume_p, int volDimX, int volDimY,
                                                        int volDimZ, int vz, int K)
{
    int vx = blockIdx.x * blockDim.x + threadIdx.x;
    int vy = blockIdx.y * blockDim.y + threadIdx.y;
    if((vx >= 0) && (vx < volDimX) && (vy >= 0) && (vy < volDimY) && (vz >= 0) && (vz < volDimZ))
    {
        int4* xySlice_yx = get2DBufferAt(xySlice, xySlice_p, vx, vy);

        int4 M1 = *xySlice_yx;
        int R = M1.x;
        int Q = M1.y;
        int first = M1.z;
        int wasReachedStop = M1.w;

        int wasReached = 0;
        int stop = 0;
        if(wasReachedStop < 0)
        {
            wasReached = 1;
        }
        else
        {
            stop = wasReachedStop;
        }

        /*
        if ((R>100000000)||(Q>100000000))
        {
                first = 0;
                wasReached = 1;
        }
        */

        if((wasReached == 0) && (stop > 0) && (Q + R >= 0))
        {
            wasReached = 1;
        }

        if((wasReached == 0) && (Q + R < 0))
        {
            stop = stop + 1;
            if(stop == K)
            {
                wasReached = 1;
            }
        }

        if(wasReached == 0)
        {
            int val = *get3DBufferAt(volume, volume_s, volume_p, vx, vy, vz);
//            int Si = (int)(val & 0xFFFF); // visibility

            val = *get3DBufferAt(volume, volume_s, volume_p, vx, vy, vz + 1);
            int Ti1 = (int)(val >> 16);    // T - edge
            int Si1 = (int)(val & 0xFFFF); // visibility

            val = *get3DBufferAt(volume, volume_s, volume_p, vx, vy, vz + K);
            int TiK = (int)(val >> 16);    // T - edge
            int SiK = (int)(val & 0xFFFF); // visibility

            // int Ei  = Si;
            int Ei = Si1;
            // int Ei  = 0;

            /*
            if (K>=3) {
                    val = volume[(vz+3)*volume_s+vy*volume_p+vx];
                    int Ti3 = (int)(val >> 16); //T - edge
                    int Si3 = (int)(val & 0xFFFF); //visibility
                    Ei  = Si3;
            }
            */

            if(Ei < Q)
            {
                Q = Ei;
                first = vz + 1;
            }

            int d1 = Si1 - Ti1;
            Q = Q + d1;
            R = R - d1 + (SiK - TiK);

            if((vz == volDimZ - K - 1) && (Q + R < 0))
            {
                wasReached = 1;
            }

            if((vz == volDimZ - K - 1) && (wasReached == 0))
            {
                first = vz + 1;
            }
        }

        if(wasReached == 0)
        {
            wasReachedStop = stop;
        }
        else
        {
            wasReachedStop = -1;
        }

        *xySlice_yx = make_int4(R, Q, first, wasReachedStop);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

__global__ void volume_compute_rDP1_kernel(int2* xySlice, int xySlice_p, int* ovolume, int ovolume_s, int ovolume_p,
                                           unsigned int* volume, int volume_s, int volume_p, int volDimX, int volDimY,
                                           int volDimZ, int vz)
{
    int vx = blockIdx.x * blockDim.x + threadIdx.x;
    int vy = blockIdx.y * blockDim.y + threadIdx.y;

    if((vx >= 0) && (vx < volDimX) && (vy >= 0) && (vy < volDimY) && (vz >= 0) && (vz < volDimZ))
    {
        unsigned int* volume_zyx = get3DBufferAt(volume, volume_s, volume_p, vx, vy, vz);
        unsigned int val = *volume_zyx;

        int* ovolume_zyx = get3DBufferAt(ovolume, ovolume_s, ovolume_p, vx, vy, vz);

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

        if(vz == volDimZ - 1)
        {
            int Q0 = 0;
            int dQ = Si - Ti;
            *xySlice_yx = make_int2(Q0, dQ);
            *ovolume_zyx = Q0 + dQ + EiM1;
        }

        if((vz >= 0) && (vz <= volDimZ - 2))
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

            if(vz > 0)
            {
                *ovolume_zyx = Q0 + dQ + EiM1;
            }

            *xySlice_yx = make_int2(Q0, dQ); // Ei = S(i+1)

            if(vz == 0)
            {
                *ovolume_zyx = Q0 + dQ;
            }
        }
    }
}

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

__global__ void volume_compute_rDP1_volume_minMaxMap_kernel(int2* xySlice, int xySlice_p, int* volume, int volume_s,
                                                            int volume_p, int volDimX, int volDimY, int volDimZ, int z,
                                                            int zPart, int volDimZGlob)
{
    int vx = blockIdx.x * blockDim.x + threadIdx.x;
    int vy = blockIdx.y * blockDim.y + threadIdx.y;

    int vzPart = z;
    int vzGlob = volDimZ * zPart + z;

    if((vx >= 0) && (vx < volDimX) && (vy >= 0) && (vy < volDimY) && (vzGlob >= 0) && (vzGlob < volDimZGlob) &&
       (vzPart < volDimZ))
    {
        int* volume_zyx = get3DBufferAt(volume, volume_s, volume_p, vx, vy, vzPart);
        int val = *volume_zyx;

        int2* xySlice_yx = get2DBufferAt(xySlice, xySlice_p, vx, vy); 

        if(vzGlob == 0)
        {
            *xySlice_yx = make_int2(val, val);
        }
        else
        {
            int minVal = xySlice_yx->x;
            int maxVal = xySlice_yx->y;
            *xySlice_yx = make_int2(min(minVal, val), max(maxVal, val));
        }
    }
}

__global__ void volume_normalize_rDP1_volume_by_minMaxMap_kernel(int2* xySlice, int xySlice_p, int* volume,
                                                                 int volume_s, int volume_p, int volDimX, int volDimY,
                                                                 int volDimZ, int z, int zPart, int volDimZGlob)
{
    int vx = blockIdx.x * blockDim.x + threadIdx.x;
    int vy = blockIdx.y * blockDim.y + threadIdx.y;

    int vzPart = z;
    int vzGlob = volDimZ * zPart + z;

    if((vx >= 0) && (vx < volDimX) && (vy >= 0) && (vy < volDimY) && (vzGlob >= 0) && (vzGlob < volDimZGlob) &&
       (vzPart < volDimZ))
    {
        int2* xySlice_yx = get2DBufferAt(xySlice, xySlice_p, vx, vy); 

        int minVal = xySlice_yx->x;
        int maxVal = xySlice_yx->y;
        int* volume_zyx = get3DBufferAt(volume, volume_s, volume_p, vx, vy, vzPart);
        int val = *volume_zyx;

        if(maxVal - minVal > 0)
        {
            // val = (int)(255.0f*((float)(val-minVal)/(float)(maxVal-minVal)));
            val = (int)(255.0f * ((float)(val - minVal) / (float)(maxVal - minVal)));
        }

        *volume_zyx = val;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

__global__ void volume_filter_VisTVolume_kernel(unsigned int* ovolume, int ovolume_s, int ovolume_p, int volDimX,
                                                int volDimY, int volDimZ, int vz, int K)
{
    int vx = blockIdx.x * blockDim.x + threadIdx.x;
    int vy = blockIdx.y * blockDim.y + threadIdx.y;

    if((vx >= 0) && (vx < volDimX) && (vy >= 0) && (vy < volDimY) && (vz >= 0) && (vz < volDimZ))
    {
        unsigned int* ovolume_zyx = get3DBufferAt(ovolume, ovolume_s, ovolume_p, vx, vy, vz);
        unsigned int val = *ovolume_zyx;
        unsigned int TvalOld = (unsigned int)(val >> 16);    // T - edge
        unsigned int VvalOld = (unsigned int)(val & 0xFFFF); // visibility

        unsigned int Tval = TvalOld;
        unsigned int Vval = VvalOld;

        // unsigned int bestCst = tex2D(sliceTexUInt,vx,0);
        // int wsh = abs(K);
        int wsh = 0;
        for(int xp = vx - wsh; xp <= vx + wsh; xp++)
        {
            for(int yp = vy - wsh; yp <= vy + wsh; yp++)
            {
                unsigned int valN = tex2D(sliceTexUInt, xp, yp);
                unsigned int VvalN = (unsigned int)(valN & 0xFFFF); // visibility

                if(K > 0)
                {
                    Vval = max(Vval, VvalN);
                }

                // if (K>0) {
                //	Tval = max(Tval,TvalN);
                //}
            }
        }

        val = (Tval << 16) | (Vval & 0xFFFF);
        *ovolume_zyx = val;
    }
}

} // namespace depthMap
} // namespace aliceVision
