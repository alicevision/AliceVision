// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

namespace aliceVision {
namespace depthMap {

inline __device__ void volume_computePatch( const CameraStructBase* rc_cam_s,
                                            const CameraStructBase* tc_cam_s,
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

__global__ void volume_init_kernel( float* volume, int volume_s, int volume_p,
                                    int volDimX, int volDimY )
{
    const int vx = blockIdx.x * blockDim.x + threadIdx.x;
    const int vy = blockIdx.y * blockDim.y + threadIdx.y;
    const int vz = blockIdx.z * blockDim.z + threadIdx.z;

    if( vx >= volDimX ) return;
    if( vy >= volDimY ) return;

    *get3DBufferAt(volume, volume_s, volume_p, vx, vy, vz) = 9999.0f;
}

__global__ void volume_slice_kernel(
                                    cudaTextureObject_t rc_tex,
                                    cudaTextureObject_t tc_tex,
                                    const CameraStructBase* rc_cam_s,
                                    const CameraStructBase* tc_cam_s,
                                    const float* depths_d,
                                    const int lowestUsedDepth,
                                    const int nbDepthsToSearch,
                                    int rcWidth, int rcHeight,
                                    int tcWidth, int tcHeight,
                                    int wsh,
                                    const float gammaC, const float gammaP,
                                    float* volume_1st,
                                    int volume1st_s, int volume1st_p,
                                    float* volume_2nd,
                                    int volume2nd_s, int volume2nd_p,
                                    int volStepXY,
                                    int volDimX, int volDimY)
{
    /*
     * Note !
     * volDimX == width  / volStepXY
     * volDimY == height / volStepXY
     * width and height are needed to compute transformations,
     * volDimX and volDimY may be the number of samples, reducing memory or computation
     */

    const int vx = blockIdx.x * blockDim.x + threadIdx.x;
    const int vy = blockIdx.y * blockDim.y + threadIdx.y;
    const int vz = blockIdx.z; // * blockDim.z + threadIdx.z;

    if( vx >= volDimX || vy >= volDimY )
        return;
    if (vz >= nbDepthsToSearch)
      return;

    const int x = vx * volStepXY;
    const int y = vy * volStepXY;

    if(x >= rcWidth || y >= rcHeight)
        return;

    const int zIndex = lowestUsedDepth + vz;
    const float fpPlaneDepth = depths_d[zIndex];

    /*
    int verbose = (vx % 100 == 0 && vy % 100 == 0 && vz % 100 == 0);

    if (verbose)
    {
        printf("______________________________________\n");
        printf("volume_slice_kernel: vx: %i, vy: %i, vz: %i, x: %i, y: %i\n", vx, vy, vz, x, y);
        printf("volume_slice_kernel: volStepXY: %i, volDimX: %i, volDimY: %i\n", volStepXY, volDimX, volDimY);
        printf("volume_slice_kernel: wsh: %i\n", wsh);
        printf("volume_slice_kernel: rcWidth: %i, rcHeight: %i\n", rcWidth, rcHeight);
        printf("volume_slice_kernel: lowestUsedDepth: %i, nbDepthsToSearch: %i\n", lowestUsedDepth, nbDepthsToSearch);
        printf("volume_slice_kernel: zIndex: %i, fpPlaneDepth: %f\n", zIndex, fpPlaneDepth);
        printf("volume_slice_kernel: gammaC: %f, gammaP: %f, epipShift: %f\n", gammaC, gammaP, epipShift);
        printf("______________________________________\n");
    }
    */
    patch ptcho;
    volume_computePatch(rc_cam_s, tc_cam_s, ptcho, fpPlaneDepth, make_int2(x, y)); // no texture use

    float fsim = compNCCby3DptsYK(rc_tex, tc_tex,
                                  rc_cam_s, tc_cam_s,
                                  ptcho, wsh,
                                  rcWidth, rcHeight,
                                  tcWidth, tcHeight,
                                  gammaC, gammaP);

    const float fminVal = -1.0f;
    const float fmaxVal = 1.0f;
    fsim = (fsim - fminVal) / (fmaxVal - fminVal);
    fsim = fminf(1.0f, fmaxf(0.0f, fsim));
    fsim *= 255.0f; // Currently needed for the next step... (TODO: should be removed at some point)

    float* fsim_1st = get3DBufferAt(volume_1st, volume1st_s, volume1st_p, vx, vy, zIndex);
    float* fsim_2nd = get3DBufferAt(volume_2nd, volume2nd_s, volume2nd_p, vx, vy, zIndex);

    if (fsim < *fsim_1st)
    {
        *fsim_2nd = *fsim_1st;
        *fsim_1st = fsim;
    }
    else if (fsim < *fsim_2nd)
    {
      *fsim_2nd = fsim;
    }
}


__global__ void volume_retrieveBestZ_kernel(
  float2* bestZ, int bestZ_s,
  const float* simVolume, int simVolume_s, int simVolume_p,
  int volDimX, int volDimY, int volDimZ, int zBorder)
{
  int x = blockIdx.x * blockDim.x + threadIdx.x;
  int y = blockIdx.y * blockDim.y + threadIdx.y;
  
  if(x >= volDimX || y >= volDimY)
    return;

  float2* outPix = get2DBufferAt(bestZ, bestZ_s, x, y);
  outPix->x = -1;
  outPix->y = 9999.0;
  for (int z = 0; z < volDimZ; ++z)
  {
    const float simAtZ = *get3DBufferAt(simVolume, simVolume_s, simVolume_p, x, y, z);
    if (simAtZ < outPix->y)
    {
      outPix->x = z;
      outPix->y = simAtZ;
    }
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

__global__ void volume_transposeAddAvgVolume_kernel(float* volumeT, int volumeT_s, int volumeT_p,
                                                    const float* volume, int volume_s, int volume_p, int volDimX,
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

        float* oldVal_ptr = get3DBufferAt(volumeT, volumeT_s, volumeT_p, vTx, vTy, vTz);
        float newVal = *get3DBufferAt(volume, volume_s, volume_p, vx, vy, vz);
        float val = (*oldVal_ptr * (float)lastN + (float)newVal) / (float)(lastN + 1);

        *oldVal_ptr = val;
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

__global__ void volume_agregateCostVolumeAtZ_kernel(float* volume, int volume_s, int volume_p,
                                                    float* xsliceBestInColCst, int volDimX, int volDimY,
                                                    int volDimZ, int vz, float P1, float P2,
                                                    bool transfer)
{
    int vx = blockIdx.x * blockDim.x + threadIdx.x;
    int vy = blockIdx.y * blockDim.y + threadIdx.y;

    if((vx >= 0) && (vx < volDimX) && (vy >= 0) && (vy < volDimY) && (vz >= 0) && (vz < volDimZ))
    {
        float* sim_ptr = get3DBufferAt(volume, volume_s, volume_p, vx, vy, vz);
        float sim = *sim_ptr;
        float pathCost = (transfer == true) ? sim : 255.0f;

        if((vz >= 1) && (vy >= 1) && (vy < volDimY - 1))
        {
            float bestCostM = xsliceBestInColCst[vx];
            float pathCostMDM1 = volume[(vz - 1) * volume_s + (vy - 1) * volume_p + vx];
            float pathCostMD = volume[(vz - 1) * volume_s + (vy + 0) * volume_p + vx];
            float pathCostMDP1 = volume[(vz - 1) * volume_s + (vy + 1) * volume_p + vx];
            pathCost = sim + multi_fminf(pathCostMD, pathCostMDM1 + P1, pathCostMDP1 + P1, bestCostM + P2) - bestCostM;
            pathCost = pathCost;
        }

        *sim_ptr = pathCost;
    }
}

__global__ void volume_computeBestXSlice_kernel(float* xySlice, int xySlice_p, float* xsliceBestInColCst, int volDimX, int volDimY)
{
    int vx = blockIdx.x * blockDim.x + threadIdx.x;

    if((vx >= 0) && (vx < volDimX))
    {
        float bestCst = *get2DBufferAt(xySlice, xySlice_p, vx, 0);

        for(int vy = 0; vy < volDimY; vy++)
        {
            float cst = *get2DBufferAt(xySlice, xySlice_p, vx, vy);
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
__global__ void volume_agregateCostVolumeAtZinSlices_kernel(cudaTextureObject_t rc_tex,
                                                            float* xySliceForZ, int xySliceForZ_p,
                                                            const float* xySliceForZM1, int xySliceForZM1_p,
                                                            const float* xSliceBestInColSimForZM1,
                                                            float* volSimT, int volSimT_s, int volSimT_p,
                                                            int volDimX, int volDimY, int volDimZ, 
                                                            int vz, unsigned int _P1, unsigned int _P2,
                                                            int dimTrnX, bool doInvZ)
{
    int vx = blockIdx.x * blockDim.x + threadIdx.x;
    int vy = blockIdx.y * blockDim.y + threadIdx.y;

    if((vx >= 0) && (vx < volDimX) && (vy >= 0) && (vy < volDimY) && (vz >= 0) && (vz < volDimZ))
    {
        float* sim_yx = get2DBufferAt(xySliceForZ, xySliceForZ_p, vx, vy);
        float sim = *sim_yx;
        float pathCost = 255.0f;

        if((vz >= 1) && (vy >= 1) && (vy < volDimY - 1))
        {
            int z = doInvZ ? volDimZ - vz : vz;
            int z1 = doInvZ ? z + 1 : z - 1; // M1
            int imX0 = (dimTrnX == 0) ? vx : z; // current
            int imY0 = (dimTrnX == 0) ?  z : vx;
            int imX1 = (dimTrnX == 0) ? vx : z1; // M1
            int imY1 = (dimTrnX == 0) ? z1 : vx;
            float4 gcr0 = 255.0f * tex2D<float4>(rc_tex, (float)imX0 + 0.5f, (float)imY0 + 0.5f);
            float4 gcr1 = 255.0f * tex2D<float4>(rc_tex, (float)imX1 + 0.5f, (float)imY1 + 0.5f);
            float deltaC = Euclidean3(gcr0, gcr1);
            // unsigned int P1 = (unsigned int)sigmoid(5.0f,20.0f,60.0f,10.0f,deltaC);
            float P1 = _P1;
            // 15.0 + (255.0 - 15.0) * (1.0 / (1.0 + exp(10.0 * ((x - 20.) / 80.))))
            float P2 = sigmoid(15.0f, 255.0f, 80.0f, 20.0f, deltaC);
            // float P2 = _P2;

            float bestCostInColM1 = xSliceBestInColSimForZM1[vx];
            float pathCostMDM1 = *get2DBufferAt(xySliceForZM1, xySliceForZM1_p, vx, vy - 1); // M1: minus 1 over depths
            float pathCostMD   = *get2DBufferAt(xySliceForZM1, xySliceForZM1_p, vx, vy);
            float pathCostMDP1 = *get2DBufferAt(xySliceForZM1, xySliceForZM1_p, vx, vy + 1); // P1: plus 1 over depths
            float minCost = multi_fminf(pathCostMD, pathCostMDM1 + P1, pathCostMDP1 + P1, bestCostInColM1 + P2);

            // if 'pathCostMD' is the minimal value of the depth
            pathCost = sim + minCost - bestCostInColM1;
        }
        float* volume_zyx = get3DBufferAt(volSimT, volSimT_s, volSimT_p, vx, vy, vz);
        *volume_zyx = pathCost;
        *sim_yx = pathCost;
    }
}

} // namespace depthMap
} // namespace aliceVision
