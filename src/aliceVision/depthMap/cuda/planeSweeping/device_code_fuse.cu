// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

namespace aliceVision {
namespace depthMap {

/**
 * @param[in] s: iteration over nSamplesHalf
 */
__global__ void fuse_computeGaussianKernelVotingSampleMap_kernel(float* out_gsvSampleMap, int out_gsvSampleMap_p,
                                                                 float2* depthSimMap, int depthSimMap_p,
                                                                 float2* midDepthPixSizeMap, int midDepthPixSizeMap_p,
                                                                 int width, int height, float s, int idCam,
                                                                 float samplesPerPixSize, float twoTimesSigmaPowerTwo)
{
    const int x = blockIdx.x * blockDim.x + threadIdx.x;
    const int y = blockIdx.y * blockDim.y + threadIdx.y;

    if(x >= width || y >= height)
        return;

    const float2 midDepthPixSize = *get2DBufferAt(midDepthPixSizeMap, midDepthPixSizeMap_p, x, y);
    const float2 depthSim = *get2DBufferAt(depthSimMap, depthSimMap_p, x, y);
    float* out_gsvSample_ptr = get2DBufferAt(out_gsvSampleMap, out_gsvSampleMap_p, x, y);
    float gsvSample = (idCam == 0) ? 0.0f : *out_gsvSample_ptr;

    if((midDepthPixSize.x > 0.0f) && (depthSim.x > 0.0f))
    {
        const float depthStep = midDepthPixSize.y / samplesPerPixSize;
        const float i = (midDepthPixSize.x - depthSim.x) / depthStep;
        const float sim = -sigmoid(0.0f, 1.0f, 0.7f, -0.7f, depthSim.y);
        gsvSample += sim * expf(-((i - s) * (i - s)) / twoTimesSigmaPowerTwo);
    }
    *out_gsvSample_ptr = gsvSample;
}


__global__ void fuse_updateBestGaussianKernelVotingSampleMap_kernel(float2* bestGsvSampleMap, int bestGsvSampleMap_p,
                                                                    float* gsvSampleMap, int gsvSampleMap_p, int width,
                                                                    int height, float s, int id)
{
    const int x = blockIdx.x * blockDim.x + threadIdx.x;
    const int y = blockIdx.y * blockDim.y + threadIdx.y;

    if(x >= width || y >= height)
        return;

    const float gsvSampleX = *get2DBufferAt(gsvSampleMap, gsvSampleMap_p, x, y);
    float2* bestGsvSample_ptr = get2DBufferAt(bestGsvSampleMap, bestGsvSampleMap_p, x, y);

    if(id == 0 || gsvSampleX < bestGsvSample_ptr->x)
    {
        *bestGsvSample_ptr = make_float2(gsvSampleX, s);
    }
}

__global__ void fuse_computeFusedDepthSimMapFromBestGaussianKernelVotingSampleMap_kernel(
    float2* oDepthSimMap, int oDepthSimMap_p, float2* bestGsvSampleMap, int bestGsvSampleMap_p,
    float2* midDepthPixSizeMap, int midDepthPixSizeMap_p, int width, int height, float samplesPerPixSize)
{
    const int x = blockIdx.x * blockDim.x + threadIdx.x;
    const int y = blockIdx.y * blockDim.y + threadIdx.y;

    if(x >= width || y >= height)
        return;

    const float2 bestGsvSample = *get2DBufferAt(bestGsvSampleMap, bestGsvSampleMap_p, x, y);
    const float2 midDepthPixSize = *get2DBufferAt(midDepthPixSizeMap, midDepthPixSizeMap_p, x, y);
    const float depthStep = midDepthPixSize.y / samplesPerPixSize;

    // normalize similarity to -1,0
    // figure; t = -5.0:0.01:0.0; plot(t,sigmoid(0.0,-1.0,6.0,-0.4,t,0));
    // bestGsvSample.x = sigmoid(0.0f, -1.0f, 6.0f, -0.4f, bestGsvSample.x);
    float2* oDepthSim = get2DBufferAt(oDepthSimMap, oDepthSimMap_p, x, y);

    if(midDepthPixSize.x <= 0.0f)
    {
        *oDepthSim = make_float2(-1.0f, 1.0f);
    }
    else
    {
        *oDepthSim = make_float2(midDepthPixSize.x - bestGsvSample.y * depthStep, bestGsvSample.x);
    }
}

__global__ void fuse_getOptDeptMapFromOptDepthSimMap_kernel(float* optDepthMap, int optDepthMap_p,
                                                            float2* optDepthMapSimMap, int optDepthMapSimMap_p,
                                                            int partWidth, int partHeight)
{
    const int x = blockIdx.x * blockDim.x + threadIdx.x;
    const int y = blockIdx.y * blockDim.y + threadIdx.y;

    if(x < partWidth && y < partHeight)
    {
        *get2DBufferAt(optDepthMap, optDepthMap_p, x, y) = get2DBufferAt(optDepthMapSimMap, optDepthMapSimMap_p, x, y)->x;
    }
}

/**
 * @return (smoothStep, energy)
 */
__device__ float2 getCellSmoothStepEnergy( int rc_cam_cache_idx, cudaTextureObject_t depthTex, const int2& cell0,
                                          int yFrom)
{
    float2 out = make_float2(0.0f, 180.0f);

    // Get pixel depth from the depth texture
    // Note: we do not use 0.5f offset as we use nearest neighbor interpolation
    float d0 = tex2D<float>(depthTex, float(cell0.x), float(cell0.y - yFrom));

    // Early exit: depth is <= 0
    if(d0 <= 0.0f)
        return out;

    // Consider the neighbor pixels
    const int2 cellL = cell0 + make_int2(0, -1); // Left
    const int2 cellR = cell0 + make_int2(0, 1);	 // Right
    const int2 cellU = cell0 + make_int2(-1, 0); // Up
    const int2 cellB = cell0 + make_int2(1, 0);	 // Bottom

    // Get associated depths from depth texture
    const float dL = tex2D<float>(depthTex, float(cellL.x), float(cellL.y - yFrom));
    const float dR = tex2D<float>(depthTex, float(cellR.x), float(cellR.y - yFrom));
    const float dU = tex2D<float>(depthTex, float(cellU.x), float(cellU.y - yFrom));
    const float dB = tex2D<float>(depthTex, float(cellB.x), float(cellB.y - yFrom));

    // Get associated 3D points
    const float3 p0 = get3DPointForPixelAndDepthFromRC(rc_cam_cache_idx, cell0, d0);
    const float3 pL = get3DPointForPixelAndDepthFromRC(rc_cam_cache_idx, cellL, dL);
    const float3 pR = get3DPointForPixelAndDepthFromRC(rc_cam_cache_idx, cellR, dR);
    const float3 pU = get3DPointForPixelAndDepthFromRC(rc_cam_cache_idx, cellU, dU);
    const float3 pB = get3DPointForPixelAndDepthFromRC(rc_cam_cache_idx, cellB, dB);

    // Compute the average point based on neighbors (cg)
    float3 cg = make_float3(0.0f, 0.0f, 0.0f);
    float n = 0.0f;

    if(dL > 0.0f) { cg = cg + pL; n++; }
    if(dR > 0.0f) { cg = cg + pR; n++; }
    if(dU > 0.0f) { cg = cg + pU; n++; }
    if(dB > 0.0f) { cg = cg + pB; n++; }

    // If we have at least one valid depth
    if(n > 1.0f)
    {
        cg = cg / n; // average of x, y, depth
        float3 vcn = camsBasesDev[rc_cam_cache_idx].C - p0;
        normalize(vcn);
        // pS: projection of cg on the line from p0 to camera
        const float3 pS = closestPointToLine3D(cg, p0, vcn);
        // keep the depth difference between pS and p0 as the smoothing step
        out.x = size(camsBasesDev[rc_cam_cache_idx].C - pS) - d0;
    }

    float e = 0.0f;
    n = 0.0f;

    if(dL > 0.0f && dR > 0.0f)
    {
        // Large angle between neighbors == flat area => low energy
        // Small angle between neighbors == non-flat area => high energy
        e = fmaxf(e, (180.0f - angleBetwABandAC(p0, pL, pR)));
        n++;
    }
    if(dU > 0.0f && dB > 0.0f)
    {
        e = fmaxf(e, (180.0f - angleBetwABandAC(p0, pU, pB)));
        n++;
    }
    // The higher the energy, the less flat the area
    if(n > 0.0f)
        out.y = e;

    return out;
}

__global__ void fuse_optimizeDepthSimMap_kernel(cudaTextureObject_t rc_tex,
                                                int rc_cam_cache_idx,
                                                cudaTextureObject_t imgVarianceTex,
                                                cudaTextureObject_t depthTex,
                                                float2* out_optDepthSimMap, int optDepthSimMap_p,
                                                const float2* roughDepthPixSizeMap, int roughDepthPixSizeMap_p,
                                                const float2* fineDepthSimMap, int fineDepthSimMap_p, 
                                                int partWidth, int partHeight, int iter, float samplesPerPixSize, int yFrom)
{
    const int x = blockIdx.x * blockDim.x + threadIdx.x;
    const int y = blockIdx.y * blockDim.y + threadIdx.y;

    if(x >= partWidth || y >= partHeight)
        return;

    const int2 pix = make_int2(x, y + yFrom);

    const float2 roughDepthPixSize = *get2DBufferAt(roughDepthPixSizeMap, roughDepthPixSizeMap_p, x, y);
    const float roughDepth = roughDepthPixSize.x;
    const float roughPixSize = roughDepthPixSize.y;

    const float2 fineDepthSim = *get2DBufferAt(fineDepthSimMap, fineDepthSimMap_p, x, y);
    const float fineDepth = fineDepthSim.x;
    const float fineSim = fineDepthSim.y;

    float2* out_optDepthSim_ptr = get2DBufferAt(out_optDepthSimMap, optDepthSimMap_p, x, y);
    float2 out_optDepthSim = (iter == 0) ? make_float2(roughDepth, fineSim) : *out_optDepthSim_ptr;

    const float depthOpt = out_optDepthSim.x;

    if (depthOpt > 0.0f)
    {
        const float2 depthSmoothStepEnergy = getCellSmoothStepEnergy(rc_cam_cache_idx, depthTex, pix, yFrom); // (smoothStep, energy)
        float stepToSmoothDepth = depthSmoothStepEnergy.x;
        stepToSmoothDepth = copysignf(fminf(fabsf(stepToSmoothDepth), roughPixSize / 10.0f), stepToSmoothDepth);
        const float depthEnergy = depthSmoothStepEnergy.y; // max angle with neighbors
        float stepToFineDM = fineDepth - depthOpt; // distance to refined/noisy input depth map
        stepToFineDM = copysignf(fminf(fabsf(stepToFineDM), roughPixSize / 10.0f), stepToFineDM);

        const float stepToRoughDM = roughDepth - depthOpt; // distance to smooth/robust input depth map
        const float imgColorVariance = tex2D<float>(imgVarianceTex, float(x) + 0.5f, float(y) + 0.5f);
        const float colorVarianceThresholdForSmoothing = 20.0f;
        const float angleThresholdForSmoothing = 30.0f; // 30

        // https://www.desmos.com/calculator/kob9lxs9qf
        const float weightedColorVariance = sigmoid2(5.0f, angleThresholdForSmoothing, 40.0f, colorVarianceThresholdForSmoothing, imgColorVariance);

        // https://www.desmos.com/calculator/jwhpjq6ppj
        const float fineSimWeight = sigmoid(0.0f, 1.0f, 0.7f, -0.7f, fineSim);

        // if geometry variation is bigger than color variation => the fineDM is considered noisy

        // if depthEnergy > weightedColorVariance   => energyLowerThanVarianceWeight=0 => smooth
        // else:                                    => energyLowerThanVarianceWeight=1 => use fineDM
        // weightedColorVariance max value is 30, so if depthEnergy > 30 (which means depthAngle < 150�) energyLowerThanVarianceWeight will be 0
        // https://www.desmos.com/calculator/jzbweilb85
        const float energyLowerThanVarianceWeight = sigmoid(0.0f, 1.0f, 30.0f, weightedColorVariance, depthEnergy); // TODO: 30 => 60

        // https://www.desmos.com/calculator/ilsk7pthvz
        const float closeToRoughWeight = 1.0f - sigmoid(0.0f, 1.0f, 10.0f, 17.0f, fabsf(stepToRoughDM / roughPixSize)); // TODO: 10 => 30

        // f(z) = c1 * s1(z_rought - z)^2 + c2 * s2(z-z_fused)^2 + coeff3 * s3*(z-z_smooth)^2

        const float depthOptStep = closeToRoughWeight * stepToRoughDM + // distance to smooth/robust input depth map
                                   (1.0f - closeToRoughWeight) * (energyLowerThanVarianceWeight * fineSimWeight * stepToFineDM + // distance to refined/noisy
                                                                 (1.0f - energyLowerThanVarianceWeight) * stepToSmoothDepth); // max angle in current depthMap

        out_optDepthSim.x = depthOpt + depthOptStep;

        out_optDepthSim.y = (1.0f - closeToRoughWeight) * (energyLowerThanVarianceWeight * fineSimWeight * fineSim +
            (1.0f - energyLowerThanVarianceWeight) * (depthEnergy / 20.0f));
    }

    *out_optDepthSim_ptr = out_optDepthSim;
}

} // namespace depthMap
} // namespace aliceVision
