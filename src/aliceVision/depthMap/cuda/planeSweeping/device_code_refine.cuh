// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "aliceVision/depthMap/cuda/deviceCommon/device_matrix.cuh"
#include "aliceVision/depthMap/cuda/deviceCommon/device_color.cuh"

#define GRIFF_TEST

namespace aliceVision {
namespace depthMap {

__global__ void refine_selectPartOfDepthMapNearFPPlaneDepth_kernel(float* o0depthMap, int o0depthMap_p,
                                                                   float* o1depthMap, int o1depthMap_p,
                                                                   float* idepthMap, int idepthMap_p, int width,
                                                                   int height, float fpPlaneDepth,
                                                                   float fpPlaneDepthNext);

__global__ void refine_dilateDepthMap_kernel(
    cudaTextureObject_t depthsTex,
    float* depthMap, int depthMap_p,
    int width, int height, const float gammaC );

__global__ void refine_dilateFPPlaneDepthMapXpYp_kernel(float* fpPlaneDepthMap, int fpPlaneDepthMap_p, float* maskMap,
                                                        int maskMap_p, int width, int height, int xp, int yp,
                                                        float fpPlaneDepth);

__global__ void refine_convertFPPlaneDepthMapToDepthMap_kernel(float* depthMap, int depthMap_p, float* fpPlaneDepthMap,
                                                               int fpPlaneDepthMap_p, int width, int height);

__global__ void refine_computeDepthsMapFromDepthMap_kernel(float3* depthsMap, int depthsMap_p, float* depthMap,
                                                           int depthMap_p, int width, int height, bool moveByTcOrRc,
                                                           float step);

__global__ void refine_reprojTarTexLABByDepthsMap_kernel(
    cudaTextureObject_t t4tex,
    float3* depthsMap, int depthsMap_p,
    uchar4* tex, int tex_p,
    int width, int height, int id);


__global__ void refine_reprojTarTexLABByDepthMap_kernel(
    cudaTextureObject_t t4tex,
    float* depthMap, int depthMap_p,
    uchar4* tex, int tex_p,
    int width, int height);

__global__ void refine_reprojTarTexLABByDepthMapMovedByStep_kernel(
    cudaTextureObject_t t4tex,
    float* depthMap, int depthMap_p,
    uchar4* tex, int tex_p,
    int width, int height, bool moveByTcOrRc,
    float step);

__global__ void refine_compYKNCCSimMap_kernel(
    cudaTextureObject_t rTexU4,
    cudaTextureObject_t tTexU4,
    float* osimMap, int osimMap_p,
    float* depthMap, int depthMap_p,
    int width, int height, int wsh, const float gammaC, const float gammaP);

__global__ void refine_compYKNCCSim_kernel(
    cudaTextureObject_t rTexU4,
    cudaTextureObject_t tTexU4,
    float3* osims, int osims_p,
    int id,
    float* depthMap, int depthMap_p,
    int width, int height, int wsh, const float gammaC, const float gammaP);

__global__ void refine_compYKNCCSimOptGammaC_kernel(
    cudaTextureObject_t rTexU4,
    cudaTextureObject_t tTexU4,
    float3* osims, int osims_p,
    int id,
    float* depthMap, int depthMap_p,
    int width, int height, int wsh, const float gammaP );

__global__ void refine_computeBestDepthSimMaps_kernel(float* osim, int osim_p, float* odpt, int odpt_p, float3* isims,
                                                      int isims_p, float3* idpts, int idpts_p, int width, int height,
                                                      float simThr);

__global__ void refine_fuseThreeDepthSimMaps_kernel(float* osim, int osim_p, float* odpt, int odpt_p, float* isimLst,
                                                    int isimLst_p, float* idptLst, int idptLst_p, float* isimAct,
                                                    int isimAct_p, float* idptAct, int idptAct_p, int width, int height,
                                                    float simThr);

#ifdef GRIFF_TEST
__global__ void refine_compYKNCCSimMapPatch_kernel_A(
    cudaTextureObject_t r4tex,
    cudaTextureObject_t t4tex,
    const float* depthMap, int depthMap_p,
    int width, int height, int wsh, const float gammaC,
    const float gammaP, const float epipShift, const float tcStep,
    bool moveByTcOrRc, int xFrom, int imWidth, int imHeight,
    float3* lastThreeSimsMap, int lastThreeSimsMap_p, const int dimension );
#endif

#ifdef GRIFF_TEST
__global__ void refine_compUpdateYKNCCSimMapPatch_kernel(
    cudaTextureObject_t r4tex,
    cudaTextureObject_t t4tex,
    float* osimMap, int osimMap_p,
    float* odptMap, int odptMap_p,
    const float* depthMap, int depthMap_p, int width, int height,
    int wsh, const float gammaC, const float gammaP,
    const float epipShift,
    const int ntcsteps,
    bool moveByTcOrRc, int xFrom, int imWidth, int imHeight,
    float3* lastThreeSimsMap, int lastThreeSimsMap_p );
#else
__global__ void refine_compUpdateYKNCCSimMapPatch_kernel(
    cudaTextureObject_t r4tex,
    cudaTextureObject_t t4tex,
    float* osimMap, int osimMap_p,
    float* odptMap, int odptMap_p,
    const float* depthMap, int depthMap_p, int width, int height,
    int wsh, const float gammaC, const float gammaP,
    const float epipShift,
    const float tcStep,    // changing in loop
    int id,                // changing in loop
    bool moveByTcOrRc, int xFrom, int imWidth, int imHeight);
#endif

__global__ void refine_coputeDepthStepMap_kernel(float* depthStepMap, int depthStepMap_p, float* depthMap,
                                                 int depthMap_p, int width, int height, bool moveByTcOrRc);

__global__ void refine_compYKNCCDepthSimMapPatch_kernel(
    cudaTextureObject_t r4tex,
    cudaTextureObject_t t4tex,
    float2* oDepthSimMap, int oDepthSimMap_p, float* depthMap,
    int depthMap_p, int width, int height, int wsh,
    const float gammaC, const float gammaP, const float epipShift,
    const float tcStep, bool moveByTcOrRc);

__global__ void refine_compYKNCCSimMapPatch_kernel(
    cudaTextureObject_t r4tex,
    cudaTextureObject_t t4tex,
    float* osimMap, int osimMap_p,
    const float* depthMap, int depthMap_p,
    int width, int height, int wsh, const float gammaC,
    const float gammaP, const float epipShift, const float tcStep,
    bool moveByTcOrRc, int xFrom, int imWidth, int imHeight);

__global__ void refine_compYKNCCSimMapPatchDMS_kernel(
    cudaTextureObject_t r4tex,
    cudaTextureObject_t t4tex,
    float* osimMap, int osimMap_p,
    float* depthMap, int depthMap_p,
    int width, int height, int wsh, const float gammaC,
    const float gammaP, const float epipShift,
    const float depthMapShift);

__global__ void refine_setLastThreeSimsMap_kernel(float3* lastThreeSimsMap, int lastThreeSimsMap_p, float* simMap,
                                                  int simMap_p, int width, int height, int id);

__global__ void refine_computeDepthSimMapFromLastThreeSimsMap_kernel(float* osimMap, int osimMap_p, float* iodepthMap,
                                                                     int iodepthMap_p, float3* lastThreeSimsMap,
                                                                     int lastThreeSimsMap_p, int width, int height,
                                                                     bool moveByTcOrRc, int xFrom);

__global__ void refine_updateLastThreeSimsMap_kernel(float3* lastThreeSimsMap, int lastThreeSimsMap_p, float* simMap,
                                                     int simMap_p, int width, int height, int id);

__global__ void refine_updateBestStatMap_kernel(float4* bestStatMap, int bestStatMap_p, float3* lastThreeSimsMap,
                                                int lastThreeSimsMap_p, int width, int height, int id, int nids,
                                                float tcStepBefore, float tcStepAct);

__global__ void refine_computeDepthSimMapFromBestStatMap_kernel(float* simMap, int simMap_p, float* depthMap,
                                                                int depthMap_p, float4* bestStatMap, int bestStatMap_p,
                                                                int width, int height, bool moveByTcOrRc);

__global__ void refine_reprojTarTexLABByRcTcDepthsMap_kernel(
    cudaTextureObject_t t4tex,
    cudaTextureObject_t depthsTex,
    uchar4* tex, int tex_p,
    float* rcDepthMap, int rcDepthMap_p,
    int width, int height,
    float depthMapShift);

__global__ void refine_compPhotoErr_kernel(
    cudaTextureObject_t rTexU4,
    cudaTextureObject_t tTexU4,
    float* osimMap, int osimMap_p,
    float* depthMap, int depthMap_p,
    int width, int height, double beta );

__global__ void refine_compPhotoErrStat_kernel(
    cudaTextureObject_t rTexU4,
    cudaTextureObject_t tTexU4,
    float* occMap, int occMap_p,
    float4* ostat1Map, int ostat1Map_p,
    float* depthMap, int depthMap_p,
    int width, int height, double beta );

__global__ void refine_compPhotoErrABG_kernel(
    cudaTextureObject_t f4Tex,
    cudaTextureObject_t rTexU4,
    cudaTextureObject_t tTexU4,
    float* osimMap, int osimMap_p,
    int width, int height );

__global__ void refine_reprojTarSobelAndDPIXTCDRCRcTcDepthsMap_kernel(
    cudaTextureObject_t tTexU4,
    float4* tex, int tex_p,
    float* rcDepthMap, int rcDepthMap_p,
    int width, int height, float depthMapShift);

__global__ void refine_computeRcTcDepthMap_kernel(
    cudaTextureObject_t depthsTex,
    float* rcDepthMap, int rcDepthMap_p,
    int width, int height, float pixSizeRatioThr );

} // namespace depthMap
} // namespace aliceVision
