// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

namespace aliceVision {
namespace depthMap {

/**
 * @param[in] s: iteration over nSamplesHalf
 */
__global__ void fuse_computeGaussianKernelVotingSampleMap_kernel(float* out_gsvSampleMap, int out_gsvSampleMap_p,
                                                                 float2* depthSimMap, int depthSimMap_p,
                                                                 float2* midDepthPixSizeMap, int midDepthPixSizeMap_p,
                                                                 int width, int height, float s, int idCam,
                                                                 float samplesPerPixSize, float twoTimesSigmaPowerTwo);


__global__ void fuse_updateBestGaussianKernelVotingSampleMap_kernel(float2* bestGsvSampleMap, int bestGsvSampleMap_p,
                                                                    float* gsvSampleMap, int gsvSampleMap_p, int width,
                                                                    int height, float s, int id);

__global__ void fuse_computeFusedDepthSimMapFromBestGaussianKernelVotingSampleMap_kernel(
    float2* oDepthSimMap, int oDepthSimMap_p, float2* bestGsvSampleMap, int bestGsvSampleMap_p,
    float2* midDepthPixSizeMap, int midDepthPixSizeMap_p, int width, int height, float samplesPerPixSize);

__global__ void fuse_getOptDeptMapFromOPtDepthSimMap_kernel(
                    float* optDepthMap, int optDepthMap_p,
                    float2* optDepthMapSimMap, int optDepthMapSimMap_p,
                    int width, int height);

__global__ void fuse_optimizeDepthSimMap_kernel(
                    cudaTextureObject_t r4tex,
                    cudaTextureObject_t depthsTex,
                    float2* out_optDepthSimMap, int optDepthSimMap_p,
                    float2* midDepthPixSizeMap, int midDepthPixSizeMap_p,
                    float2* fusedDepthSimMap, int fusedDepthSimMap_p, int width, int height,
                    int iter, float samplesPerPixSize, int yFrom);

} // namespace depthMap
} // namespace aliceVision
