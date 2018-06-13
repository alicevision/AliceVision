// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/depthMap/cuda/commonStructures.hpp>

namespace aliceVision {
namespace depthMap {

// Helper functions
// function clamping x between a and b
__device__ int clamp(int x, int a, int b)
{
    return max(a, min(b, x));
}


// Global data handlers and parameters
// defines

texture<unsigned char, 2, cudaReadModeNormalizedFloat> rtex;
texture<unsigned char, 2, cudaReadModeNormalizedFloat> ttex;
texture<unsigned char, 2, cudaReadModeNormalizedFloat> gtex;
texture<unsigned char, 2, cudaReadModeNormalizedFloat> btex;

texture<uchar4, 2, cudaReadModeNormalizedFloat> r4tex;
texture<uchar4, 2, cudaReadModeNormalizedFloat> t4tex;
texture<float, 1, cudaReadModeElementType> gaussianTex;

texture<unsigned char, 2, cudaReadModeNormalizedFloat> wshtex;
texture<float, 2, cudaReadModeElementType> watex;

CudaArray<char4, 2>** texs_arr = NULL;
CudaArray<unsigned char, 2>** wshs_arr = NULL;
CudaArray<float, 2>* watex_arr = NULL;

////////////////////////////////////////////////////////////////////////////////
// CONSTANT MEMORY

// cameras matrices

/*
__device__ __constant__ float rP[12];	//12*4 bytes
__device__ __constant__ float riP[9];	//12*4 bytes
__device__ __constant__ float rR[9];	// 9*4 bytes
__device__ __constant__ float riR[9];	// 9*4 bytes
__device__ __constant__ float rK[9];	// 9*4 bytes
__device__ __constant__ float riK[9];	// 9*4 bytes
__device__ __constant__ float3 rC;	// 3*4 bytes
__device__ __constant__ float3 rXVect;	// 3*4 bytes
__device__ __constant__ float3 rYVect;	// 3*4 bytes
__device__ __constant__ float3 rZVect;	// 3*4 bytes

__device__ __constant__ float tP[12];	//12*4 bytes
__device__ __constant__ float tiP[12];	//12*4 bytes
__device__ __constant__ float tR[9];	// 9*4 bytes
__device__ __constant__ float tiR[9];	// 9*4 bytes
__device__ __constant__ float tK[9];	// 9*4 bytes
__device__ __constant__ float tiK[9];	// 9*4 bytes
__device__ __constant__ float3 tC;	// 3*4 bytes
__device__ __constant__ float3 tXVect;	// 3*4 bytes
__device__ __constant__ float3 tYVect;	// 3*4 bytes
__device__ __constant__ float3 tZVect;	// 3*4 bytes
*/

// total bytes (12+9+9+9+3)*4*2 = 168*2 = 336 bytes

__device__ __constant__ float H[9]; // 9*4 bytes

// total bytes (12+9+9+9+3)*4*2 = 168*2 = 336 bytes

// MATLAB: x = [-2:2]; delta = 1; y = exp( - (x .* x) / (2 * delta * delta)); format long g; y
__device__ __constant__ float gauss5[5] = {0.135335283236613f, 0.606530659712633f, 1.0f, 0.606530659712633f,
                                           0.135335283236613f};
__device__ __constant__ float sumGauss55 = 6.16892408102888f;

// MATLAB: distFcnHeight=1.0; maxDist = 0.3;  dist = 0:0.01:1; y =
// 1-distFcnHeight*exp(-(dist.*dist)/(2*maxDist*maxDist)); plot(dist,y);
// MATLAB: distFcnHeight=1.0; maxDist = 0.3;  dist = 0:0.25:1; y =
// 1-distFcnHeight*exp(-(dist.*dist)/(2*maxDist*maxDist)); plot(dist,y); int32(125*y)
__device__ __constant__ unsigned char distFcnConst5[5] = {0, 37, 94, 120, 125};

// MATLAB: distFcnHeight=1.0; maxDist = 0.3;  dist = 0:1/2:1; y =
// 1-distFcnHeight*exp(-(dist.*dist)/(2*maxDist*maxDist)); plot(dist,y); int32(125*y)
__device__ __constant__ unsigned char distFcnConst3[3] = {0, 94, 125};

#define BLOCK_DIM 8

__device__ __constant__ float sg_s_rP[12];  // 12*4 bytes
__device__ __constant__ float sg_s_riP[9];  // 12*4 bytes
__device__ __constant__ float sg_s_rR[9];   // 9*4 bytes
__device__ __constant__ float sg_s_riR[9];  // 9*4 bytes
__device__ __constant__ float sg_s_rK[9];   // 9*4 bytes
__device__ __constant__ float sg_s_riK[9];  // 9*4 bytes
__device__ __constant__ float3 sg_s_rC;     // 3*4 bytes
__device__ __constant__ float3 sg_s_rXVect; // 3*4 bytes
__device__ __constant__ float3 sg_s_rYVect; // 3*4 bytes
__device__ __constant__ float3 sg_s_rZVect; // 3*4 bytes

__device__ __constant__ float sg_s_tP[12];  // 12*4 bytes
__device__ __constant__ float sg_s_tiP[12]; // 12*4 bytes
__device__ __constant__ float sg_s_tR[9];   // 9*4 bytes
__device__ __constant__ float sg_s_tiR[9];  // 9*4 bytes
__device__ __constant__ float sg_s_tK[9];   // 9*4 bytes
__device__ __constant__ float sg_s_tiK[9];  // 9*4 bytes
__device__ __constant__ float3 sg_s_tC;     // 3*4 bytes
__device__ __constant__ float3 sg_s_tXVect; // 3*4 bytes
__device__ __constant__ float3 sg_s_tYVect; // 3*4 bytes
__device__ __constant__ float3 sg_s_tZVect; // 3*4 bytes

/*
__device__ __constant__ struct shared_rCam_tCam
{
        float s_rP[12];	//12*4 bytes
        float s_riP[9];	//12*4 bytes
        float s_rR[9];	// 9*4 bytes
        float s_riR[9];	// 9*4 bytes
        float s_rK[9];	// 9*4 bytes
        float s_riK[9];	// 9*4 bytes
        float3 s_rC;	// 3*4 bytes
        float3 s_rXVect;	// 3*4 bytes
        float3 s_rYVect;	// 3*4 bytes
        float3 s_rZVect;	// 3*4 bytes

        float s_tP[12];	//12*4 bytes
        float s_tiP[12];	//12*4 bytes
        float s_tR[9];	// 9*4 bytes
        float s_tiR[9];	// 9*4 bytes
        float s_tK[9];	// 9*4 bytes
        float s_tiK[9];	// 9*4 bytes
        float3 s_tC;	// 3*4 bytes
        float3 s_tXVect;	// 3*4 bytes
        float3 s_tYVect;	// 3*4 bytes
        float3 s_tZVect;	// 3*4 bytes

        __device__ void copy(shared_rCam_tCam *sg)
        {
                for (int i=0;i<12;i++) {
                        s_rP[i]  = sg->s_rP[i];
                        s_riP[i] = sg->s_riP[i];
                        s_tP[i]  = sg->s_tP[i];
                        s_tiP[i] = sg->s_tiP[i];
                };

                for (int i=0;i<9;i++) {
                        s_rR[i]  = sg->s_rR[i];
                        s_riR[i] = sg->s_riR[i];
                        s_rK[i]  = sg->s_rK[i];
                        s_riK[i] = sg->s_riK[i];
                        s_tR[i]  = sg->s_tR[i];
                        s_tiR[i] = sg->s_tiR[i];
                        s_tK[i]  = sg->s_tK[i];
                        s_tiK[i] = sg->s_tiK[i];
                };

                s_rC     = sg->s_rC;
                s_rXVect = sg->s_rXVect;
                s_rYVect = sg->s_rYVect;
                s_rZVect = sg->s_rZVect;
                s_tC     = sg->s_tC;
                s_tXVect = sg->s_tXVect;
                s_tYVect = sg->s_tYVect;
                s_tZVect = sg->s_tZVect;
        };

        __device__ shared_rCam_tCam& operator = (const shared_rCam_tCam &param)
        {
                for (int i=0;i<12;i++) {
                        s_rP[i]  = param.s_rP[i];
                        s_riP[i] = param.s_riP[i];
                        s_tP[i]  = param.s_tP[i];
                        s_tiP[i] = param.s_tiP[i];
                };

                for (int i=0;i<9;i++) {
                        s_rR[i]  = param.s_rR[i];
                        s_riR[i] = param.s_riR[i];
                        s_rK[i]  = param.s_rK[i];
                        s_riK[i] = param.s_riK[i];
                        s_tR[i]  = param.s_tR[i];
                        s_tiR[i] = param.s_tiR[i];
                        s_tK[i]  = param.s_tK[i];
                        s_tiK[i] = param.s_tiK[i];
                };

                s_rC     = param.s_rC;
                s_rXVect = param.s_rXVect;
                s_rYVect = param.s_rYVect;
                s_rZVect = param.s_rZVect;
                s_tC     = param.s_tC;
                s_tXVect = param.s_tXVect;
                s_tYVect = param.s_tYVect;
                s_tZVect = param.s_tZVect;
                return *this;
        };

};



__device__ __constant__ shared_rCam_tCam sg;
*/

} // namespace depthMap
} // namespace aliceVision
