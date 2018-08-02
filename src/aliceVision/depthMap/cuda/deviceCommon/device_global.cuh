// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "aliceVision/depthMap/cuda/commonStructures.hpp"

namespace aliceVision {
namespace depthMap {

// Helper functions
// function clamping x between a and b
inline static __device__ int clamp(int x, int a, int b)
{
    return max(a, min(b, x));
}

////////////////////////////////////////////////////////////////////////////////
// CONSTANT MEMORY

// cameras matrices

// total bytes (12+9+9+9+3)*4*2 = 168*2 = 336 bytes

extern __device__ __constant__ float H[9]; // 9*4 bytes

// total bytes (12+9+9+9+3)*4*2 = 168*2 = 336 bytes

// MATLAB: x = [-2:2]; delta = 1; y = exp( - (x .* x) / (2 * delta * delta)); format long g; y
extern __device__ __constant__ float gauss5[5];
extern __device__ __constant__ float sumGauss55;

// MATLAB: distFcnHeight=1.0; maxDist = 0.3;  dist = 0:0.01:1; y =
// 1-distFcnHeight*exp(-(dist.*dist)/(2*maxDist*maxDist)); plot(dist,y);
// MATLAB: distFcnHeight=1.0; maxDist = 0.3;  dist = 0:0.25:1; y =
// 1-distFcnHeight*exp(-(dist.*dist)/(2*maxDist*maxDist)); plot(dist,y); int32(125*y)
extern __device__ __constant__ unsigned char distFcnConst5[5];

// MATLAB: distFcnHeight=1.0; maxDist = 0.3;  dist = 0:1/2:1; y =
// 1-distFcnHeight*exp(-(dist.*dist)/(2*maxDist*maxDist)); plot(dist,y); int32(125*y)
extern __device__ __constant__ unsigned char distFcnConst3[3];

#define BLOCK_DIM 8

extern __device__ __constant__ float sg_s_rP[12];  // 12*4 bytes
extern __device__ __constant__ float sg_s_riP[9];  // 12*4 bytes
extern __device__ __constant__ float sg_s_rR[9];   // 9*4 bytes
extern __device__ __constant__ float sg_s_riR[9];  // 9*4 bytes
extern __device__ __constant__ float sg_s_rK[9];   // 9*4 bytes
extern __device__ __constant__ float sg_s_riK[9];  // 9*4 bytes
extern __device__ __constant__ float3 sg_s_rC;     // 3*4 bytes
extern __device__ __constant__ float3 sg_s_rXVect; // 3*4 bytes
extern __device__ __constant__ float3 sg_s_rYVect; // 3*4 bytes
extern __device__ __constant__ float3 sg_s_rZVect; // 3*4 bytes

extern __device__ __constant__ float sg_s_tP[12];  // 12*4 bytes
extern __device__ __constant__ float sg_s_tiP[12]; // 12*4 bytes
extern __device__ __constant__ float sg_s_tR[9];   // 9*4 bytes
extern __device__ __constant__ float sg_s_tiR[9];  // 9*4 bytes
extern __device__ __constant__ float sg_s_tK[9];   // 9*4 bytes
extern __device__ __constant__ float sg_s_tiK[9];  // 9*4 bytes
extern __device__ __constant__ float3 sg_s_tC;     // 3*4 bytes
extern __device__ __constant__ float3 sg_s_tXVect; // 3*4 bytes
extern __device__ __constant__ float3 sg_s_tYVect; // 3*4 bytes
extern __device__ __constant__ float3 sg_s_tZVect; // 3*4 bytes

} // namespace depthMap
} // namespace aliceVision
