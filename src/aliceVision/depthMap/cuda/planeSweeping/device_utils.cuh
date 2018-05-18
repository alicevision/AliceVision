// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

namespace aliceVision {
namespace depthMap {

// Helper functions

inline static __host__ clock_t tic()
{
    return clock();
}

// returns the ms passed after last call to tic()
inline static __host__ float toc(clock_t ticClk)
{
    return (float)((clock() - ticClk) * 1000.0 / CLOCKS_PER_SEC);
}

/**
* @brief
* @param[int] ptr
* @param[int] pitch raw length of a line in bytes
* @param[int] x
* @param[int] y
* @return
*/
template <typename T>
inline __device__ T* get2DBufferAt(T* ptr, int pitch, int x, int y)
{

    return ((T*)(((char*)ptr) + y * pitch)) + x;
}

/**
* @brief
* @param[int] ptr
* @param[int] spitch raw length of a 2D array in bytes
* @param[int] pitch raw length of a line in bytes
* @param[int] x
* @param[int] y
* @return
*/
template <typename T>
inline __device__ T* get3DBufferAt(T* ptr, int spitch, int pitch, int x, int y, int z)
{

    return ((T*)(((char*)ptr) + z * spitch + y * pitch)) + x;
}

/*

// function clamping x between a and b

__device__ int clamp(int x, int a, int b){

return fmaxf(a, fminf(b,x));

}

*/

} // namespace depthMap
} // namespace aliceVision
