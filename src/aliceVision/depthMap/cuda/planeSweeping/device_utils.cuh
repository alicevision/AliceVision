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

template <typename T>
class Plane
{
    inline __device__ __host__
    T* getLine( int y )
    {
        return ((T*)(((char*)_ptr) + y * _pitch));
    }

    inline __device__ __host__
    const T* getLine( int y ) const
    {
        return ((const T*)(((char*)_ptr) + y * _pitch));
    }

public:
    inline __device__ __host__
    Plane( T* ptr, int pitch ) : _ptr(ptr), _pitch(pitch) { }

    inline __device__ __host__
    const T& get( int x, int y ) const
    {
        return getLine(y)[x];
    }

    inline __device__ __host__
    T& getRef( int x, int y )
    {
        return getLine(y)[x];
    }

    inline __device__ __host__
    void set( int x, int y, const T& val )
    {
        getLine(y)[x] = val;
    }

    inline __device__ __host__
    T* getPtr( int x, int y )
    {
        return getLine(y) + x;
    }

private:
    T*  _ptr;
    int _pitch;
};

// /**
// * @brief
// * @param[int] ptr
// * @param[int] pitch raw length of a line in bytes
// * @param[int] x
// * @param[int] y
// * @return
// */
// template <typename T>
// inline __device__ T* get2DBufferAt(T* ptr, int pitch, int x, int y)
// {
//     return Plane<T>(ptr,pitch).getPtr(x,y);
// 
//     // return ((T*)(((char*)ptr) + y * pitch)) + x;
// }

template <typename T>
class Block
{
    inline __device__ __host__
    T* getLine( int y, int z )
    {
        return (T*)(((char*)_ptr) + z * _spitch + y * _pitch);
    }

    inline __device__ __host__
    const T* getLine( int y, int z ) const
    {
        return (const T*)(((const char*)_ptr) + z * _spitch + y * _pitch);
    }

public:
    inline __device__ __host__
    Block( T* ptr, int spitch, int pitch ) : _ptr(ptr), _spitch(spitch), _pitch(pitch) { }

    inline __device__ __host__
    const T& get( int x, int y, int z ) const
    {
        return getLine(y,z)[x];
    }

    inline __device__ __host__
    T& getRef( int x, int y, int z )
    {
        return getLine(y,z)[x];
    }

    inline __device__ __host__
    void set( int x, int y, int z, const T& val )
    {
        getLine(y,z)[x] = val;
    }

    inline __device__ __host__
    T* getPtr( int x, int y, int z )
    {
        return getLine(y,z) + x;
    }

private:
    T*  _ptr;
    int _spitch;
    int _pitch;
};
/**
* @brief
* @param[int] ptr
* @param[int] spitch raw length of a 2D array in bytes
* @param[int] pitch raw length of a line in bytes
* @param[int] x
* @param[int] y
* @return
*/
// template <typename T>
// inline __device__ T* get3DBufferAt(T* ptr, int spitch, int pitch, int x, int y, int z)
// {
//     // return ((T*)(((char*)ptr) + z * spitch + y * pitch)) + x;
//     return Block<T>(ptr,spitch,pitch).getPtr(x,y,z);
// }

/*

// function clamping x between a and b

__device__ int clamp(int x, int a, int b){

return fmaxf(a, fminf(b,x));

}

*/

} // namespace depthMap
} // namespace aliceVision
