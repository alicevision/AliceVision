// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

// allows code sharing between NVCC and other compilers
#if defined(__NVCC__)
    #define CUDA_HOST_DEVICE __host__ __device__
    #define CUDA_HOST __host__
#else
    #define CUDA_HOST_DEVICE
    #define CUDA_HOST
#endif

namespace aliceVision {
namespace depthMap {

template<typename T>
class BufPtr
{
  public:
    CUDA_HOST_DEVICE BufPtr(T* ptr, size_t pitch)
      : _ptr((unsigned char*)ptr),
        _pitch(pitch)
    {}

    CUDA_HOST_DEVICE inline T* ptr() { return (T*)(_ptr); }
    CUDA_HOST_DEVICE inline T* row(size_t y) { return (T*)(_ptr + y * _pitch); }
    CUDA_HOST_DEVICE inline T& at(size_t x, size_t y) { return row(y)[x]; }

    CUDA_HOST_DEVICE inline const T* ptr() const { return (const T*)(_ptr); }
    CUDA_HOST_DEVICE inline const T* row(size_t y) const { return (const T*)(_ptr + y * _pitch); }
    CUDA_HOST_DEVICE inline const T& at(size_t x, size_t y) const { return row(y)[x]; }

  private:
    BufPtr();
    BufPtr(const BufPtr&);
    BufPtr& operator*=(const BufPtr&);

    unsigned char* const _ptr;
    const size_t _pitch;
};

template<typename T>
static inline T* get3DBufferAt_h(T* ptr, size_t spitch, size_t pitch, size_t x, size_t y, size_t z)
{
    return ((T*)(((char*)ptr) + z * spitch + y * pitch)) + x;
}

template<typename T>
static inline const T* get3DBufferAt_h(const T* ptr, size_t spitch, size_t pitch, size_t x, size_t y, size_t z)
{
    return ((const T*)(((const char*)ptr) + z * spitch + y * pitch)) + x;
}

}  // namespace depthMap
}  // namespace aliceVision
