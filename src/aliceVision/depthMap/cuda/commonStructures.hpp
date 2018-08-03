// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <cuda_runtime.h>
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <stdexcept>
#include <memory>

namespace aliceVision {
namespace depthMap {

static inline void memOpErrorCheck( cudaError_t err, const char* file, int line, const char* info )
{
    if( err == cudaSuccess ) return;

    printf( "%s in %s:%d, reason %s\n", info, file, line, cudaGetErrorString(err) );
    throw std::runtime_error( info );
}

template <unsigned Dim> class CudaSizeBase
{
public:
  CudaSizeBase()
  {
    for(int i = Dim; i--;)
      size[i] = 0;
  }
  inline size_t dim() const { return Dim; }
  inline size_t operator[](size_t i) const { return size[i]; }
  inline size_t &operator[](size_t i) { return size[i]; }
  inline CudaSizeBase operator+(const CudaSizeBase<Dim> &s) const {
    CudaSizeBase<Dim> r;

    for(size_t i = Dim; i--;)
      r[i] = (*this)[i] + s[i];

    return r;
  }
  inline CudaSizeBase operator-(const CudaSizeBase<Dim> &s) const {
    CudaSizeBase<Dim> r;

    for(size_t i = Dim; i--;)
      r[i] = (*this)[i] - s[i];

    return r;
  }
  size_t getSize() const {
    size_t s = 1;

    for(int i = Dim; i--;)
      s *= size[i];

    return s;
  }
protected:
  size_t size[Dim];
};

template <unsigned Dim>
class CudaSize: public CudaSizeBase<Dim>
{
  CudaSize() {}
};

template <>
class CudaSize<1>: public CudaSizeBase<1>
{
public:
  CudaSize() {}
  explicit CudaSize(size_t s0) { size[0] = s0; }
};

template <>
class CudaSize<2>: public CudaSizeBase<2>
{
public:
  CudaSize() {}
  CudaSize(size_t s0, size_t s1) { size[0] = s0; size[1] = s1; }
};

template <>
class CudaSize<3>: public CudaSizeBase<3>
{
public:
  CudaSize() {}
  CudaSize(size_t s0, size_t s1, size_t s2) { size[0] = s0; size[1] = s1; size[2] = s2; }
};

template <unsigned Dim>
bool operator==(const CudaSizeBase<Dim> &s1, const CudaSizeBase<Dim> &s2)
{
  for(int i = Dim; i--;)
    if(s1[i] != s2[i])
      return false;

  return true;
}

template <unsigned Dim>
bool operator!=(const CudaSizeBase<Dim> &s1, const CudaSizeBase<Dim> &s2)
{
  for(size_t i = Dim; i--;)
    if(s1[i] != s2[i])
      return true;

  return false;
}

template <unsigned Dim>
CudaSize<Dim> operator/(const CudaSize<Dim> &lhs, const float &rhs) {
  if (rhs == 0)
    fprintf(stderr, "Division by zero!!\n");
  CudaSize<Dim> out = lhs;
  for(size_t i = 0; i < Dim; ++i)
    out[i] /= rhs;

  return out;
}

template <unsigned Dim>
CudaSize<Dim> operator-(const CudaSize<Dim> &lhs, const CudaSize<Dim> &rhs) {
  CudaSize<Dim> out = lhs;
  for(size_t i = Dim; i--;)
    out[i]-= rhs[i];
  return out;
}

/*********************************************************************************
 * declarations
 *********************************************************************************/

template <class Type, unsigned Dim> class CudaHostMemoryHeap;
template <class Type, unsigned Dim> class CudaDeviceMemoryPitched;
template <class Type, unsigned Dim> class CudaArray;

/*********************************************************************************
 * CudaHostMemoryHeap
 * unspecialized template
 *********************************************************************************/

template <class Type, unsigned Dim> class CudaHostMemoryHeap
{
  Type* buffer;
  size_t sx, sy, sz;
  CudaSize<Dim> size;
public:
  explicit CudaHostMemoryHeap(const CudaSize<Dim> &_size)
  {
    size = _size;
    sx = 1;
    sy = 1;
    sz = 1;
    if (Dim >= 1) sx = _size[0];
    if (Dim >= 2) sy = _size[1];
    if (Dim >= 3) sz = _size[2];
    buffer = (Type*)malloc(sx * sy * sz * sizeof (Type));
    if( buffer == 0 ) {
        printf("%d malloc failed\n", __LINE__ );
        throw std::runtime_error("Malloc mem allocation error");
    }
    memset(buffer, 0, sx * sy * sz * sizeof (Type));
  }
  CudaHostMemoryHeap<Type,Dim>& operator=(const CudaHostMemoryHeap<Type,Dim>& rhs)
  {
    size = rhs.size;
    sx = 1;
    sy = 1;
    sz = 1;
    if (Dim >= 1) sx = rhs.sx;
    if (Dim >= 2) sy = rhs.sy;
    if (Dim >= 3) sz = rhs.sz;
    buffer = (Type*)malloc(sx * sy * sz * sizeof (Type));
    if( buffer == 0 ) {
        printf("%d malloc failed\n", __LINE__ );
        throw std::runtime_error("Malloc mem allocation error");
    }
    memcpy(buffer, rhs.buffer, sx * sy * sz * sizeof (Type));
    return *this;
  }
  ~CudaHostMemoryHeap()
  {
    free(buffer);
  }
  const CudaSize<Dim>& getSize() const
  {
    return size;
  }
  size_t getBytes() const
  {
    return sx * sy * sz * sizeof (Type);
  }
  Type *getBuffer()
  {
    return buffer;
  }
  const Type *getBuffer() const
  {
    return buffer;
  }
  Type& operator()(size_t x, size_t y)
  {
    return buffer[y * sx + x];
  }

  void copyFrom(const CudaDeviceMemoryPitched<Type, Dim>& _src)
  {
    cudaError_t err;
    cudaMemcpyKind kind = cudaMemcpyDeviceToHost;
    if(Dim == 1) {
      err = cudaMemcpy(this->getBuffer(), _src.getBuffer(), _src.getBytes(), kind);
      memOpErrorCheck( err, __FILE__, __LINE__, "Failed to copy from CUDA flat mem to host flat mem" );
    }
    else if(Dim == 2) {
      err = cudaMemcpy2D(this->getBuffer(), size[0] * sizeof (Type), _src.getBuffer(), _src.getPitch(), size[0] * sizeof (Type), size[1], kind);
      memOpErrorCheck( err, __FILE__, __LINE__, "Failed to copy from CUDA 2D mem to host flat mem" );
    }
    else if(Dim >= 3) {
      for (unsigned int slice=0; slice<size[2]; slice++)
      {
        err = cudaMemcpy2D( this->getBuffer() + slice * size[0] * size[1],
                            size[0] * sizeof (Type),
                            &_src.getBuffer()[slice * _src.stride()[1]],
                            _src.getPitch(),
                            size[0] * sizeof (Type), size[1], kind);
        memOpErrorCheck( err, __FILE__, __LINE__, "Failed to copy from sliced CUDA 3D mem to host flat mem" );
      }
    }
  }

  void copyFrom(const CudaArray<Type, Dim>& _src)
  {
    cudaError_t err;

    cudaMemcpyKind kind = cudaMemcpyDeviceToHost;
    if(Dim == 1) {
        err = cudaMemcpyFromArray(this->getBuffer(), _src.getArray(), 0, 0, this->getSize()[0] * sizeof (Type), kind);
        memOpErrorCheck( err, __FILE__, __LINE__, "Failed to copy from CUDA 1D array to host flat mem" );
    }
    else if(Dim == 2) {
        err = cudaMemcpy2DFromArray(this->getBuffer(), size[0] * sizeof (Type), _src.getArray(), 0, 0, size[0] * sizeof (Type), size[1], kind);
        memOpErrorCheck( err, __FILE__, __LINE__, "Failed to copy from CUDA 2D array to host flat mem" );
    }
    else if(Dim == 3) {
        cudaMemcpy3DParms p = { 0 };
        p.srcArray = const_cast<cudaArray *>(_src.getArray());
        p.srcPtr.pitch = _src.getPitch();
        p.srcPtr.xsize = _src.getSize()[0];
        p.srcPtr.ysize = _src.getSize()[1];
        p.dstPtr.ptr = (void *)this->getBuffer();
        p.dstPtr.pitch = size[0] * sizeof (Type);
        p.dstPtr.xsize = size[0];
        p.dstPtr.ysize = size[1];
        p.extent.width = size[0];
        p.extent.height = size[1];
        p.extent.depth = size[2];
        for(unsigned i = 3; i < Dim; ++i)
            p.extent.depth *= _src.getSize()[i];
        p.kind = kind;
        err = cudaMemcpy3D(&p);
        memOpErrorCheck( err, __FILE__, __LINE__, "Failed to copy from CUDA 3D array to host flat mem" );
    }
  }
};

/*********************************************************************************
 * CudaHostMemoryHeap
 * Dim=1
 *********************************************************************************/

template <class Type> class CudaHostMemoryHeap<Type,1>
{
  Type* buffer;
  size_t sx, sy, sz;
  CudaSize<1> size;
public:
  explicit CudaHostMemoryHeap(const CudaSize<1> &_size)
  {
    size = _size;
    sx = _size[0];
    sy = 1;
    sz = 1;
    buffer = (Type*)malloc(sx * sy * sz * sizeof (Type));
    if( buffer == 0 ) {
        printf("%d malloc failed\n", __LINE__ );
        throw std::runtime_error("Malloc mem allocation error");
    }
    memset(buffer, 0, sx * sy * sz * sizeof (Type));
  }
  CudaHostMemoryHeap<Type,1>& operator=(const CudaHostMemoryHeap<Type,1>& rhs)
  {
    size = rhs.size;
    sx = rhs.sx;
    sy = 1;
    sz = 1;
    buffer = (Type*)malloc(sx * sy * sz * sizeof (Type));
    if( buffer == 0 ) {
        printf("%d malloc failed\n", __LINE__ );
        throw std::runtime_error("Malloc mem allocation error");
    }
    memcpy(buffer, rhs.buffer, sx * sy * sz * sizeof (Type));
    return *this;
  }
  ~CudaHostMemoryHeap()
  {
    free(buffer);
  }
  const CudaSize<1>& getSize() const
  {
    return size;
  }
  size_t getBytes() const
  {
    return sx * sy * sz * sizeof (Type);
  }
  Type *getBuffer()
  {
    return buffer;
  }
  const Type *getBuffer() const
  {
    return buffer;
  }
  Type& operator()(size_t x, size_t y)
  {
    return buffer[y * sx + x];
  }

  void copyFrom(const CudaDeviceMemoryPitched<Type, 1>& _src)
  {
      cudaError_t err;

      const cudaMemcpyKind kind = cudaMemcpyDeviceToHost;
      err = cudaMemcpy(this->getBuffer(), _src.getBuffer(), _src.getBytes(), kind);
      memOpErrorCheck( err, __FILE__, __LINE__, "Failed to copy from CUDA flat mem to host flat mem" );
  }

  void copyFrom(const CudaArray<Type, 1>& _src)
  {
      cudaError_t err;

      const cudaMemcpyKind kind = cudaMemcpyDeviceToHost;
      err = cudaMemcpyFromArray(this->getBuffer(), _src.getArray(), 0, 0, this->getSize()[0] * sizeof (Type), kind);
      memOpErrorCheck( err, __FILE__, __LINE__, "Failed to copy from CUDA 1D array to host flat mem" );
  }
};

/*********************************************************************************
 * CudaDeviceMemoryPitched
 *********************************************************************************/

template <class Type, unsigned Dim> class CudaDeviceMemoryPitched
{
  Type* buffer;
  size_t pitch;
  size_t sx, sy, sz;
  CudaSize<Dim> size;
public:
  explicit CudaDeviceMemoryPitched(const CudaSize<Dim> &_size)
  {
    cudaError_t err;

    size = _size;
    sx = 1;
    sy = 1;
    sz = 1;
    if (Dim >= 1) sx = _size[0];
    if (Dim >= 2) sy = _size[1];
    if (Dim >= 3) sx = _size[2];
    if(Dim == 2)
    {
      err = cudaMallocPitch(&buffer, &pitch, _size[0] * sizeof(Type), _size[1]);
      memOpErrorCheck( err, __FILE__, __LINE__, "Failed to alloc CUDA pitched mem" );
    }
    else if(Dim >= 3)
    {
      cudaExtent extent;
      extent.width = _size[0] * sizeof(Type);
      extent.height = _size[1];
      extent.depth = _size[2];
      for(unsigned i = 3; i < Dim; ++i)
        extent.depth *= _size[i];
      cudaPitchedPtr pitchDevPtr;
      err = cudaMalloc3D(&pitchDevPtr, extent);
      memOpErrorCheck( err, __FILE__, __LINE__, "Failed to alloc CUDA 3D mem" );
      buffer = (Type*)pitchDevPtr.ptr;
      pitch = pitchDevPtr.pitch;
    }
  }
  ~CudaDeviceMemoryPitched()
  {
      cudaError_t err;
      err = cudaFree(buffer);
      memOpErrorCheck( err, __FILE__, __LINE__, "Failed to free CUDA mem" );
  }
  explicit inline CudaDeviceMemoryPitched(const CudaHostMemoryHeap<Type, Dim> &rhs)
  {
    cudaError_t err;

    size = rhs.getSize();
    sx = 1;
    sy = 1;
    sz = 1;
    if (Dim >= 1) sx = rhs.getSize()[0];
    if (Dim >= 2) sy = rhs.getSize()[1];
    if (Dim >= 3) sx = rhs.getSize()[2];
    if(Dim == 2)
    {
      err = cudaMallocPitch(&buffer, &pitch, size[0] * sizeof(Type), size[1]);
      memOpErrorCheck( err, __FILE__, __LINE__, "Failed to alloc CUDA pitched mem" );
    }
    if(Dim >= 3)
    {
      cudaExtent extent;
      extent.width = size[0] * sizeof(Type);
      extent.height = size[1];
      extent.depth = size[2];
      for(unsigned i = 3; i < Dim; ++i)
        extent.depth *= size[i];
      cudaPitchedPtr pitchDevPtr;
      err = cudaMalloc3D(&pitchDevPtr, extent);
      memOpErrorCheck( err, __FILE__, __LINE__, "Failed to alloc CUDA 3D mem" );
      buffer = (Type*)pitchDevPtr.ptr;
      pitch = pitchDevPtr.pitch;
    }
    copyFrom( rhs);
  }
  CudaDeviceMemoryPitched<Type,Dim> & operator=(const CudaDeviceMemoryPitched<Type,Dim> & rhs)
  {
    copyFrom( rhs);
    return *this;
  }
  const CudaSize<Dim>& getSize() const
  {
    return size;
  }
  size_t getBytes() const
  {
    size_t s;
    s = pitch;
    for(unsigned i = 1; i < Dim; ++i)
      s *= size[i];
    return s;
  }
  size_t getPitch() const
  {
    return pitch;
  }
  Type *getBuffer()
  {
    return buffer;
  }
  const Type *getBuffer() const
  {
    return buffer;
  }

  const CudaSize<Dim> stride() const
  {
    CudaSize<Dim> s;
    s[0] = pitch;
    for(unsigned i = 1; i < Dim; ++i)
      s[i] = s[i - 1] * size[i];
    return s;
  }

  void copyFrom(const CudaHostMemoryHeap<Type, Dim>& _src)
  {
    cudaError_t err;

    cudaMemcpyKind kind = cudaMemcpyHostToDevice;
    if(Dim == 1) {
      err = cudaMemcpy(this->getBuffer(), _src.getBuffer(), _src.getBytes(), kind);
      memOpErrorCheck( err, __FILE__, __LINE__, "Failed to copy host mem to CUDA flat mem" );
    }
    else if(Dim == 2)
    {
      err = cudaMemcpy2D( this->getBuffer(), this->getPitch(), _src.getBuffer(),
                          _src.getSize()[0] * sizeof (Type), _src.getSize()[0] * sizeof (Type),
                          _src.getSize()[1], kind);
      memOpErrorCheck( err, __FILE__, __LINE__, "Failed to copy host mem to CUDA pitched mem" );
    }
    else if(Dim >= 3)
    {
      for (unsigned int slice=0; slice<_src.getSize()[2]; slice++)
      {
        err = cudaMemcpy2D( &this->getBuffer()[slice * this->stride()[1]],
                            this->getPitch(),
                            _src.getBuffer() + slice * _src.getSize()[0] * _src.getSize()[1],
                            _src.getSize()[0] * sizeof (Type),
                            _src.getSize()[0] * sizeof (Type),
                            _src.getSize()[1], kind);
        memOpErrorCheck( err, __FILE__, __LINE__, "Failed to copy host mem to sliced CUDA 3D mem" );
      }
    }
  }

  void copyFrom(const CudaDeviceMemoryPitched<Type, Dim>& _src)
  {
    cudaError_t err;

    cudaMemcpyKind kind = cudaMemcpyDeviceToDevice;
    if(Dim == 1)
    {
      err = cudaMemcpy(this->getBuffer(), _src.getBuffer(), _src.getBytes(), kind);
      memOpErrorCheck( err, __FILE__, __LINE__, "Failed to copy CUDA pitched mem to CUDA flat mem" );
    }
    else if(Dim == 2)
    {
      err = cudaMemcpy2D( this->getBuffer(), this->getPitch(),
                          _src.getBuffer(), _src.getPitch(),
                          _src.getSize()[0] * sizeof(Type),
                          _src.getSize()[1], kind);
      memOpErrorCheck( err, __FILE__, __LINE__, "Failed to copy CUDA pitched mem to CUDA pitched mem" );
    }
    else if(Dim >= 3)
    {
      for (unsigned int slice=0; slice<_src.getSize()[2]; slice++)
      {
        err = cudaMemcpy2D( &this->getBuffer()[slice * this->stride()[1]],
                            this->getPitch(),
                            &_src.getBuffer()[slice * _src.stride()[1]], _src.getPitch(),
                            _src.getSize()[0] * sizeof(Type),
                            _src.getSize()[1], kind);
        memOpErrorCheck( err, __FILE__, __LINE__, "Failed to copy CUDA pitched mem to sliced CUDA 3D mem" );
      }
    }
  }

  void copyFrom(const CudaArray<Type, Dim>& _src)
  {
    cudaError_t err;

    cudaMemcpyKind kind = cudaMemcpyDeviceToDevice;
    if(Dim == 1) {
      err = cudaMemcpyFromArray(this->getBuffer(), _src.getArray(), 0, 0, _src.getSize()[0] * sizeof(Type), kind);
      memOpErrorCheck( err, __FILE__, __LINE__, "Failed to copy CUDA array to CUDA 1D mem" );
    }
    else if(Dim == 2) {
      err = cudaMemcpy2DFromArray(this->getBuffer(), this->getPitch(), _src.getArray(), 0, 0, _src.getSize()[0] * sizeof(Type), _src.getSize()[1], kind);
      memOpErrorCheck( err, __FILE__, __LINE__, "Failed to copy CUDA array to CUDA 2D mem" );
    }
    else if(Dim == 3) {
      cudaMemcpy3DParms p = { 0 };
      p.srcArray = const_cast<cudaArray *>(_src.getArray());
      p.srcPtr.pitch = _src.getPitch();
      p.srcPtr.xsize = _src.getSize()[0];
      p.srcPtr.ysize = _src.getSize()[1];
      p.dstPtr.ptr = (void *)this->getBuffer();
      p.dstPtr.pitch = this->getPitch();
      p.dstPtr.xsize = size[0];
      p.dstPtr.ysize = size[1];
      p.extent.width = _src.getSize()[0];
      p.extent.height = _src.getSize()[1];
      p.extent.depth = _src.getSize()[2];
      for(unsigned i = 3; i < Dim; ++i)
        p.extent.depth *= _src.getSize()[i];
      p.kind = kind;
      err = cudaMemcpy3D(&p);
      memOpErrorCheck( err, __FILE__, __LINE__, "Failed to copy CUDA array to CUDA 3D mem" );
    }
  }
};

/*********************************************************************************
 * CudaArray
 *********************************************************************************/

template <class Type, unsigned Dim> class CudaArray
{
  cudaArray *array;
  size_t sx, sy, sz;
  CudaSize<Dim> size;
public:
  explicit CudaArray(const CudaSize<Dim> &_size)
  {
    cudaError_t err;

    size = _size;
    sx = 1;
    sy = 1;
    sz = 1;
    if (Dim >= 1) sx = _size[0];
    if (Dim >= 2) sy = _size[1];
    if (Dim >= 3) sz = _size[2];
    cudaChannelFormatDesc channelDesc = cudaCreateChannelDesc<Type>();
    if(Dim == 1)
    {
      err = cudaMallocArray(&array, &channelDesc, _size[0], 1, cudaArraySurfaceLoadStore);
      memOpErrorCheck( err, __FILE__, __LINE__, "Failed to allocate CUDA 1D array" );
    }
    else if(Dim == 2)
    {
      err = cudaMallocArray(&array, &channelDesc, _size[0], _size[1], cudaArraySurfaceLoadStore);
      memOpErrorCheck( err, __FILE__, __LINE__, "Failed to allocate CUDA 2D array" );
    }
    else
    {
      cudaExtent extent;
      extent.width = _size[0];
      extent.height = _size[1];
      extent.depth = _size[2];
      for(unsigned i = 3; i < Dim; ++i)
        extent.depth *= _size[i];
      err = cudaMalloc3DArray(&array, &channelDesc, extent);
      memOpErrorCheck( err, __FILE__, __LINE__, "Failed to allocate CUDA 3D array" );
    }
  }
  explicit inline CudaArray(const CudaDeviceMemoryPitched<Type, Dim> &rhs)
  {
    cudaError_t err;

    size = rhs.getSize();
    sx = 1;
    sy = 1;
    sz = 1;
    if (Dim >= 1) sx = rhs.getSize()[0];
    if (Dim >= 2) sy = rhs.getSize()[1];
    if (Dim >= 3) sz = rhs.getSize()[2];
    cudaChannelFormatDesc channelDesc = cudaCreateChannelDesc<Type>();
    if(Dim == 1)
    {
      err = cudaMallocArray(&array, &channelDesc, size[0], 1, cudaArraySurfaceLoadStore);
      memOpErrorCheck( err, __FILE__, __LINE__, "Failed to allocate CUDA 1D array" );
    }
    else if(Dim == 2)
    {
      err = cudaMallocArray(&array, &channelDesc, size[0], size[1], cudaArraySurfaceLoadStore);
      memOpErrorCheck( err, __FILE__, __LINE__, "Failed to allocate CUDA 2D array" );
    }
    else
    {
      cudaExtent extent;
      extent.width = size[0];
      extent.height = size[1];
      extent.depth = size[2];
      for(unsigned i = 3; i < Dim; ++i)
        extent.depth *= size[i];
      err = cudaMalloc3DArray(&array, &channelDesc, extent);
      memOpErrorCheck( err, __FILE__, __LINE__, "Failed to allocate CUDA 3D array" );
    }
    copyFrom( rhs );
  }
  explicit inline CudaArray(const CudaHostMemoryHeap<Type, Dim> &rhs)
  {
    cudaError_t err;

    size = rhs.getSize();
    sx = 1;
    sy = 1;
    sz = 1;
    if (Dim >= 1) sx = rhs.getSize()[0];
    if (Dim >= 2) sy = rhs.getSize()[1];
    if (Dim >= 3) sz = rhs.getSize()[2];
    cudaChannelFormatDesc channelDesc = cudaCreateChannelDesc<Type>();
    if(Dim == 1)
    {
      err = cudaMallocArray(&array, &channelDesc, size[0], 1, cudaArraySurfaceLoadStore);
      memOpErrorCheck( err, __FILE__, __LINE__, "Failed to allocate CUDA 1D array" );
    }
    else if(Dim == 2)
    {
      err = cudaMallocArray(&array, &channelDesc, size[0], size[1], cudaArraySurfaceLoadStore);
      memOpErrorCheck( err, __FILE__, __LINE__, "Failed to allocate CUDA 2D array" );
    }
    else
    {
      cudaExtent extent;
      extent.width = size[0];
      extent.height = size[1];
      extent.depth = size[2];
      for(unsigned i = 3; i < Dim; ++i)
        extent.depth *= size[i];
      err = cudaMalloc3DArray(&array, &channelDesc, extent);
      memOpErrorCheck( err, __FILE__, __LINE__, "Failed to allocate CUDA 3D array" );
    }
    copyFrom( rhs );
  }
  virtual ~CudaArray()
  {
      cudaError_t err;
      err = cudaFreeArray(array);
      memOpErrorCheck( err, __FILE__, __LINE__, "Failed to free CUDA array" );
  }
  size_t getBytes() const
  {
    size_t s;
    s = 1;
    for(unsigned i = 0; i < Dim; ++i)
      s *= size[i];
    return s;
  }
  const CudaSize<Dim>& getSize() const
  {
    return size;
  }
  size_t getPitch() const
  {
    return size[0] * sizeof (Type);
  }
  cudaArray *getArray()
  {
    return array;
  }
  const cudaArray *getArray() const
  {
    return array;
  }

  void copyFrom(const CudaHostMemoryHeap<Type, Dim>& _src)
  {
    cudaError_t err;

    cudaMemcpyKind kind = cudaMemcpyHostToDevice;
    if(Dim == 1) {
      err = cudaMemcpyToArray(this->getArray(), 0, 0, _src.getBuffer(), _src.getSize()[0] * sizeof (Type), kind);
      memOpErrorCheck( err, __FILE__, __LINE__, "Failed to copy host mem to CUDA 1D array" );
    }
    else if(Dim == 2) {
      err = cudaMemcpy2DToArray(this->getArray(), 0, 0, _src.getBuffer(), _src.getSize()[0] * sizeof (Type), _src.getSize()[0] * sizeof (Type), _src.getSize()[1], kind);
      memOpErrorCheck( err, __FILE__, __LINE__, "Failed to copy host mem to CUDA 2D array" );
    }
    else if(Dim == 3) {
      cudaMemcpy3DParms p = { 0 };
      p.srcPtr.ptr = (void *)_src.getBuffer();
      p.srcPtr.pitch = _src.getSize()[0] * sizeof (Type);
      p.srcPtr.xsize = _src.getSize()[0];
      p.srcPtr.ysize = _src.getSize()[1];
      p.dstArray = this->getArray();
      p.dstPtr.pitch = this->getPitch();
      p.dstPtr.xsize = this->getSize()[0];
      p.dstPtr.ysize = this->getSize()[1];
      p.extent.width = _src.getSize()[0];
      p.extent.height = _src.getSize()[1];
      p.extent.depth = _src.getSize()[2];
      for(unsigned i = 3; i < Dim; ++i)
        p.extent.depth *= _src.getSize()[i];
      p.kind = kind;
      err = cudaMemcpy3D(&p);
      memOpErrorCheck( err, __FILE__, __LINE__, "Failed to copy host mem to CUDA 3D array" );
    }
  }

  void copyFrom(const CudaDeviceMemoryPitched<Type, Dim>& _src)
  {
    cudaError_t err;

    cudaMemcpyKind kind = cudaMemcpyDeviceToDevice;
    if(Dim == 1) {
      err = cudaMemcpyToArray(this->getArray(), 0, 0, _src.getBuffer(), _src.getSize()[0] * sizeof(Type), kind);
      memOpErrorCheck( err, __FILE__, __LINE__, "Failed to copy CUDA pitched mem to CUDA 1D array" );
    }
    else if(Dim == 2) {
      err = cudaMemcpy2DToArray(this->getArray(), 0, 0, _src.getBuffer(), _src.getPitch(), _src.getSize()[0] * sizeof(Type), _src.getSize()[1], kind);
      memOpErrorCheck( err, __FILE__, __LINE__, "Failed to copy CUDA pitched mem to CUDA 2D array" );
    }
    else if(Dim == 3) {
      cudaMemcpy3DParms p = { 0 };
      p.srcPtr.ptr = (void *)_src.getBuffer();
      p.srcPtr.pitch = _src.getPitch();
      p.srcPtr.xsize = _src.getSize()[0];
      p.srcPtr.ysize = _src.getSize()[1];
      p.dstArray = this->getArray();
      p.dstPtr.pitch = this->getPitch();
      p.dstPtr.xsize = this->getSize()[0];
      p.dstPtr.ysize = this->getSize()[1];
      p.extent.width = _src.getSize()[0];
      p.extent.height = _src.getSize()[1];
      p.extent.depth = _src.getSize()[2];
      for(unsigned i = 3; i < Dim; ++i)
        p.extent.depth *= _src.getSize()[i];
      p.kind = kind;
      err = cudaMemcpy3D(&p);
      memOpErrorCheck( err, __FILE__, __LINE__, "Failed to copy CUDA pitched mem to CUDA 3D array" );
    }
  }
};

/*********************************************************************************
 * support functions
 *********************************************************************************/

#if 0
template<class Type, unsigned Dim> void copy(CudaHostMemoryHeap<Type, Dim>& _dst, const CudaDeviceMemoryPitched<Type, Dim>& _src)
{
  _dst.copyFrom( _src );
}

template<class Type, unsigned Dim> void copy(CudaHostMemoryHeap<Type, Dim>& _dst, const CudaArray<Type, Dim>& _src)
{
  _dst.copyFrom( _src );
}

template<class Type, unsigned Dim> void copy(CudaDeviceMemoryPitched<Type, Dim>& _dst, const CudaHostMemoryHeap<Type, Dim>& _src)
{
  _dst.copyFrom( _src );
}

template<class Type, unsigned Dim> void copy(CudaDeviceMemoryPitched<Type, Dim>& _dst, const CudaDeviceMemoryPitched<Type, Dim>& _src)
{
  _dst.copyFrom( _src );
}

template<class Type, unsigned Dim> void copy(CudaDeviceMemoryPitched<Type, Dim>& _dst, const CudaArray<Type, Dim>& _src)
{
  _dst.copyFrom( _src );
}

template<class Type, unsigned Dim> void copy(CudaArray<Type, Dim>& _dst, const CudaHostMemoryHeap<Type, Dim>& _src)
{
  _dst.copyFrom( _src );
}

template<class Type, unsigned Dim> void copy(CudaArray<Type, Dim>& _dst, const CudaDeviceMemoryPitched<Type, Dim>& _src)
{
  _dst.copyFrom( _src );
}
#endif

template<class Type, unsigned Dim> void copy(Type* _dst, size_t sx, size_t sy, const CudaDeviceMemoryPitched<Type, Dim>& _src)
{
  cudaError_t err;
  if(Dim == 2)
  {
    err = cudaMemcpy2D(_dst, sx * sizeof (Type), _src.getBuffer(), _src.getPitch(), sx * sizeof (Type), sy, cudaMemcpyDeviceToHost);
    memOpErrorCheck( err, __FILE__, __LINE__, "Failed to copy from CUDA 2D mem to host flat mem" );
  }
  else
  {
    throw std::runtime_error( "Called copy function with wrong dimensionality" );
  }
}

template<class Type, unsigned Dim> void copy(CudaDeviceMemoryPitched<Type, Dim>& _dst, const Type* _src, size_t sx, size_t sy)
{
  cudaError_t err;
  if(Dim == 2) {
    err = cudaMemcpy2D(_dst.getBuffer(), _dst.getPitch(), _src, sx * sizeof (Type), sx * sizeof(Type), sy, cudaMemcpyHostToDevice);
    memOpErrorCheck( err, __FILE__, __LINE__, "Failed to copy from host flat mem to CUDA 2D mem" );
  }
  else
  {
    throw std::runtime_error( "Called copy function with wrong dimensionality" );
  }
}

template<class Type, unsigned Dim> void copy(Type* _dst, size_t sx, size_t sy, size_t sz, const CudaDeviceMemoryPitched<Type, Dim>& _src)
{
    cudaError_t err;
    if(Dim >= 3) {
        for (unsigned int slice=0; slice<sz; slice++)
        {
            err = cudaMemcpy2D( _dst + sx * sy * slice, sx * sizeof (Type), &_src.getBuffer()[slice * _src.stride()[1]], _src.getPitch(), sx * sizeof (Type), sy, cudaMemcpyDeviceToHost);
            memOpErrorCheck( err, __FILE__, __LINE__, "Failed to copy from sliced CUDA 3D mem to host flat mem" );
        }
    }
    else
    {
      throw std::runtime_error( "Called copy function with wrong dimensionality" );
    }
}

template<class Type, unsigned Dim> void copy(CudaDeviceMemoryPitched<Type, Dim>& _dst, const Type* _src, size_t sx, size_t sy, size_t sz)
{
  cudaError_t err;
  if(Dim >= 3) {
    for (unsigned int slice=0; slice<sz; slice++)
    {
      err = cudaMemcpy2D( &_dst.getBuffer()[slice * _dst.stride()[1]], _dst.getPitch(), _src + sx * sy * slice, sx * sizeof (Type), sx * sizeof(Type), sy, cudaMemcpyHostToDevice);
      memOpErrorCheck( err, __FILE__, __LINE__, "Failed to copy from host flat mem to sliced CUDA 3D mem" );
    }
  }
}

struct CameraBaseStruct; // forward reference, include only for destructor

class cameraStruct
{
private:
    std::shared_ptr<CudaHostMemoryHeap<uchar4, 2> > tex_rgba_hmh;
public:
    std::shared_ptr<CameraBaseStruct> cam;
    int camId;
    int rc;
    int scale;

    cameraStruct();
    cameraStruct( const cameraStruct& );

    cameraStruct& operator=( const cameraStruct& orig );

    ~cameraStruct();

    inline bool haveTexRGBA() const
    {
        return (bool)tex_rgba_hmh;
    }

    inline const CudaHostMemoryHeap<uchar4, 2>& getRefTexRGBA( ) const
    {
        return *( tex_rgba_hmh.get() );
    }

    void makeTexRGBA( int w, int h );
    void setTexRGBA( int x, int y, const uchar4& val );
};

struct ps_parameters
{
    int epipShift;
    int rotX;
    int rotY;
};

#define MAX_PTS 500           // 500
#define MAX_PATCH_PIXELS 2500 // 50*50

} // namespace depthMap
} // namespace aliceVision
