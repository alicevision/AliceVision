// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <cuda_runtime.h>
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>

namespace aliceVision {
namespace depthMap {

template <unsigned Dim> class CudaSizeBase
{
public:
  CudaSizeBase()
  {
    for(int i = Dim; i--;)
      size[i] = 0;
  }
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
    if (Dim >= 3) sx = _size[2];
    buffer = (Type*)malloc(sx * sy * sz * sizeof (Type));
    if( buffer == 0 ) {
        printf("%d malloc failed\n", __LINE__ );
        exit(-1);
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
    if (Dim >= 3) sx = rhs.sz;
    buffer = (Type*)malloc(sx * sy * sz * sizeof (Type));
    if( buffer == 0 ) {
        printf("%d malloc failed\n", __LINE__ );
        exit(-1);
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
};

template <class Type, unsigned Dim> class CudaDeviceMemoryPitched
{
  Type* buffer;
  size_t pitch;
  size_t sx, sy, sz;
  CudaSize<Dim> size;
public:
  explicit CudaDeviceMemoryPitched(const CudaSize<Dim> &_size)
  {
    size = _size;
    sx = 1;
    sy = 1;
    sz = 1;
    if (Dim >= 1) sx = _size[0];
    if (Dim >= 2) sy = _size[1];
    if (Dim >= 3) sx = _size[2];
    if(Dim == 2)
    {
      cudaMallocPitch<Type>(&buffer, &pitch, _size[0] * sizeof(Type), _size[1]);
    }
    if(Dim >= 3)
    {
      cudaExtent extent;
      extent.width = _size[0] * sizeof(Type);
      extent.height = _size[1];
      extent.depth = _size[2];
      for(unsigned i = 3; i < Dim; ++i)
        extent.depth *= _size[i];
      cudaPitchedPtr pitchDevPtr;
      cudaMalloc3D(&pitchDevPtr, extent);
      buffer = (Type*)pitchDevPtr.ptr;
      pitch = pitchDevPtr.pitch;
    }
  }
  ~CudaDeviceMemoryPitched()
  {
    cudaFree(buffer);
  }
  explicit inline CudaDeviceMemoryPitched(const CudaHostMemoryHeap<Type, Dim> &rhs)
  {
    size = rhs.getSize();
    sx = 1;
    sy = 1;
    sz = 1;
    if (Dim >= 1) sx = rhs.getSize()[0];
    if (Dim >= 2) sy = rhs.getSize()[1];
    if (Dim >= 3) sx = rhs.getSize()[2];
    if(Dim == 2)
    {
      cudaMallocPitch<Type>(&buffer, &pitch, size[0] * sizeof(Type), size[1]);
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
      cudaMalloc3D(&pitchDevPtr, extent);
      buffer = (Type*)pitchDevPtr.ptr;
      pitch = pitchDevPtr.pitch;
    }
    copy(*this, rhs);
  }
  CudaDeviceMemoryPitched<Type,Dim> & operator=(const CudaDeviceMemoryPitched<Type,Dim> & rhs)
  {
    copy(*this, rhs);
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

};

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
    if (Dim >= 3) sx = _size[2];
    cudaChannelFormatDesc channelDesc = cudaCreateChannelDesc<Type>();
    if(Dim == 1)
    {
      err = cudaMallocArray(&array, &channelDesc, _size[0], 1, cudaArraySurfaceLoadStore);
      if( err != cudaSuccess )
      {
          printf( "Failed to allocate CUDA Array in line %d\n", __LINE__-3 );
      }
    }
    else if(Dim == 2)
    {
      err = cudaMallocArray(&array, &channelDesc, _size[0], _size[1], cudaArraySurfaceLoadStore);
      if( err != cudaSuccess )
      {
          printf( "Failed to allocate CUDA Array in line %d\n", __LINE__-3 );
      }
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
      if( err != cudaSuccess )
      {
          printf( "Failed to allocate CUDA Array in line %d\n", __LINE__-3 );
      }
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
    if (Dim >= 3) sx = rhs.getSize()[2];
    cudaChannelFormatDesc channelDesc = cudaCreateChannelDesc<Type>();
    if(Dim == 1)
    {
      err = cudaMallocArray(&array, &channelDesc, size[0], 1, cudaArraySurfaceLoadStore);
      if( err != cudaSuccess )
      {
          printf( "Failed to allocate CUDA Array in line %d\n", __LINE__-3 );
      }
    }
    else if(Dim == 2)
    {
      err = cudaMallocArray(&array, &channelDesc, size[0], size[1], cudaArraySurfaceLoadStore);
      if( err != cudaSuccess )
      {
          printf( "Failed to allocate CUDA Array in line %d\n", __LINE__-3 );
      }
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
      if( err != cudaSuccess )
      {
          printf( "Failed to allocate CUDA Array in line %d\n", __LINE__-3 );
      }
    }
    copy(*this, rhs);
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
    if (Dim >= 3) sx = rhs.getSize()[2];
    cudaChannelFormatDesc channelDesc = cudaCreateChannelDesc<Type>();
    if(Dim == 1)
    {
      err = cudaMallocArray(&array, &channelDesc, size[0], 1, cudaArraySurfaceLoadStore);
      if( err != cudaSuccess )
      {
          printf( "Failed to allocate CUDA Array in line %d\n", __LINE__-3 );
      }
    }
    else if(Dim == 2)
    {
      err = cudaMallocArray(&array, &channelDesc, size[0], size[1], cudaArraySurfaceLoadStore);
      if( err != cudaSuccess )
      {
          printf( "Failed to allocate CUDA Array in line %d\n", __LINE__-3 );
      }
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
      if( err != cudaSuccess )
      {
          printf( "Failed to allocate CUDA Array in line %d\n", __LINE__-3 );
      }
    }
    copy(*this, rhs);
  }
  virtual ~CudaArray()
  {
    cudaFreeArray(array);
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
};

template<class Type, unsigned Dim> void copy(CudaHostMemoryHeap<Type, Dim>& _dst, const CudaDeviceMemoryPitched<Type, Dim>& _src)
{
  cudaMemcpyKind kind = cudaMemcpyDeviceToHost;
  if(Dim == 1) {
    cudaMemcpy(_dst.getBuffer(), _src.getBuffer(), _src.getBytes(), kind);
  }
  else if(Dim == 2) {
    cudaMemcpy2D(_dst.getBuffer(), _dst.getSize()[0] * sizeof (Type), _src.getBuffer(), _src.getPitch(), _dst.getSize()[0] * sizeof (Type), _dst.getSize()[1], kind);
  }
  else if(Dim >= 3) {
    for (unsigned int slice=0; slice<_dst.getSize()[2]; slice++)
    {
      cudaMemcpy2D( _dst.getBuffer() + slice * _dst.getSize()[0] * _dst.getSize()[1], _dst.getSize()[0] * sizeof (Type), &_src.getBuffer()[slice * _src.stride()[1]], _src.getPitch(), _dst.getSize()[0] * sizeof (Type), _dst.getSize()[1], kind);
    }
  }
}

template<class Type, unsigned Dim> void copy(CudaHostMemoryHeap<Type, Dim>& _dst, const CudaArray<Type, Dim>& _src)
{
  cudaMemcpyKind kind = cudaMemcpyDeviceToHost;
  if(Dim == 1) {
    cudaMemcpyFromArray(_dst.getBuffer(), _src.getArray(), 0, 0, _dst.getSize()[0] * sizeof (Type), kind);
  }
  else if(Dim == 2) {
    cudaMemcpy2DFromArray(_dst.getBuffer(), _dst.getSize()[0] * sizeof (Type), _src.getArray(), 0, 0, _dst.getSize()[0] * sizeof (Type), _dst.getSize()[1], kind);
  }
  else if(Dim == 3) {
    cudaMemcpy3DParms p = { 0 };
    p.srcArray = const_cast<cudaArray *>(_src.getArray());
    p.srcPtr.pitch = _src.getPitch();
    p.srcPtr.xsize = _src.getSize()[0];
    p.srcPtr.ysize = _src.getSize()[1];
    p.dstPtr.ptr = (void *)_dst.getBuffer();
    p.dstPtr.pitch = _dst.getSize()[0] * sizeof (Type);
    p.dstPtr.xsize = _dst.getSize()[0];
    p.dstPtr.ysize = _dst.getSize()[1];
    p.extent.width = _dst.getSize()[0];
    p.extent.height = _dst.getSize()[1];
    p.extent.depth = _dst.getSize()[2];
    for(unsigned i = 3; i < Dim; ++i)
      p.extent.depth *= _src.getSize()[i];
    p.kind = kind;
    cudaMemcpy3D(&p);
  }
}

template<class Type, unsigned Dim> void copy(CudaDeviceMemoryPitched<Type, Dim>& _dst, const CudaHostMemoryHeap<Type, Dim>& _src)
{
  cudaMemcpyKind kind = cudaMemcpyHostToDevice;
  if(Dim == 1) {
    cudaMemcpy(_dst.getBuffer(), _src.getBuffer(), _src.getBytes(), kind);
  }
  else if(Dim == 2) {
    cudaMemcpy2D(_dst.getBuffer(), _dst.getPitch(), _src.getBuffer(), _src.getSize()[0] * sizeof (Type), _src.getSize()[0] * sizeof (Type), _src.getSize()[1], kind);
  }
  else if(Dim >= 3) {
    for (unsigned int slice=0; slice<_src.getSize()[2]; slice++)
    {
      cudaMemcpy2D( &_dst.getBuffer()[slice * _dst.stride()[1]], _dst.getPitch(), _src.getBuffer() + slice * _src.getSize()[0] * _src.getSize()[1], _src.getSize()[0] * sizeof (Type), _src.getSize()[0] * sizeof (Type), _src.getSize()[1], kind);
    }
  }
}

template<class Type, unsigned Dim> void copy(CudaDeviceMemoryPitched<Type, Dim>& _dst, const CudaDeviceMemoryPitched<Type, Dim>& _src)
{
  cudaMemcpyKind kind = cudaMemcpyDeviceToDevice;
  if(Dim == 1) {
    cudaMemcpy(_dst.getBuffer(), _src.getBuffer(), _src.getBytes(), kind);
  }
  else if(Dim == 2) {
    cudaMemcpy2D(_dst.getBuffer(), _dst.getPitch(), _src.getBuffer(), _src.getPitch(), _src.getSize()[0] * sizeof(Type), _src.getSize()[1], kind);
  }
  else if(Dim >= 3) {
    for (unsigned int slice=0; slice<_src.getSize()[2]; slice++)
    {
      cudaMemcpy2D( &_dst.getBuffer()[slice * _dst.stride()[1]], _dst.getPitch(), &_src.getBuffer()[slice * _src.stride()[1]], _src.getPitch(), _src.getSize()[0] * sizeof(Type), _src.getSize()[1], kind);
    }
  }
}

template<class Type, unsigned Dim> void copy(CudaDeviceMemoryPitched<Type, Dim>& _dst, const CudaArray<Type, Dim>& _src)
{
  cudaMemcpyKind kind = cudaMemcpyDeviceToDevice;
  if(Dim == 1) {
    cudaMemcpyFromArray(_dst.getBuffer(), _src.getArray(), 0, 0, _src.getSize()[0] * sizeof(Type), kind);
  }
  else if(Dim == 2) {
    cudaMemcpy2DFromArray(_dst.getBuffer(), _dst.getPitch(), _src.getArray(), 0, 0, _src.getSize()[0] * sizeof(Type), _src.getSize()[1], kind);
  }
  else if(Dim == 3) {
    cudaMemcpy3DParms p = { 0 };
    p.srcArray = const_cast<cudaArray *>(_src.getArray());
    p.srcPtr.pitch = _src.getPitch();
    p.srcPtr.xsize = _src.getSize()[0];
    p.srcPtr.ysize = _src.getSize()[1];
    p.dstPtr.ptr = (void *)_dst.getBuffer();
    p.dstPtr.pitch = _dst.getPitch();
    p.dstPtr.xsize = _dst.getSize()[0];
    p.dstPtr.ysize = _dst.getSize()[1];
    p.extent.width = _src.getSize()[0];
    p.extent.height = _src.getSize()[1];
    p.extent.depth = _src.getSize()[2];
    for(unsigned i = 3; i < Dim; ++i)
      p.extent.depth *= _src.getSize()[i];
    p.kind = kind;
    cudaMemcpy3D(&p);
  }
}

template<class Type, unsigned Dim> void copy(CudaArray<Type, Dim>& _dst, const CudaHostMemoryHeap<Type, Dim>& _src)
{
  cudaMemcpyKind kind = cudaMemcpyHostToDevice;
  if(Dim == 1) {
    cudaMemcpyToArray(_dst.getArray(), 0, 0, _src.getBuffer(), _src.getSize()[0] * sizeof (Type), kind);
  }
  else if(Dim == 2) {
    cudaMemcpy2DToArray(_dst.getArray(), 0, 0, _src.getBuffer(), _src.getSize()[0] * sizeof (Type), _src.getSize()[0] * sizeof (Type), _src.getSize()[1], kind);
  }
  else if(Dim == 3) {
    cudaMemcpy3DParms p = { 0 };
    p.srcPtr.ptr = (void *)_src.getBuffer();
    p.srcPtr.pitch = _src.getSize()[0] * sizeof (Type);
    p.srcPtr.xsize = _src.getSize()[0];
    p.srcPtr.ysize = _src.getSize()[1];
    p.dstArray = _dst.getArray();
    p.dstPtr.pitch = _dst.getPitch();
    p.dstPtr.xsize = _dst.getSize()[0];
    p.dstPtr.ysize = _dst.getSize()[1];
    p.extent.width = _src.getSize()[0];
    p.extent.height = _src.getSize()[1];
    p.extent.depth = _src.getSize()[2];
    for(unsigned i = 3; i < Dim; ++i)
      p.extent.depth *= _src.getSize()[i];
    p.kind = kind;
    cudaMemcpy3D(&p);
  }
}

template<class Type, unsigned Dim> void copy(CudaArray<Type, Dim>& _dst, const CudaDeviceMemoryPitched<Type, Dim>& _src)
{
  cudaMemcpyKind kind = cudaMemcpyDeviceToDevice;
  if(Dim == 1) {
    cudaMemcpyToArray(_dst.getArray(), 0, 0, _src.getBuffer(), _src.getSize()[0] * sizeof(Type), kind);
  }
  else if(Dim == 2) {
    cudaMemcpy2DToArray(_dst.getArray(), 0, 0, _src.getBuffer(), _src.getPitch(), _src.getSize()[0] * sizeof(Type), _src.getSize()[1], kind);
  }
  else if(Dim == 3) {
    cudaMemcpy3DParms p = { 0 };
    p.srcPtr.ptr = (void *)_src.getBuffer();
    p.srcPtr.pitch = _src.getPitch();
    p.srcPtr.xsize = _src.getSize()[0];
    p.srcPtr.ysize = _src.getSize()[1];
    p.dstArray = _dst.getArray();
    p.dstPtr.pitch = _dst.getPitch();
    p.dstPtr.xsize = _dst.getSize()[0];
    p.dstPtr.ysize = _dst.getSize()[1];
    p.extent.width = _src.getSize()[0];
    p.extent.height = _src.getSize()[1];
    p.extent.depth = _src.getSize()[2];
    for(unsigned i = 3; i < Dim; ++i)
      p.extent.depth *= _src.getSize()[i];
    p.kind = kind;
    cudaMemcpy3D(&p);
  }
}

template<class Type, unsigned Dim> void copy(Type* _dst, size_t sx, size_t sy, const CudaDeviceMemoryPitched<Type, Dim>& _src)
{
  if(Dim == 2) {
    cudaMemcpy2D(_dst, sx * sizeof (Type), _src.getBuffer(), _src.getPitch(), sx * sizeof (Type), sy, cudaMemcpyDeviceToHost);
  }
}

template<class Type, unsigned Dim> void copy(CudaDeviceMemoryPitched<Type, Dim>& _dst, const Type* _src, size_t sx, size_t sy)
{
  if(Dim == 2) {
    cudaMemcpy2D(_dst.getBuffer(), _dst.getPitch(), _src, sx * sizeof (Type), sx * sizeof(Type), sy, cudaMemcpyHostToDevice);
  }
}

template<class Type, unsigned Dim> void copy(Type* _dst, size_t sx, size_t sy, size_t sz, const CudaDeviceMemoryPitched<Type, Dim>& _src)
{
  if(Dim >= 3) {
    for (unsigned int slice=0; slice<sz; slice++)
    {
      cudaMemcpy2D( _dst + sx * sy * slice, sx * sizeof (Type), &_src.getBuffer()[slice * _src.stride()[1]], _src.getPitch(), sx * sizeof (Type), sy, cudaMemcpyDeviceToHost);
    }
  }
}

template<class Type, unsigned Dim> void copy(CudaDeviceMemoryPitched<Type, Dim>& _dst, const Type* _src, size_t sx, size_t sy, size_t sz)
{
  if(Dim >= 3) {
    for (unsigned int slice=0; slice<sz; slice++)
    {
      cudaMemcpy2D( &_dst.getBuffer()[slice * _dst.stride()[1]], _dst.getPitch(), _src + sx * sy * slice, sx * sizeof (Type), sx * sizeof(Type), sy, cudaMemcpyHostToDevice);
    }
  }
}

struct cameraStruct
{
    float P[12], iP[9], R[9], iR[9], K[9], iK[9], C[3];
    CudaHostMemoryHeap<uchar4, 2>* tex_rgba_hmh;
    int camId;
    int rc;
    float* H;
    int scale;
    int blurid;
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
