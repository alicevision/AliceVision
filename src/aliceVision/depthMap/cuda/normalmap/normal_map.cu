// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/depthMap/cuda/commonStructures.hpp>
#include <aliceVision/depthMap/cuda/planeSweeping/host_utils.h>
#include <aliceVision/depthMap/cuda/deviceCommon/device_matrix.cuh>
#include <aliceVision/depthMap/cuda/deviceCommon/device_utils.cuh>
#include <aliceVision/depthMap/cuda/normalmap/normal_map.hpp>
#include <aliceVision/depthMap/cuda/normalmap/device_eig33.cuh>

#include <math_constants.h>

#include <iostream>
#include <algorithm>
#include <map>
#include <array>

namespace aliceVision {
namespace depthMap {

// Macro for checking cuda errors
#define CHECK_CUDA_ERROR()                                                    \
    if(cudaError_t err = cudaGetLastError())                                  \
    {                                                                         \
        fprintf(stderr, "\n\nCUDAError: %s\n", cudaGetErrorString(err));      \
        fprintf(stderr, "  file:       %s\n", __FILE__);                      \
        fprintf(stderr, "  function:   %s\n", __FUNCTION__);                  \
        fprintf(stderr, "  line:       %d\n\n", __LINE__);                    \
        std::stringstream s;                                                  \
        s << "\n  CUDA Error: " << cudaGetErrorString(err)                    \
          << "\n  file:       " << __FILE__                                   \
          << "\n  function:   " << __FUNCTION__                               \
          << "\n  line:       " << __LINE__ << "\n";                          \
        throw std::runtime_error(s.str());                                    \
    }

__device__ static inline
float3 get3DPointForPixelAndDepthFromRC(const CameraStructBase& rc_cam, const float2& pix, float depth)
{
    float3 rpv = M3x3mulV2(rc_cam.iP, pix);
    normalize(rpv);
    return rc_cam.C + rpv * depth;
}

__device__ static inline
float3 get3DPointForPixelAndDepthFromRC(const CameraStructBase& rc_cam, const int2& pixi, float depth)
{
    float2 pix;
    pix.x = (float)pixi.x;
    pix.y = (float)pixi.y;
    return get3DPointForPixelAndDepthFromRC(rc_cam, pix, depth);
}

__device__ static inline
float orientedPointPlaneDistanceNormalizedNormal(const float3& point, const float3& planePoint,
                                                 const float3& planeNormalNormalized)
{
    return (dot(point, planeNormalNormalized) - dot(planePoint, planeNormalNormalized));
}

__global__ void computeNormalMap_kernel(
    const CameraStructBase& rc_cam,
    float* depthMap, int depthMap_p, //cudaTextureObject_t depthsTex,
    float3* nmap, int nmap_p,
    int width, int height, int wsh, const float gammaC, const float gammaP)
{
  int x = blockIdx.x * blockDim.x + threadIdx.x;
  int y = blockIdx.y * blockDim.y + threadIdx.y;

  if ((x >= width) || (y >= height))
    return;

  float depth = *get2DBufferAt<float>(depthMap, depthMap_p, x, y); // tex2D<float>(depthsTex, x, y);
  if(depth <= 0.0f)
  {
    *get2DBufferAt(nmap, nmap_p, x, y) = make_float3(-1.f, -1.f, -1.f);
    return;
  }

  int2 pix1 = make_int2(x, y);
  float3 p = get3DPointForPixelAndDepthFromRC(rc_cam, pix1, depth);
  float pixSize = 0.0f;
  {
    int2 pix2 = make_int2(x + 1, y);
    float3 p2 = get3DPointForPixelAndDepthFromRC(rc_cam, pix2, depth);
    pixSize = size(p - p2);
  }

  cuda_stat3d s3d = cuda_stat3d();

  for (int yp = -wsh; yp <= wsh; ++yp)
  {
    for (int xp = -wsh; xp <= wsh; ++xp)
    {
      float depthn = *get2DBufferAt<float>(depthMap, depthMap_p, x + xp, y + yp); // tex2D<float>(depthsTex, x + xp, y + yp);
      if ((depth > 0.0f) && (fabs(depthn - depth) < 30.0f * pixSize))
      {
        float w = 1.0f;
        float2 pixn = make_float2(x + xp, y + yp);
        float3 pn = get3DPointForPixelAndDepthFromRC(rc_cam, pixn, depthn);
        s3d.update(pn, w);
      }
    }
  }

  float3 pp = p;
  float3 nn = make_float3(-1.f, -1.f, -1.f);
  if(!s3d.computePlaneByPCA(pp, nn))
  {
    *get2DBufferAt(nmap, nmap_p, x, y) = make_float3(-1.f, -1.f, -1.f);
    return;
  }

  float3 nc = rc_cam.C - p;
  normalize(nc);
  if (orientedPointPlaneDistanceNormalizedNormal(pp + nn, pp, nc) < 0.0f)
  {
    nn.x = -nn.x;
    nn.y = -nn.y;
    nn.z = -nn.z;
  }
  *get2DBufferAt(nmap, nmap_p, x, y) = nn;
}

void ps_computeNormalMap(
    NormalMapping* mapping,
    int width, int height,
    int scale, int ncamsAllocated, int scales, int wsh, bool verbose,
    float gammaC, float gammaP)
{
  clock_t tall = tic();

  const CameraStructBase* camera = mapping->camsBasesDev;

  CudaDeviceMemoryPitched<float, 2>  depthMap_dmp(CudaSize<2>( width, height ));
  depthMap_dmp.copyFrom( mapping->getDepthMapHst(), width, height );

  CudaDeviceMemoryPitched<float3, 2> normalMap_dmp(CudaSize<2>( width, height ));

  int block_size = 8;
  dim3 block(block_size, block_size, 1);
  dim3 grid(divUp(width, block_size), divUp(height, block_size), 1);

  if (verbose)
    printf("computeNormalMap_kernel\n");

  // compute normal map
  computeNormalMap_kernel<<<grid, block>>>(
    *camera,
    depthMap_dmp.getBuffer(),
    depthMap_dmp.getPitch(),
    normalMap_dmp.getBuffer(),
    normalMap_dmp.getPitch(),
    width, height, wsh,
    gammaC, gammaP);

  // cudaThreadSynchronize();
  // CHECK_CUDA_ERROR();

  if (verbose)
    printf("copy normal map to host\n");

  normalMap_dmp.copyTo( mapping->getNormalMapHst(), width, height );
  CHECK_CUDA_ERROR();

  if (verbose)
    printf("gpu elapsed time: %f ms \n", toc(tall));
}

NormalMapping::NormalMapping()
    : _allocated_floats(0)
    , _depthMapHst(0)
    , _normalMapHst(0)
{
    cudaError_t err;

    err = cudaMallocHost( &camsBasesHst, sizeof(CameraStructBase) );
    THROW_ON_CUDA_ERROR( err, "Failed to allocate camera parameters on host in normal mapping" );

    err = cudaMalloc(     &camsBasesDev, sizeof(CameraStructBase) );
    THROW_ON_CUDA_ERROR( err, "Failed to allocate camera parameters on device in normal mapping" );
}

NormalMapping::~NormalMapping()
{
    cudaFree(     camsBasesDev );
    cudaFreeHost( camsBasesHst );

    if( _depthMapHst  ) cudaFreeHost( _depthMapHst );
    if( _normalMapHst ) cudaFreeHost( _normalMapHst );
}

void NormalMapping::loadCameraParameters()
{
    cudaError_t err;
    err = cudaMemcpy( camsBasesDev,
                      camsBasesHst,
                      sizeof(CameraStructBase),
                      cudaMemcpyHostToDevice );
    THROW_ON_CUDA_ERROR( err, "Failed to copy camera parameters from host to device in normal mapping" );
}

void NormalMapping::allocHostMaps( int w, int h )
{
    cudaError_t err;
    if( _depthMapHst )
    {
        if( w*h > _allocated_floats );
        {
            err = cudaFreeHost( _depthMapHst );
            THROW_ON_CUDA_ERROR( err, "Failed to free host depth map in normal mapping" );
            err = cudaMallocHost( &_depthMapHst, w*h*sizeof(float) );
            THROW_ON_CUDA_ERROR( err, "Failed to re-allocate host depth map in normal mapping" );

            err = cudaFreeHost( _normalMapHst );
            THROW_ON_CUDA_ERROR( err, "Failed to free host normal map in normal mapping" );
            err = cudaMallocHost( &_normalMapHst, w*h*sizeof(float3) );
            THROW_ON_CUDA_ERROR( err, "Failed to re-allocate host normal map in normal mapping" );
            _allocated_floats = w * h;
        }
    }
    else
    {
        err = cudaMallocHost( &_depthMapHst, w*h*sizeof(float) );
        THROW_ON_CUDA_ERROR( err, "Failed to allocate host depth map in normal mapping" );
        err = cudaMallocHost( &_normalMapHst, w*h*sizeof(float3) );
        THROW_ON_CUDA_ERROR( err, "Failed to allocate host normal map in normal mapping" );
        _allocated_floats = w * h;
    }
}

void NormalMapping::copyDepthMap( const std::vector<float>& depthMap )
{
    if( _allocated_floats > depthMap.size() )
    {
        std::cerr << "WARNING: " << __FILE__ << ":" << __LINE__
                  << ": copying depthMap whose origin is too small" << std::endl;
    }
    memcpy( _depthMapHst, depthMap.data(), _allocated_floats*sizeof(float) );
}

const float* NormalMapping::getDepthMapHst() const
{
    return _depthMapHst;
}

float3* NormalMapping::getNormalMapHst()
{
    return _normalMapHst;
}

} // namespace depthMap
} // namespace aliceVision

