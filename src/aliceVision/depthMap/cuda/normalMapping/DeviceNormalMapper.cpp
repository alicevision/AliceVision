// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "DeviceNormalMapper.hpp"

#include <aliceVision/depthMap/cuda/host/memory.hpp>

namespace aliceVision {
namespace depthMap {

DeviceNormalMapper::DeviceNormalMapper()
    : _allocated_floats(0)
    , _depthMapHst(0)
    , _normalMapHst(0)
{
    cudaError_t err;

    err = cudaMallocHost(&cameraParameters_h, sizeof(DeviceCameraParams) );
    THROW_ON_CUDA_ERROR( err, "Failed to allocate camera parameters on host in normal mapping" );

    err = cudaMalloc(&cameraParameters_d, sizeof(DeviceCameraParams));
    THROW_ON_CUDA_ERROR( err, "Failed to allocate camera parameters on device in normal mapping" );
}

DeviceNormalMapper::~DeviceNormalMapper()
{
    cudaFree(cameraParameters_d);
    cudaFreeHost(cameraParameters_h);

    if( _depthMapHst  ) cudaFreeHost( _depthMapHst );
    if( _normalMapHst ) cudaFreeHost( _normalMapHst );
}

void DeviceNormalMapper::loadCameraParameters()
{
    cudaError_t err;
    err = cudaMemcpy(cameraParameters_d, cameraParameters_h, sizeof(DeviceCameraParams), cudaMemcpyHostToDevice);
    THROW_ON_CUDA_ERROR( err, "Failed to copy camera parameters from host to device in normal mapping" );
}

void DeviceNormalMapper::allocHostMaps(int w, int h)
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

void DeviceNormalMapper::copyDepthMap(const std::vector<float>& depthMap)
{
    if( _allocated_floats > depthMap.size() )
    {
        std::cerr << "WARNING: " << __FILE__ << ":" << __LINE__
                  << ": copying depthMap whose origin is too small" << std::endl;
    }
    memcpy( _depthMapHst, depthMap.data(), _allocated_floats*sizeof(float) );
}

const float* DeviceNormalMapper::getDepthMapHst() const
{
    return _depthMapHst;
}

float3* DeviceNormalMapper::getNormalMapHst()
{
    return _normalMapHst;
}

} // namespace depthMap
} // namespace aliceVision

