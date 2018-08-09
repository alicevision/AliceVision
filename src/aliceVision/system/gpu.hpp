// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/config.hpp>
#include <aliceVision/system/Logger.hpp>

#include <string>
#include <sstream>
#include <memory.h>

#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_CUDA)
#include <cuda_runtime.h>
#endif

namespace aliceVision {
namespace system {

/**
 * @brief Check if the system support CUDA with the given parameters
 * @param[in] minComputeCapabilityMajor The minimum compute capability major
 * @param[in] minComputeCapabilityMinor The minimum compute capability minor
 * @param[in] minTotalDeviceMemory The minimum device total memory in MB
 * @return True if system support CUDA with the given parameters
 */
inline bool gpuSupportCUDA(int minComputeCapabilityMajor,
                    int minComputeCapabilityMinor,
                    int minTotalDeviceMemory = 0)
{
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_CUDA)
  int nbDevices = 0;
  cudaError_t success;
  success = cudaGetDeviceCount(&nbDevices);
  if( success != cudaSuccess )
  {
    ALICEVISION_LOG_ERROR("cudaGetDeviceCount failed: " << cudaGetErrorString(success));
  }

  if(nbDevices > 0)
  {
    for(int i = 0; i < nbDevices; ++i)
    {
      cudaDeviceProp deviceProperties;

      if(cudaGetDeviceProperties(&deviceProperties, i) != cudaSuccess)
        throw std::runtime_error("Cannot get properties for CUDA gpu device " + std::to_string(i));

      if((deviceProperties.major > minComputeCapabilityMajor ||
         (deviceProperties.major == minComputeCapabilityMajor &&
          deviceProperties.minor >= minComputeCapabilityMinor)) &&
          deviceProperties.totalGlobalMem >= (minTotalDeviceMemory*1024*1024))
      {
        ALICEVISION_LOG_INFO("Supported CUDA-Enabled GPU detected.");
        return true;
      }
      else
      {
        ALICEVISION_LOG_ERROR("CUDA-Enabled GPU detected, but the compute capabilities is not enough.\n"
          << " - Device " << i << ": " << deviceProperties.major << "." << deviceProperties.minor << ", global memory: " << int(deviceProperties.totalGlobalMem / (1024*1024)) << "MB\n"
          << " - Requirements: " << minComputeCapabilityMajor << "." << minComputeCapabilityMinor << ", global memory: " << minTotalDeviceMemory << "MB\n"
          );
      }
    }
    ALICEVISION_LOG_INFO("CUDA-Enabled GPU not supported.");
  }
  else
  {
    ALICEVISION_LOG_INFO("Can't find CUDA-Enabled GPU.");
  }
#endif
  return false;
}

/**
 * @brief gpuInformationCUDA
 * @return string with all CUDA device(s) information
 */
inline std::string gpuInformationCUDA()
{
  std::string information;
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_CUDA)
  int nbDevices = 0;
  if( cudaGetDeviceCount(&nbDevices) != cudaSuccess )
  {
    ALICEVISION_LOG_WARNING( "Could not determine number of CUDA cards in this system" );
    nbDevices = 0;
  }

  if(nbDevices > 0)
  {
    information = "CUDA-Enabled GPU.\n";
    for(int i = 0; i < nbDevices; ++i)
    {
      cudaDeviceProp deviceProperties;
      if(cudaGetDeviceProperties( &deviceProperties, i) != cudaSuccess )
        throw std::runtime_error("Cannot get properties for CUDA gpu device " + std::to_string(i));

      if( cudaSetDevice( i ) != cudaSuccess )
      {
        ALICEVISION_LOG_WARNING( "Device with number " << i << " does not exist" );
        continue;
      }

      std::size_t avail;
      std::size_t total;
      if(cudaMemGetInfo(&avail, &total) != cudaSuccess)
      {
          // if the card does not provide this information.
          avail = 0;
          total = 0;
          ALICEVISION_LOG_WARNING("Cannot get available memory information for CUDA gpu device " << i << ".");
      }
      std::stringstream deviceSS;

      deviceSS << "Device information:" << std::endl
               << "\t- id:                      " << i << std::endl
               << "\t- name:                    " << deviceProperties.name << std::endl
               << "\t- compute capability:      " << deviceProperties.major << "." << deviceProperties.minor << std::endl
               << "\t- total device memory:     " << deviceProperties.totalGlobalMem / (1024 * 1024) << " MB " << std::endl
               << "\t- device memory available: " << avail / (1024 * 1024) << " MB " << std::endl
               << "\t- per-block shared memory: " << deviceProperties.sharedMemPerBlock << std::endl
               << "\t- warp size:               " << deviceProperties.warpSize << std::endl
               << "\t- max threads per block:   " << deviceProperties.maxThreadsPerBlock << std::endl
               << "\t- max threads per SM(X):   " << deviceProperties.maxThreadsPerMultiProcessor << std::endl
               << "\t- max block sizes:         "
               << "{" << deviceProperties.maxThreadsDim[0]
               << "," << deviceProperties.maxThreadsDim[1]
               << "," << deviceProperties.maxThreadsDim[2] << "}" << std::endl
               << "\t- max grid sizes:          "
               << "{" << deviceProperties.maxGridSize[0]
               << "," << deviceProperties.maxGridSize[1]
               << "," << deviceProperties.maxGridSize[2] << "}" << std::endl
               << "\t- max 2D array texture:    "
               << "{" << deviceProperties.maxTexture2D[0]
               << "," << deviceProperties.maxTexture2D[1] << "}" << std::endl
               << "\t- max 3D array texture:    "
               << "{" << deviceProperties.maxTexture3D[0]
               << "," << deviceProperties.maxTexture3D[1]
               << "," << deviceProperties.maxTexture3D[2] << "}" << std::endl
               << "\t- max 2D linear texture:   "
               << "{" << deviceProperties.maxTexture2DLinear[0]
               << "," << deviceProperties.maxTexture2DLinear[1]
               << "," << deviceProperties.maxTexture2DLinear[2] << "}" << std::endl
               << "\t- max 2D layered texture:  "
               << "{" << deviceProperties.maxTexture2DLayered[0]
               << "," << deviceProperties.maxTexture2DLayered[1]
               << "," << deviceProperties.maxTexture2DLayered[2] << "}" << std::endl
               << "\t- number of SM(x)s:        " << deviceProperties.multiProcessorCount << std::endl
               << "\t- registers per SM(x):     " << deviceProperties.regsPerMultiprocessor << std::endl
               << "\t- registers per block:     " << deviceProperties.regsPerBlock << std::endl
               << "\t- concurrent kernels:      " << (deviceProperties.concurrentKernels ? "yes":"no") << std::endl
               << "\t- mapping host memory:     " << (deviceProperties.canMapHostMemory ? "yes":"no") << std::endl
               << "\t- unified addressing:      " << (deviceProperties.unifiedAddressing ? "yes":"no") << std::endl
               << "\t- texture alignment:       " << deviceProperties.textureAlignment << " byte" << std::endl
               << "\t- pitch alignment:         " << deviceProperties.texturePitchAlignment << " byte" << std::endl;

      information += deviceSS.str();
    }
  }
#else
  information = "No CUDA-Enabled GPU.";
#endif
  return information;
}

} // namespace system
} // namespace aliceVision
