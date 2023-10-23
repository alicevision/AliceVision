// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "utils.hpp"

#include <aliceVision/system/Logger.hpp>

#include <cuda_runtime.h>

namespace aliceVision {
namespace depthMap {

int listCudaDevices()
{
    int nbDevices = 0;  // number of CUDA GPUs

    // determine the number of CUDA capable GPUs
    cudaError_t err = cudaGetDeviceCount(&nbDevices);
    CHECK_CUDA_ERROR();
    if (err != cudaSuccess)
    {
        ALICEVISION_LOG_ERROR("Cannot get CUDA device count.");
        return 0;
    }

    if (nbDevices < 1)
    {
        ALICEVISION_LOG_ERROR("No CUDA capable devices detected.");
        return 0;
    }

    // display CPU and GPU configuration
    std::stringstream s;
    for (int i = 0; i < nbDevices; ++i)
    {
        cudaDeviceProp dprop;
        cudaGetDeviceProperties(&dprop, i);
        s << "\t- Device " << i << ": " << dprop.name << std::endl;
    }
    ALICEVISION_LOG_DEBUG(nbDevices << " CUDA devices found:" << std::endl << s.str());

    return nbDevices;
}

int getCudaDeviceId()
{
    int currentCudaDeviceId;

    if (cudaGetDevice(&currentCudaDeviceId) != cudaSuccess)
    {
        ALICEVISION_LOG_ERROR("Cannot get current CUDA device id.");
    }

    CHECK_CUDA_ERROR();

    return currentCudaDeviceId;
}

void setCudaDeviceId(int cudaDeviceId)
{
    if (cudaSetDevice(cudaDeviceId) != cudaSuccess)
    {
        ALICEVISION_LOG_ERROR("Cannot set device id " << cudaDeviceId << " as current CUDA device.");
    }

    CHECK_CUDA_ERROR();
}

bool testCudaDeviceId(int cudaDeviceId)
{
    int currentCudaDeviceId;
    cudaGetDevice(&currentCudaDeviceId);
    if (currentCudaDeviceId != cudaDeviceId)
    {
        ALICEVISION_LOG_WARNING("CUDA device id should be: " << cudaDeviceId << ", program curently use device id: " << currentCudaDeviceId << ".");
        return false;
    }
    return true;
}

void logDeviceMemoryInfo()
{
    size_t iavail;
    size_t itotal;

    cudaMemGetInfo(&iavail, &itotal);

    const double availableMB = double(iavail) / (1024.0 * 1024.0);
    const double totalMB = double(itotal) / (1024.0 * 1024.0);
    const double usedMB = double(itotal - iavail) / (1024.0 * 1024.0);

    int cudaDeviceId;
    cudaGetDevice(&cudaDeviceId);

    ALICEVISION_LOG_INFO("Device memory (device id: " << cudaDeviceId << "):" << std::endl
                                                      << "\t- used: " << usedMB << " MB" << std::endl
                                                      << "\t- available: " << availableMB << " MB" << std::endl
                                                      << "\t- total: " << totalMB << " MB");
}

void getDeviceMemoryInfo(double& availableMB, double& usedMB, double& totalMB)
{
    size_t iavail;
    size_t itotal;

    cudaMemGetInfo(&iavail, &itotal);

    availableMB = double(iavail) / (1024.0 * 1024.0);
    totalMB = double(itotal) / (1024.0 * 1024.0);
    usedMB = double(itotal - iavail) / (1024.0 * 1024.0);
}

}  // namespace depthMap
}  // namespace aliceVision
