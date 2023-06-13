// This file is part of the extention to AliceVision project.
// Copyright (c) 2018 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "computeOnMultiGPUs.hpp"

#include <aliceVision/alicevision_omp.hpp>
#include <aliceVision/depthMap/cuda/host/utils.hpp>

namespace aliceVision {
namespace depthMap {

void computeOnMultiGPUs(const std::vector<int>& cams, IGPUJob& gpujob, int nbGPUsToUse)
{
    const int nbGPUDevices = listCudaDevices();
    const int nbCPUThreads = omp_get_max_threads();

    ALICEVISION_LOG_INFO("Number of GPU devices: " << nbGPUDevices << ", number of CPU threads: " << nbCPUThreads);

    int nbThreads = std::min(nbGPUDevices, nbCPUThreads);

    if (nbGPUsToUse > 0)
    {
        // Use the user specified limit on the number of GPUs to use
        nbThreads = std::min(nbThreads, nbGPUsToUse);
    }

    if (nbThreads == 1)
    {
        // the GPU sorting is determined by an environment variable named CUDA_DEVICE_ORDER
        // possible values: FASTEST_FIRST (default) or PCI_BUS_ID
        const int cudaDeviceId = 0;
        gpujob.compute(cudaDeviceId, cams);
    }
    else
    {
        //backup max threads to keep potentially previously set value
        int previous_count_threads = omp_get_max_threads();
        omp_set_num_threads(nbThreads); // create as many CPU threads as there are CUDA devices
#pragma omp parallel
        {
            const int cpuThreadId = omp_get_thread_num();
            const int cudaDeviceId = cpuThreadId % nbThreads;

            ALICEVISION_LOG_INFO("CPU thread " << cpuThreadId << " (of " << nbThreads << ") uses CUDA device: " << cudaDeviceId);

            const int nbCamsPerThread = (cams.size() / nbThreads);
            const int rcFrom = cudaDeviceId * nbCamsPerThread;
            int rcTo = (cudaDeviceId + 1) * nbCamsPerThread;
            if(cudaDeviceId == nbThreads - 1)
            {
                rcTo = cams.size();
            }

            std::vector<int> subcams;
            subcams.reserve(cams.size());

            for (int rc = rcFrom; rc < rcTo; ++rc)
            {
                subcams.push_back(cams[rc]);
            }

            gpujob.compute(cudaDeviceId, subcams);
        }
        omp_set_num_threads(previous_count_threads);
    }
}

} // namespace depthMap
} // namespace aliceVision
