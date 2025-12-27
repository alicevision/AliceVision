// This file is part of the extension to AliceVision project.
// Copyright (c) 2018 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "computeOnMultiDevices.hpp"

#include <aliceVision/alicevision_omp.hpp>
#include <sycl/sycl.hpp>

// Needed for checking device caracteristics
#include <aliceVision/depthMap_sycl/sycl/planeSweeping/similarity.hpp>
#include <aliceVision/depthMap_sycl/sycl/memory.hpp>

namespace aliceVision {
namespace depthMap {

void computeOnMultiDevices(const std::vector<int>& cams, IDeviceJob& devicejob, int nbDevicesToUse)
{
    // aspect_selector finds the "best" device that supports the features we need
    // sycl::detail::select_devices is an AdaptiveCpp unique but it's functionality is easy to replicate should we need to
    // usm_device_allocations and usm_host_allocations should be supported by all AdaptiveCpp backends
    // likewise fp16 and fp64 are probably supported by anything we will be running on
#if defined(TSIM_REFINE_USE_HALF) || defined (ALICEVISION_DEPTHMAP_TEXTURE_USE_HALF)
    const static std::vector<sycl::device> devices = sycl::detail::select_devices(sycl::aspect_selector({
                sycl::aspect::fp64,
                sycl::aspect::fp16,
                sycl::aspect::usm_device_allocations,
                sycl::aspect::usm_host_allocations
            }));
#else
    const static std::vector<sycl::device> devices = sycl::detail::select_devices(sycl::aspect_selector({
                sycl::aspect::fp64,
                sycl::aspect::usm_device_allocations,
                sycl::aspect::usm_host_allocations
            }));
#endif
    const int nbDevices = devices.size();
    if (nbDevices == 0) ALICEVISION_LOG_ERROR("Could not find a suitable device for computation.");
    const int nbCPUThreads = omp_get_max_threads();

    ALICEVISION_LOG_INFO("Number of compute devices: " << nbDevices << ", number of CPU threads: " << nbCPUThreads);

    int nbThreads = std::min(nbDevices, nbCPUThreads);

    if (nbDevicesToUse > 0)
    {
        // Use the user specified limit on the number of Devices to use
        nbThreads = std::min(nbThreads, nbDevicesToUse);
    }

    if (nbThreads == 1)
    {
        const sycl::device device = devices[0];
        ALICEVISION_LOG_INFO("Using device " << device.get_info<sycl::info::device::name>());
        const sycl::queue queue = sycl::queue(device);
        devicejob.compute(queue, cams);
    }
    else
    {
        // loadsharing algorithm to balance differently powered devices
        // we need to do floating point division regardless, so there is no point in storing values as ints
        float totalCapacity = 0;
        float allocatedCapacity = 0;
        std::vector<float> deviceCapacities(nbThreads);
        std::vector<std::vector<int>> deviceCams(nbThreads);

        for (int i; i<nbThreads; i++)
        {
            float deviceCapacity = devices.at(i).get_info<sycl::info::device::max_compute_units>()*devices.at(i).get_info<sycl::info::device::max_clock_frequency>(); // functional if perhaps crude
            totalCapacity += deviceCapacity;
            deviceCapacities.at(i) = deviceCapacity;
        }

        for (int i; i<nbThreads; i++)
        {
            auto start = cams.begin() + int(allocatedCapacity/totalCapacity*cams.size());
            allocatedCapacity += deviceCapacities.at(i);
            auto end = (i == nbThreads - 1) ? cams.end() : cams.begin() + int(allocatedCapacity/totalCapacity*cams.size()) - 1;
            deviceCams.at(i) = std::vector<int>(start,end);
        }

        // backup max threads to keep potentially previously set value
        int previous_count_threads = omp_get_max_threads();
        omp_set_num_threads(nbThreads);  // create as many CPU threads as there are SYCL devices
#pragma omp parallel
        {
            const int cpuThreadId = omp_get_thread_num();
            const int deviceId = cpuThreadId % nbThreads;
            const sycl::device device = devices.at(deviceId);
            const sycl::queue queue = sycl::queue(device); // AdaptiveCpp reccomends in-order queues for performance, they also make code cleaner

            ALICEVISION_LOG_INFO("CPU thread " << cpuThreadId << " (of " << nbThreads << ") uses device " << deviceId << " with name " << device.get_info<sycl::info::device::name>());

            devicejob.compute(queue, deviceCams.at(deviceId));
        }
        omp_set_num_threads(previous_count_threads);
    }
}

}  // namespace depthMap
}  // namespace aliceVision
