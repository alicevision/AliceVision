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

static auto async_handler_object = [] (sycl::exception_list exceptions) {
  for (auto e : exceptions) {
    try {
      std::rethrow_exception(e);
    } catch (sycl::exception const &e) {
        ALICEVISION_LOG_INFO("Caught asynchronous SYCL exception " << e.code() << ": \""
                             << e.what() << "\" Warning: Only allocation faliures will dealt with!");
    }
  }
};

namespace aliceVision {
namespace depthMap {

void computeOnMultiDevices(const std::vector<int>& cams, IDeviceJob& devicejob, int nbDevicesToUse)
{
    // aspect_selector finds the "best" device that supports the features we need
    // sycl::detail::select_devices is an AdaptiveCpp unique but it's functionality is easy to replicate should we need to
    // usm_device_allocations and usm_host_allocations should be supported by all AdaptiveCpp backends
    // half support is still a wip but should work fine
#if defined(TSIM_REFINE_USE_HALF) || defined (ALICEVISION_DEPTHMAP_TEXTURE_USE_HALF)
    const std::vector<sycl::device> devices = sycl::detail::select_devices(sycl::multi_device_selector(sycl::aspect_selector({
                //sycl::aspect::fp16, currently returns false regardless because it's still under development
                sycl::aspect::usm_device_allocations,
                sycl::aspect::usm_host_allocations
            })));
#else
    const std::vector<sycl::device> devices = sycl::detail::select_devices(sycl::multi_device_selector(sycl::aspect_selector({
                sycl::aspect::usm_device_allocations,
                sycl::aspect::usm_host_allocations
            })));
#endif

    for (sycl::device d : devices)
            ALICEVISION_LOG_DEBUG("Found possible device: " << d.get_info<sycl::info::device::name>());

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
        const sycl::device& device = devices[0];
        ALICEVISION_LOG_INFO("Using device " << device.get_info<sycl::info::device::name>());
        sycl::queue queue = sycl::queue(device, async_handler_object, {sycl::property::queue::in_order()});
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

        for (int i = 0; i<nbThreads; i++)
        {
            const sycl::device& device = devices.at(i);

            // functional if perhaps crude
            // note that AdaptiveCPP's OpenMP CPU backend has max_clock_frequency 0, which means we will only use it if it's the only device available. Preliminary testing shows that trying to use it alongside other devices just slows everything down
            const float deviceCapacity = device.get_info<sycl::info::device::max_compute_units>()*device.get_info<sycl::info::device::max_clock_frequency>();

            ALICEVISION_LOG_DEBUG("Device " << device.get_info<sycl::info::device::name>()
                                  << " has relative power measure of " << deviceCapacity
                                  << " with " << device.get_info<sycl::info::device::max_compute_units>() << " CUs"
                                  << " at " << device.get_info<sycl::info::device::max_clock_frequency>() << "MHz");

            totalCapacity += deviceCapacity;
            deviceCapacities.at(i) = deviceCapacity;
        }
        ALICEVISION_LOG_DEBUG("Total relative capacity selected for computation is " << totalCapacity);

        for (int i = 0; i<nbThreads; i++)
        {
            auto start = cams.begin() + lround(allocatedCapacity/totalCapacity*cams.size());
            allocatedCapacity += deviceCapacities.at(i);
            auto end = (i == nbThreads - 1) ? cams.end() : cams.begin() + lround(allocatedCapacity/totalCapacity*cams.size()); // end refers to the past-the-end element, no need to subtract one
            deviceCams.at(i) = std::vector<int>(start,end);
        }

        // backup max threads to keep potentially previously set value
        int previous_count_threads = omp_get_max_threads();

#pragma omp parallel default(shared) num_threads(nbThreads)
        {
            const int cpuThreadId = omp_get_thread_num();
            const int deviceId = cpuThreadId % nbThreads;
            const std::vector<int>& camsToCompute = deviceCams.at(deviceId);
            const sycl::device& device = devices.at(deviceId);
            if(camsToCompute.size()>0) // Check to make sure we have been assigned at least one image
            {
                sycl::queue queue = sycl::queue(device, async_handler_object, {sycl::property::queue::in_order()}); // AdaptiveCpp reccomends in-order queues for performance, they also make code cleaner

                ALICEVISION_LOG_INFO("CPU thread " << cpuThreadId << " (of " << nbThreads << ") uses device " << deviceId << " with name " << device.get_info<sycl::info::device::name>());
                ALICEVISION_LOG_DEBUG("Device " << deviceId << " with name " << device.get_info<sycl::info::device::name>() << " has work share " << deviceCapacities.at(deviceId)/totalCapacity << " or " << camsToCompute.size() << " (of " << cams.size() << ", factor " << float(camsToCompute.size())/float(cams.size()) <<") cameras");

                devicejob.compute(queue, camsToCompute);
            }
            else
                ALICEVISION_LOG_INFO("CPU thread " << cpuThreadId << " (of " << nbThreads << ") is using device " << deviceId << " with name " << device.get_info<sycl::info::device::name>() << " whic has been evaluated to weak to be helpful, and will not be executing any work");
        }
        omp_set_num_threads(previous_count_threads);
    }
}

}  // namespace depthMap
}  // namespace aliceVision
