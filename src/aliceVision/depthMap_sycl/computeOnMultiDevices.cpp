// This file is part of the extension to AliceVision project.
// Copyright (c) 2018 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "computeOnMultiDevices.hpp"

#include <aliceVision/alicevision_omp.hpp>
#include <aliceVision/depthMap_sycl/sycl/sycl.hpp>

// Needed for checking device caracteristics
#include <aliceVision/depthMap_sycl/sycl/planeSweeping/similarity.hpp>
#include <aliceVision/depthMap_sycl/sycl/memory.hpp>

namespace aliceVision {
namespace depthMap_sycl {

void computeOnMultiDevices(std::vector<int> cams, IDeviceJob& devicejob, int nbDevicesToUse)
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

    for (const sycl::device& d : devices)
        ALICEVISION_LOG_DEBUG("Found possible device: " << d.get_info<sycl::info::device::name>());

    const int nbDevices = devices.size();
    if (nbDevices == 0) [[unlikely]] ALICEVISION_LOG_ERROR("Could not find a suitable device for computation."); // shouldn't ever happen

    ALICEVISION_LOG_INFO("Number of compute devices: " << nbDevices);

    if (nbDevicesToUse == 0)
    {
        // Use the user specified limit on the number of Devices to use
        nbDevicesToUse = nbDevices == 2 ? 1 /* exactly two devices means one host CPU and one accelerator device */ : nbDevices;
    }

    if (nbDevicesToUse == 1)
    {
        const sycl::device& device = devices[0];
        ALICEVISION_LOG_INFO("Using device " << device.get_info<sycl::info::device::name>());
        devicejob.compute(device, cams);
    }
    else
    {
        // lock to protect list of cameras
        std::mutex cams_lock{};

        // atomic global finished counter
        std::atomic<uint> global_finished = 0;
#pragma omp parallel num_threads(nbDevicesToUse) default(shared)
        {
            const int cpuThreadId = omp_get_thread_num();
            const int deviceId = cpuThreadId % devices.size();
            const sycl::device& device = devices.at(deviceId);

            if(device.is_cpu())
               ALICEVISION_LOG_DEBUG("Ignoring host cpu device as it needs to handle i/o and dispatch");
            else
            {
                ALICEVISION_LOG_DEBUG("CPU thread " << cpuThreadId << " (of " << nbDevicesToUse << ") uses device " << deviceId << " with name " << device.get_info<sycl::info::device::name>());

                uint thread_finished = 0;

                cams_lock.lock();
                while(cams.size() > 0)
                {
                    if(thread_finished != 0 && thread_finished * cams.size() / global_finished == 0) // this comparison is protected by the mutex
                    {
                        // we are not going to benefit overall computation time by taking another camera, so exit
                        global_finished -= thread_finished; // Remove our contributions as we will no longer be doing any work
                        break;
                    }
                    int cam = cams.back();
                    cams.pop_back();
                    cams_lock.unlock();

                    devicejob.compute(device, std::vector{cam});

                    thread_finished++;
                    global_finished++;

                    cams_lock.lock();
                }
                cams_lock.unlock();
            }
        }
    }
}

}  // namespace depthMap_sycl
}  // namespace aliceVision
