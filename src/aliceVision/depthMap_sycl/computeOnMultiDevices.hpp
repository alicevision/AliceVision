// This file is part of the extension to AliceVision project.
// Copyright (c) 2018 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/mvsUtils/MultiViewParams.hpp>
#include <sycl/sycl.hpp>

namespace aliceVision {
namespace depthMap_sycl {

/**
 * @class IDeviceJob
 * @brief Interface for multi-GPUs computation.
 */
class IDeviceJob
{
  public:
    /**
     * @brief Perform computation from the given cameras.
     * @param[in] queue the SYCL queue on which to execute
     * @param[in] cams the list of cameras
     */
    virtual void compute(const sycl::device& device, const std::vector<int>& cams) = 0;
};

/**
 * @brief Perform computation from the given cameras on multiple SYCL devices.
 * @param[in] cams the given list of cameras
 * @param[in,out] devicejob the object that wraps computation (should use IDeviceJob interface)
 * @param[in] nbDevicesToUse the number of devices to use
 */
void computeOnMultiDevices(std::vector<int> cams, IDeviceJob& devicejob, int nbDevicesToUse);

}  // namespace depthMap_sycl
}  // namespace aliceVision
