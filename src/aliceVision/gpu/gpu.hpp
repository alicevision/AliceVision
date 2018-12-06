// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <string>

namespace aliceVision {
namespace gpu {

/**
 * @brief Check if the system support CUDA with the given parameters
 * @param[in] minComputeCapabilityMajor The minimum compute capability major
 * @param[in] minComputeCapabilityMinor The minimum compute capability minor
 * @param[in] minTotalDeviceMemory The minimum device total memory in MB
 * @return True if system support CUDA with the given parameters
 */
bool gpuSupportCUDA(int minComputeCapabilityMajor,
                    int minComputeCapabilityMinor,
                    int minTotalDeviceMemory = 0);


/**
 * @brief gpuInformationCUDA
 * @return string with all CUDA device(s) information
 */
std::string gpuInformationCUDA();

} // namespace gpu
} // namespace aliceVision
