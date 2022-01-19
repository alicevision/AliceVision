// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

namespace aliceVision {
namespace depthMap {

/**
 * @brief Get and log available CUDA devices. 
 * @return the number of CUDA devices
 */
int listCudaDevices();

/**
 * @brief Get the device id currently used for GPU executions.
 * @return current CUDA device id
 */
int getCudaDeviceId();

/**
 * @brief Set the device to use for GPU executions.
 * @param[in] cudaDeviceId the CUDA device id to use
 */
void setCudaDeviceId(int cudaDeviceId);

/**
 * @brief Test if the device id currently used for GPU executions 
 *        is the same as the one given.
 * @param[in] cudaDeviceId the given CUDA device id to test
 */
bool testCudaDeviceId(int cudaDeviceId);

/**
 * @brief Log current CUDA device memory information.
 */
void logDeviceMemoryInfo();

/**
 * @brief Get current CUDA device memory information.
 * @param[out] available the available memory in MB on the current CUDA device
 * @param[out] used the used memory in MB on the current CUDA device
 * @param[out] total the total memory in MB on the current CUDA device
 */
void getDeviceMemoryInfo(double& available, double& used, double& total);

} // namespace depthMap
} // namespace aliceVision

