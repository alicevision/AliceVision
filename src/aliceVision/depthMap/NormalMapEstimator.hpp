// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/mvsUtils/MultiViewParams.hpp>
#include <aliceVision/depthMap/computeOnMultiGPUs.hpp>

#include <vector>

namespace aliceVision {
namespace depthMap {

/**
 * @brief Normal Map Estimator
 * @brief Wrap normal maps estimation computation.
 * @note Allows muli-GPUs computation (interface IGPUJob)
 */
class NormalMapEstimator : public IGPUJob
{
public:

    /**
     * @brief Normal Map Estimator constructor.
     * @param[in] mp the multi-view parameters
     */
    NormalMapEstimator(const mvsUtils::MultiViewParams& mp);

    // no copy constructor
    NormalMapEstimator(NormalMapEstimator const&) = delete;

    // no copy operator
    void operator=(NormalMapEstimator const&) = delete;

    // destructor
    ~NormalMapEstimator() = default;

    /**
     * @brief Compute normal maps of the given cameras.
     * @param[in] cudaDeviceId the CUDA device id
     * @param[in] cams the list of cameras
     */
    void compute(int cudaDeviceId, const std::vector<int>& cams) override;

private:

    // private members

    const mvsUtils::MultiViewParams& _mp;      //< multi-view parameters
};

} // namespace depthMap
} // namespace aliceVision
