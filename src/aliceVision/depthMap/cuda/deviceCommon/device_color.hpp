// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/depthMap/cuda/memory.hpp>

namespace aliceVision {
namespace depthMap {

/**
 * @brief In-place color conversion into CIELAB using CUDA.
 * @param[in, out] frame_dmp the camera frame in device memory
 * @param[in] width the frame width
 * @param[in] height the frame height
 * @param[in] stream the CUDA stream for gpu execution
 */
void cuda_rgb2lab(CudaDeviceMemoryPitched<CudaRGBA, 2>& frame_dmp, int width, int height, cudaStream_t stream);

} // namespace depthMap
} // namespace aliceVision

