// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/depthMap/cuda/host/memory.hpp>

namespace aliceVision {
namespace depthMap {

/**
 * @brief In-place color conversion from RGB into CIELAB using CUDA.
 * @param[in, out] inout_img_dmp the input RGB buffer, the output LAB buffer in device memory
 * @param[in] stream the CUDA stream for gpu execution
 */
extern void cuda_rgb2lab(CudaDeviceMemoryPitched<CudaRGBA, 2>& inout_img_dmp, cudaStream_t stream);

}  // namespace depthMap
}  // namespace aliceVision
