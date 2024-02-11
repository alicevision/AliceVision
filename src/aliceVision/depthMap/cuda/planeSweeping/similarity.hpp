// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#define TSIM_REFINE_USE_HALF

#ifdef TSIM_REFINE_USE_HALF
    #define CUDA_NO_HALF
    #include <cuda_fp16.h>
#endif

namespace aliceVision {
namespace depthMap {

/*
 * @note TSim is the similarity type for volume in device memory.
 * @note TSimAcc is the similarity accumulation type for volume in device memory.
 * @note TSimRefine is the similarity type for volume refinement in device memory.
 */

#ifdef TSIM_USE_FLOAT
using TSim = float;
using TSimAcc = float;
#else
using TSim = unsigned char;
using TSimAcc = unsigned int;  // TSimAcc is the similarity accumulation type
#endif

#ifdef TSIM_REFINE_USE_HALF
using TSimRefine = __half;
#else
using TSimRefine = float;
#endif

}  // namespace depthMap
}  // namespace aliceVision
