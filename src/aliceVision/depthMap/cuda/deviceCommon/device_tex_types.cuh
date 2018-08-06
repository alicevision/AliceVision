// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <cuda_runtime.h>

/* This struct exists because texture objects have very different properties
 * and compile-time checking is very desirable. A typedef is only an alias
 * and does not work.
 */
template<typename T>
struct NormLinearTex
{
    cudaTextureObject_t obj;
};

template<typename T>
struct PointTex
{
    cudaTextureObject_t obj;
};

