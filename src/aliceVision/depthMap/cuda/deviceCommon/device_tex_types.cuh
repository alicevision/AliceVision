// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <cuda_runtime.h>

template<typename T,cudaTextureFilterMode fMode,cudaTextureReadMode rMode>
struct BaseTex
{
    cudaTextureObject_t obj;
};

/* This struct exists because texture objects have very different properties
 * and compile-time checking is very desirable. A typedef is only an alias
 * and does not work.
 */
using NormLinearTexUchar4 = BaseTex<uchar4,cudaFilterModeLinear,cudaReadModeNormalizedFloat>;

using ElemPointTexUchar4  = BaseTex<uchar4,      cudaFilterModePoint,cudaReadModeElementType>;
using ElemPointTexFloat   = BaseTex<float,       cudaFilterModePoint,cudaReadModeElementType>;
using ElemPointTexUint    = BaseTex<unsigned int,cudaFilterModePoint,cudaReadModeElementType>;
using ElemPointTexInt     = BaseTex<int,         cudaFilterModePoint,cudaReadModeElementType>;


