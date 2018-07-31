// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#ifdef __CUDACC__
#include <cuda_runtime.h>
#endif

namespace aliceVision {
namespace depthMap {

struct CameraBaseStruct
{
    float  P[12], iP[9], R[9], iR[9], K[9], iK[9];
#ifdef __CUDACC__
    float3 C;
    float3 XVect;
    float3 YVect;
    float3 ZVect;
#else
    float  C[3];
    float  XVect[3];
    float  YVect[3];
    float  ZVect[3];
#endif
};

} // namespace depthMap
} // namespace aliceVision

