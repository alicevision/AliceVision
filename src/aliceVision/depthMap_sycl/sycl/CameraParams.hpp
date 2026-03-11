// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <sycl/sycl.hpp>

#include <aliceVision/mvsUtils/MultiViewParams.hpp>
#include <aliceVision/depthMap_sycl/sycl/matrix.hpp>

namespace aliceVision {
namespace depthMap_sycl {

struct CameraParams
{
    sycl::marray<float, 12> P;
    sycl::marray<float, 9> iP;
    sycl::marray<float, 9> R;
    sycl::marray<float, 9> iR;
    sycl::marray<float, 9> K;
    sycl::marray<float, 9> iK;
    sycl::float3 C;
    sycl::float3 XVect;
    sycl::float3 YVect;
    sycl::float3 ZVect;
};

inline CameraParams getCameraParameters(int camId, int downscale, const mvsUtils::MultiViewParams& mp)
{
    Matrix3x3 scaleM;
    scaleM.m11 = 1.0 / float(downscale);
    scaleM.m12 = 0.0;
    scaleM.m13 = 0.0;
    scaleM.m21 = 0.0;
    scaleM.m22 = 1.0 / float(downscale);
    scaleM.m23 = 0.0;
    scaleM.m31 = 0.0;
    scaleM.m32 = 0.0;
    scaleM.m33 = 1.0;

    Matrix3x3 K = scaleM * mp.KArr[camId];
    Matrix3x3 iK = K.inverse();
    Matrix3x4 P = K * (mp.RArr[camId] | (Point3d(0.0, 0.0, 0.0) - mp.RArr[camId] * mp.CArr[camId]));
    Matrix3x3 iP = mp.iRArr[camId] * iK;

    CameraParams params;

    params.C.x() = mp.CArr[camId].x;
    params.C.y() = mp.CArr[camId].y;
    params.C.z() = mp.CArr[camId].z;

    params.P[0] = P.m11;
    params.P[1] = P.m21;
    params.P[2] = P.m31;
    params.P[3] = P.m12;
    params.P[4] = P.m22;
    params.P[5] = P.m32;
    params.P[6] = P.m13;
    params.P[7] = P.m23;
    params.P[8] = P.m33;
    params.P[9] = P.m14;
    params.P[10] = P.m24;
    params.P[11] = P.m34;

    params.iP[0] = iP.m11;
    params.iP[1] = iP.m21;
    params.iP[2] = iP.m31;
    params.iP[3] = iP.m12;
    params.iP[4] = iP.m22;
    params.iP[5] = iP.m32;
    params.iP[6] = iP.m13;
    params.iP[7] = iP.m23;
    params.iP[8] = iP.m33;

    params.R[0] = mp.RArr[camId].m11;
    params.R[1] = mp.RArr[camId].m21;
    params.R[2] = mp.RArr[camId].m31;
    params.R[3] = mp.RArr[camId].m12;
    params.R[4] = mp.RArr[camId].m22;
    params.R[5] = mp.RArr[camId].m32;
    params.R[6] = mp.RArr[camId].m13;
    params.R[7] = mp.RArr[camId].m23;
    params.R[8] = mp.RArr[camId].m33;

    params.iR[0] = mp.iRArr[camId].m11;
    params.iR[1] = mp.iRArr[camId].m21;
    params.iR[2] = mp.iRArr[camId].m31;
    params.iR[3] = mp.iRArr[camId].m12;
    params.iR[4] = mp.iRArr[camId].m22;
    params.iR[5] = mp.iRArr[camId].m32;
    params.iR[6] = mp.iRArr[camId].m13;
    params.iR[7] = mp.iRArr[camId].m23;
    params.iR[8] = mp.iRArr[camId].m33;

    params.K[0] = K.m11;
    params.K[1] = K.m21;
    params.K[2] = K.m31;
    params.K[3] = K.m12;
    params.K[4] = K.m22;
    params.K[5] = K.m32;
    params.K[6] = K.m13;
    params.K[7] = K.m23;
    params.K[8] = K.m33;

    params.iK[0] = iK.m11;
    params.iK[1] = iK.m21;
    params.iK[2] = iK.m31;
    params.iK[3] = iK.m12;
    params.iK[4] = iK.m22;
    params.iK[5] = iK.m32;
    params.iK[6] = iK.m13;
    params.iK[7] = iK.m23;
    params.iK[8] = iK.m33;

    params.XVect = sycl::normalize(M3x3mulV3(params.iR, sycl::float3(1.f, 0.f, 0.f)));

    params.YVect = sycl::normalize(M3x3mulV3(params.iR, sycl::float3(0.f, 1.f, 0.f)));

    params.ZVect = sycl::normalize(M3x3mulV3(params.iR, sycl::float3(0.f, 0.f, 1.f)));

    return params;
}

}  // namespace depthMap_sycl
}  // namespace aliceVision
