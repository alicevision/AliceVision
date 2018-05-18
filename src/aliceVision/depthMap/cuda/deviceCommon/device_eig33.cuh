// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

namespace aliceVision {
namespace depthMap {

struct cuda_stat3d
{
    double xsum;
    double ysum;
    double zsum;
    double xxsum;
    double yysum;
    double zzsum;
    double xysum;
    double xzsum;
    double yzsum;
    double count;

    __device__ cuda_stat3d();

    __device__ void update(const float3& p, const float& w);

    __device__ bool computePlaneByPCA(float3& p, float3& n);

private:
    __device__ void getEigenVectorsDesc(float3& cg, float3& v1, float3& v2, float3& v3, float& d1, float& d2, float& d3);
};

} // namespace depthMap
} // namespace aliceVision
