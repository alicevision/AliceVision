// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

namespace aliceVision {
namespace depthMap {

struct simStat
{
    float xsum;
    float ysum;
    float xxsum;
    float yysum;
    float xysum;
    float count;
    float wsum;
    float sim;

    __device__ simStat()
    {
        xsum = 0.0f;
        ysum = 0.0f;
        xxsum = 0.0f;
        yysum = 0.0f;
        xysum = 0.0f;
        count = 0.0f;
        wsum = 0.0f;
        sim = 1.0f;
    }

    __device__ simStat& operator=(const simStat& param)
    {
        xxsum = param.xxsum;
        yysum = param.yysum;
        xysum = param.xysum;
        xsum = param.xsum;
        ysum = param.ysum;
        wsum = param.wsum;
        count = param.count;
        return *this;
    }

    __device__ void computeSimSub(simStat& ss);

    __device__ float getVarianceX() { return (xxsum / count - (xsum * xsum) / (count * count)); }

    __device__ float getVarianceY() { return (yysum / count - (ysum * ysum) / (count * count)); }

    __device__ float getVarianceXY() { return (xysum / count - (xsum * ysum) / (count * count)); }

    __device__ void computeSim();

    /**
    * @brief Variance of X
    * formula: sum(w*x*x) / sum(w) - sum(w*x)^2 / sum(w)^2
    */
    __device__ float getVarianceXW()
    {
        return (xxsum - xsum * xsum / wsum) / wsum;
    }

    /**
    * @brief Variance of Y
    * formula: sum(w*y*y) / sum(w) - sum(w*y)^2 / sum(w)^2
    */
    __device__ float getVarianceYW()
    {
        return (yysum - ysum * ysum / wsum) / wsum;
    }

    /**
     * @brief Cross-Correlation between X and Y
     * Formula is: sum(w*x*y) - 2 * sum(w*x)*sum(w*y) / sum(w) + sum(w*x*y)
     * Simplified as: sum(w*x*y) / sum(w) - sum(w*y) * sum(w*x) / sum(w)^2
     */
    __device__ float getVarianceXYW()
    {
        return (xysum - xsum * ysum / wsum) / wsum;
    }

    /**
     * @brief Compute Normalized Cross-Correlation.
     * @return similarity value in range (-1, 0) or 1 if infinity
     */
    __device__ void computeWSim();

    __device__ void update(const float2 g);

    __device__ void update(const float2 g, float w);

    __device__ void update(const float gx, const float gy);

    __device__ void update(const float gx, const float gy, float w);

    __device__ void update(const uchar4 c1, const uchar4 c2);

    __device__ void update(const float3 c1, const float3 c2);
};

} // namespace depthMap
} // namespace aliceVision
