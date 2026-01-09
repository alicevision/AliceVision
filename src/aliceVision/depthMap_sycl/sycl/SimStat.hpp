// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
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

    simStat()
    {
        xsum = 0.0f;
        ysum = 0.0f;
        xxsum = 0.0f;
        yysum = 0.0f;
        xysum = 0.0f;
        count = 0.0f;
        wsum = 0.0f;
    }

    simStat& operator=(const simStat& param)
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

    float computeSimSub(simStat& ss)
    {
        // NCC
        float avx = ss.xsum / ss.count;
        float avy = ss.ysum / ss.count;
        float dx = xsum - 2.0f * avx * xsum + count * avx;
        float dy = ysum - 2.0f * avy * ysum + count * avy;
        float d = dx * dy;
        float u = xysum - avx * ysum - avy * xsum + count * avx * avy;

        float sim = 1.0f;
        if(sycl::fabs(d) > 0.0f)
        {
            sim = -(u / sycl::sqrt(d));
        }
        return sim;
    }

    float getVarianceX() const { return (xxsum / count - (xsum * xsum) / (count * count)); }

    float getVarianceY() const { return (yysum / count - (ysum * ysum) / (count * count)); }

    float getVarianceXY() const { return (xysum / count - (xsum * ysum) / (count * count)); }

    /**
    * @brief Variance of X
    * formula: sum(w*x*x) / sum(w) - sum(w*x)^2 / sum(w)^2
    */
    float getVarianceXW() const
    {
        return (xxsum - xsum * xsum / wsum) / wsum;
    }

    /**
    * @brief Variance of Y
    * formula: sum(w*y*y) / sum(w) - sum(w*y)^2 / sum(w)^2
    */
    float getVarianceYW() const
    {
        return (yysum - ysum * ysum / wsum) / wsum;
    }

    /**
     * @brief Cross-Correlation between X and Y
     * Formula is: sum(w*x*y) - 2 * sum(w*x)*sum(w*y) / sum(w) + sum(w*x*y)
     * Simplified as: sum(w*x*y) / sum(w) - sum(w*y) * sum(w*x) / sum(w)^2
     */
    float getVarianceXYW() const
    {
        return (xysum - xsum * ysum / wsum) / wsum;
    }

    /**
     * @brief Compute Normalized Cross-Correlation.
     * @return similarity value in range (-1, 0) or 1 if infinity
     */
    float computeWSim()
     {
         // NCC

         // without optimization
         // const float rawSim = getVarianceXYW() / sqrtf(getVarianceXW() * getVarianceYW());
         const float rawSim = getVarianceXYW() * sycl::half_precision::rsqrt(getVarianceXW() * getVarianceYW());

         const float sim = sycl::isfinite(rawSim) ? -rawSim : 1.0f;
         // sim = fmaxf(fminf(sim, 1.0f), -1.0f); // clamp
         return sim;
     }

    void update(const sycl::float2& g)
    {
        count += 1.0f;
        xsum += g.x();
        ysum += g.y();
        xxsum += g.x() * g.x();
        yysum += g.y() * g.y();
        xysum += g.x() * g.y();
    }

    void update(const sycl::float2& g, float w)
    {
        wsum += w;
        count += 1.0f;
        xsum += w * g.x();
        ysum += w * g.y();
        xxsum += w * g.x() * g.x();
        yysum += w * g.y() * g.y();
        xysum += w * g.x() * g.y();
    }

    void update(const float gx, const float gy)
    {
        count += 1.0f;
        xsum += gx;
        ysum += gy;
        xxsum += gx * gx;
        yysum += gy * gy;
        xysum += gx * gy;
    }

    void update(const float gx, const float gy, float w)
    {
        wsum += w;
        count += 1.0f;
        xsum += w * gx;
        ysum += w * gy;
        xxsum += w * gx * gx;
        yysum += w * gy * gy;
        xysum += w * gx * gy;
    }

    void update(const sycl::float3& c1, const sycl::float3& c2)
    {
        update(sycl::float2(c1.x(), c2.x()));
        update(sycl::float2(c1.y(), c2.y()));
        update(sycl::float2(c1.z(), c2.z()));
    }
};

} // namespace depthMap
} // namespace aliceVision
