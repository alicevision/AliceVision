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

    __device__ void computeSimSub(simStat& ss)
    {
        // NCC

        float avx = ss.xsum / ss.count;
        float avy = ss.ysum / ss.count;
        float dx = xsum - 2.0f * avx * xsum + count * avx;
        float dy = ysum - 2.0f * avy * ysum + count * avy;
        float d = dx * dy;
        float u = xysum - avx * ysum - avy * xsum + count * avx * avy;

        if(fabs(d) > 0.0f)
        {
            sim = u / sqrtf(d);
            sim = 0.0f - sim;
        }
        else
        {
            sim = 1.0f;
        };
    }

    __device__ float getVarianceX() { return (xxsum / count - (xsum * xsum) / (count * count)); }

    __device__ float getVarianceY() { return (yysum / count - (ysum * ysum) / (count * count)); }

    __device__ float getVarianceXY() { return (xysum / count - (xsum * ysum) / (count * count)); }

    /*
            __device__ void computeSim()
            {
                    //NCC
                    float d = (xxsum - ( (xsum*xsum)/count ) ) * (yysum - ( (ysum*ysum)/count ));
                    if (fabs(d)>0.0f)
                    {
                            sim = ( ( xysum - ( (xsum*ysum)/count ) ) ) /  sqrtf( d );
                            sim = 0.0f - sim;
                    }else
                    {
                            sim = 1.0f;
                    };

                    sim = fmaxf(sim,-1.0f);
                    sim = fminf(sim,1.0f);
            };
    */
    __device__ void computeSim()
    {
        // NCC
        float d = getVarianceX() * getVarianceY();
        if(fabs(d) > 0.0f)
        {
            sim = getVarianceXY() / sqrtf(d);
            sim = 0.0f - sim;
        }
        else
        {
            sim = 1.0f;
        };

        sim = fmaxf(sim, -1.0f);
        sim = fminf(sim, 1.0f);
    }

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
    __device__ void computeWSim()
    {
        // NCC
        sim = getVarianceXYW() / sqrtf(getVarianceXW() * getVarianceYW());
        sim = isinf(sim) ? 1.0f : 0.0f - sim;
        sim = fmaxf(fminf(sim, 1.0f), -1.0f);
    }

    __device__ void update(const float2 g)
    {
        count += 1.0f;
        xsum += g.x;
        ysum += g.y;
        xxsum += g.x * g.x;
        yysum += g.y * g.y;
        xysum += g.x * g.y;
    }

    __device__ void update(const float2 g, float w)
    {
        wsum += w;
        count += 1.0f;
        xsum += w * g.x;
        ysum += w * g.y;
        xxsum += w * g.x * g.x;
        yysum += w * g.y * g.y;
        xysum += w * g.x * g.y;
    }

    __device__ void update(const float gx, const float gy)
    {
        count += 1.0f;
        xsum += gx;
        ysum += gy;
        xxsum += gx * gx;
        yysum += gy * gy;
        xysum += gx * gy;
    }

    __device__ void update(const float gx, const float gy, float w)
    {
        wsum += w;
        count += 1.0f;
        xsum += w * gx;
        ysum += w * gy;
        xxsum += w * gx * gx;
        yysum += w * gy * gy;
        xysum += w * gx * gy;
    }

    __device__ void update(const uchar4 c1, const uchar4 c2)
    {
        float2 g;
        g.x = (float)c1.x / 255.0f;
        g.y = (float)c2.x / 255.0f;
        update(g);

        g.x = (float)c1.y / 255.0f;
        g.y = (float)c2.y / 255.0f;
        update(g);

        g.x = (float)c1.z / 255.0f;
        g.y = (float)c2.z / 255.0f;
        update(g);
    }

    __device__ void update(const float3 c1, const float3 c2)
    {
        float2 g;
        g.x = c1.x;
        g.y = c2.x;
        update(g);

        g.x = c1.y;
        g.y = c2.y;
        update(g);

        g.x = c1.z;
        g.y = c2.z;
        update(g);
    }
};

} // namespace depthMap
} // namespace aliceVision
