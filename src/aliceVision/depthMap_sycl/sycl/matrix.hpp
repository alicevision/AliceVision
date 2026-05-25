// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/depthMap_sycl/sycl/sycl.hpp>

// mn MATRIX ADDRESSING: mxy = x*n+y (x-row,y-col), (m-number of rows, n-number of columns)

namespace aliceVision {
namespace depthMap_sycl {

inline sycl::float3 M3x3mulV3(const sycl::marray<float, 9>& M3x3, const sycl::float3& V)
{
    return sycl::float3(M3x3[0] * V.x() + M3x3[3] * V.y() + M3x3[6] * V.z(),
                        M3x3[1] * V.x() + M3x3[4] * V.y() + M3x3[7] * V.z(),
                        M3x3[2] * V.x() + M3x3[5] * V.y() + M3x3[8] * V.z());
}

inline sycl::float3 M3x3mulV2(const sycl::marray<float, 9>& M3x3, const sycl::float2& V)
{
    return sycl::float3(M3x3[0] * V.x() + M3x3[3] * V.y() + M3x3[6],
                        M3x3[1] * V.x() + M3x3[4] * V.y() + M3x3[7],
                        M3x3[2] * V.x() + M3x3[5] * V.y() + M3x3[8]);
}

inline sycl::float3 M3x4mulV3(const sycl::marray<float, 12>& M3x4, const sycl::float3& V)
{
    return sycl::float3(M3x4[0] * V.x() + M3x4[3] * V.y() + M3x4[6] * V.z() + M3x4[9],
                        M3x4[1] * V.x() + M3x4[4] * V.y() + M3x4[7] * V.z() + M3x4[10],
                        M3x4[2] * V.x() + M3x4[5] * V.y() + M3x4[8] * V.z() + M3x4[11]);
}

inline sycl::float2 V2M3x3mulV2(sycl::marray<float, 9>& M3x3, sycl::float2& V)
{
    const float d = M3x3[2] * V.x() + M3x3[5] * V.y() + M3x3[8];
    return sycl::float2(M3x3[0] * V.x() + M3x3[3] * V.y() + M3x3[6], M3x3[1] * V.x() + M3x3[4] * V.y() + M3x3[7])/d;
}


inline sycl::float2 project3DPoint(const sycl::marray<float, 12>& M3x4, const sycl::float3& V)
{
    // without optimization
    // const sycl::float3& p = M3x4mulV3(M3x4, V);
    // return make_float2(p.x / p.z, p.y / p.z);

    const sycl::float3 p = M3x4mulV3(M3x4, V);
    return sycl::half_precision::divide(sycl::float2(p.x(), p.y()), sycl::float2(p.z()));
}

static constexpr void M3x3mulM3x3(sycl::marray<float, 9>& O3x3, const sycl::marray<float, 9>& A3x3, const sycl::marray<float, 9>& B3x3)
{
    O3x3[0] = A3x3[0] * B3x3[0] + A3x3[3] * B3x3[1] + A3x3[6] * B3x3[2];
    O3x3[3] = A3x3[0] * B3x3[3] + A3x3[3] * B3x3[4] + A3x3[6] * B3x3[5];
    O3x3[6] = A3x3[0] * B3x3[6] + A3x3[3] * B3x3[7] + A3x3[6] * B3x3[8];

    O3x3[1] = A3x3[1] * B3x3[0] + A3x3[4] * B3x3[1] + A3x3[7] * B3x3[2];
    O3x3[4] = A3x3[1] * B3x3[3] + A3x3[4] * B3x3[4] + A3x3[7] * B3x3[5];
    O3x3[7] = A3x3[1] * B3x3[6] + A3x3[4] * B3x3[7] + A3x3[7] * B3x3[8];

    O3x3[2] = A3x3[2] * B3x3[0] + A3x3[5] * B3x3[1] + A3x3[8] * B3x3[2];
    O3x3[5] = A3x3[2] * B3x3[3] + A3x3[5] * B3x3[4] + A3x3[8] * B3x3[5];
    O3x3[8] = A3x3[2] * B3x3[6] + A3x3[5] * B3x3[7] + A3x3[8] * B3x3[8];
}

static constexpr void M3x3transpose(sycl::marray<float, 9>& O3x3, const sycl::marray<float, 9>& A3x3)
{
    O3x3[0] = A3x3[0];
    O3x3[1] = A3x3[3];
    O3x3[2] = A3x3[6];
    O3x3[3] = A3x3[1];
    O3x3[4] = A3x3[4];
    O3x3[5] = A3x3[7];
    O3x3[6] = A3x3[2];
    O3x3[7] = A3x3[5];
    O3x3[8] = A3x3[8];
}

static constexpr void outerMultiply(sycl::marray<float, 9>& O3x3, const sycl::float3& a, const sycl::float3& b)
{
    O3x3[0] = a.x() * b.x();
    O3x3[3] = a.x() * b.y();
    O3x3[6] = a.x() * b.z();
    O3x3[1] = a.y() * b.x();
    O3x3[4] = a.y() * b.y();
    O3x3[7] = a.y() * b.z();
    O3x3[2] = a.z() * b.x();
    O3x3[5] = a.z() * b.y();
    O3x3[8] = a.z() * b.z();
}

inline sycl::float3 linePlaneIntersect(const sycl::float3& linePoint,
                                       const sycl::float3& lineVect,
                                       const sycl::float3& planePoint,
                                       const sycl::float3& planeNormal)
{
    const float k = (sycl::dot(planePoint, planeNormal) - sycl::dot(planeNormal, linePoint)) / sycl::dot(planeNormal, lineVect);
    return linePoint + lineVect * k;
}

inline sycl::float3 closestPointOnPlaneToPoint(const sycl::float3& point, const sycl::float3& planePoint, const sycl::float3& planeNormalNormalized)
{
    return point - planeNormalNormalized * sycl::dot(planeNormalNormalized, point - planePoint);
}

inline sycl::float3 closestPointToLine3D(const sycl::float3& point, const sycl::float3& linePoint, const sycl::float3& lineVectNormalized)
{
    return linePoint + lineVectNormalized * sycl::dot(lineVectNormalized, point - linePoint);
}

inline float pointLineDistance3D(const sycl::float3& point, const sycl::float3& linePoint, const sycl::float3& lineVectNormalized)
{
    return sycl::length(sycl::cross(lineVectNormalized, linePoint - point));
}

// v1,v2 dot not have to be normalized
inline float angleBetwV1andV2(const sycl::float3& iV1, const sycl::float3& iV2)
{
    const sycl::float3 V1 = sycl::normalize(iV1);

    const sycl::float3 V2 = sycl::normalize(iV2);

    return sycl::fabs(sycl::acos(sycl::dot(V1,V2)) / (std::numbers::pi / 180.0f));
}

inline float angleBetwABandAC(const sycl::float3& A, const sycl::float3& B, const sycl::float3& C)
{
    const sycl::float3 V1 = sycl::normalize(B - A);
    const sycl::float3 V2 = sycl::normalize(C - A);

    const double x = double(sycl::dot(V1,V2));
    double a = sycl::acos(x);
    a = sycl::isinf(a) ? 0.0 : a;
    return float(sycl::fabs(a) / (std::numbers::pi / 180.0));
}

/**
 * @brief Calculate the line segment PaPb that is the shortest route between two lines p1-p2 and p3-p4.
 *        Calculate also the values of mua and mub where:
 *          -> pa = p1 + mua (p2 - p1)
 *          -> pb = p3 + mub (p4 - p3)
 *
 * @note This a simple conversion to MATLAB of the C code posted by Paul Bourke at:
 *       http://astronomy.swin.edu.au/~pbourke/geometry/lineline3d/
 *       The author of this all too imperfect translation is Cristian Dima (csd@cmu.edu).
 *
 * @see https://web.archive.org/web/20060422045048/http://astronomy.swin.edu.au/~pbourke/geometry/lineline3d/
 */
// C to MATLAB to CUDA C++ to SYCL C++. What a story. Let's hope this remains decently correct/performant
inline sycl::float3 lineLineIntersect(float& k,
                                      float& l,
                                      sycl::float3& lli1,
                                      sycl::float3& lli2,
                                      const sycl::float3& p1,
                                      const sycl::float3& p2,
                                      const sycl::float3& p3,
                                      const sycl::float3& p4)
{
    // float d1343, d4321, d1321, d4343, d2121, denom, numer, fract;
    //sycl::float3 p13, p43, p21, pa, pb, S;
    // sycl::float2 muab;

    const sycl::float3 p13 = p1 - p3;
    const sycl::float3 p43 = p4 - p3;

    /*
    if ((abs(p43[0])  < eps) & ...
        (abs(p43[1])  < eps) & ...
        (abs(p43[2])  < eps))
      error('Could not compute LineLineIntersect!');
    end
    */

    const sycl::float3 p21 = p2 - p1;

    /*
    if ((abs(p21[0])  < eps) & ...
        (abs(p21[1])  < eps) & ...
        (abs(p21[2])  < eps))
      error('Could not compute LineLineIntersect!');
    end
    */

    const float d1343 = sycl::dot(p13, p43);
    const float d4321 = sycl::dot(p43, p21);
    const float d1321 = sycl::dot(p13, p21);
    const float d4343 = sycl::dot(p43, p43); // Equivalent to length squared, but SYCL doesn't have a dedicated function
    const float d2121 = sycl::dot(p21, p21);

    const float denom = d2121 * d4343 - d4321 * d4321;

    /*
    if (abs(denom) < eps)
      error('Could not compute LineLineIntersect!');
    end
     */

    const float numer = d1343 * d4321 - d1321 * d4343;
    const float fract = numer / denom;

    //const sycl::float2 muab = sycl::float2(fract, (d1343 + d4321 * fract) / d4343);
    const float mub = (d4321 * fract + d1343) / d4343;

    const sycl::float3 pa = p1 + p21 * fract; //muab.x
    const sycl::float3 pb = p3 + p43 * mub; //muab.y

    const sycl::float3 S = (pa + pb) * 0.5;

    k = fract;
    l = mub;

    lli1 = pa;
    lli2 = pb;

    return S;
}

/**
 * @brief Sigmoid function filtering
 * @note f(x) = min + (max-min) * \frac{1}{1 + e^{10 * (x - mid) / width}}
 * @see https://www.desmos.com/calculator/1qvampwbyx
 */
static constexpr float sigmoid(float zeroVal, float endVal, float sigwidth, float sigMid, float xval)
{
    return zeroVal + (endVal - zeroVal) * (1.0f / (1.0f + sycl::exp((xval - sigMid) / sigwidth * 10.f)));
}

/**
 * @brief Sigmoid function filtering
 * @note f(x) = min + (max-min) * \frac{1}{1 + e^{10 * (mid - x) / width}}
 */
static constexpr float sigmoid2(float zeroVal, float endVal, float sigwidth, float sigMid, float xval)
{
    return zeroVal + (endVal - zeroVal) * (1.0f / (1.0f + sycl::exp((sigMid - xval) / sigwidth * 10.f)));
}

/**
 * @brief fmin with more than two values
 */
template<typename T>
static constexpr T multi_fmin(T a, T b) { return sycl::fmin(a, b); }

template<typename T>
static constexpr T multi_fmin(T a, T b, T c, T d) { return sycl::fmin(sycl::fmin(a, b), sycl::fmin(c, d)); }

template<typename T, typename... Rest>
static constexpr T multi_fmin(T a, T b, Rest... rest) { return multi_fmin(sycl::fmin(a, b), rest...); }

} // namespace depthMap_sycl
} // namespace aliceVision
