// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/depthMap_sycl/sycl/buffer.hpp>
#include <aliceVision/depthMap_sycl/sycl/color.hpp>
#include <aliceVision/depthMap_sycl/sycl/matrix.hpp>
#include <aliceVision/depthMap_sycl/sycl/SimStat.hpp>
#include <aliceVision/depthMap_sycl/sycl/CameraParams.hpp>
#include <aliceVision/depthMap_sycl/sycl/PatchPattern.hpp>

namespace aliceVision {
namespace depthMap_sycl {

struct Patch
{
    sycl::float3 p; //< 3d point
    sycl::float3 n; //< normal
    sycl::float3 x; //< x axis
    sycl::float3 y; //< y axis
    float d;  //< pixel size
};

inline void rotPointAroundVect(sycl::float3& point, const sycl::float3& vect, const int& angle)
{
    double ux, uy, uz, vx, vy, vz, wx, wy, wz, sa, ca, x, y, z, u, v, w;

    const double size = sycl::length(point);
    x = point.x() / size;
    y = point.y() / size;
    z = point.z() / size;
    u = vect.x();
    v = vect.y();
    w = vect.z();

    /*Rotate the point (x,y,z) around the vector (u,v,w)*/
    ux = u * x;
    uy = u * y;
    uz = u * z;
    vx = v * x;
    vy = v * y;
    vz = v * z;
    wx = w * x;
    wy = w * y;
    wz = w * z;
    sa = sycl::sin((double)angle * (std::numbers::pi / 180.0));
    ca = sycl::cos((double)angle * (std::numbers::pi / 180.0));
    x = u * (ux + vy + wz) + (x * (v * v + w * w) - u * (vy + wz)) * ca + (-wy + vz) * sa;
    y = v * (ux + vy + wz) + (y * (u * u + w * w) - v * (ux + wz)) * ca + (wx - uz) * sa;
    z = w * (ux + vy + wz) + (z * (u * u + v * v) - w * (ux + vy)) * ca + (-vx + uy) * sa;

    point[0] = x;
    point[1] = y;
    point[2] = z;
    point = sycl::normalize(point) * size;
}

inline static constexpr void rotatePatch(Patch& ptch, const int& rx, const int& ry)
{
    // rotate patch around x axis by angle rx
    rotPointAroundVect(ptch.n, ptch.x, rx);
    rotPointAroundVect(ptch.y, ptch.x, rx);

    // rotate new patch around y axis by angle ry
    rotPointAroundVect(ptch.n, ptch.y, ry);
    rotPointAroundVect(ptch.x, ptch.y, ry);
}

inline void movePatch(Patch& ptch, const int& pt)
{
    // sycl::float3 v = ptch.p-rC;
    // normalize(v);
    const float d = ptch.d * static_cast<float>(pt);
    ptch.p += ptch.n * d;
}

inline static constexpr void computeRotCS(sycl::float3& xax, sycl::float3& yax, const sycl::float3& n)
{
    xax.x() = -n.y() + n.z(); // get any cross product
    xax.y() = +n.x() + n.z();
    xax.z() = -n.x() - n.y();
    if(sycl::fabs(xax.x()) < 0.0000001f && sycl::fabs(xax.y()) < 0.0000001f && sycl::fabs(xax.z()) < 0.0000001f)
    {
        xax.x() = -n.y() - n.z(); // get any cross product (complementar)
        xax.y() = +n.x() - n.z();
        xax.z() = +n.x() + n.y();
    };
    xax = sycl::normalize(xax);
    yax = sycl::cross(n, xax);
}

inline void computeRotCSEpip(Patch& ptch,
                             const CameraParams& rcDeviceCamParams,
                             const CameraParams& tcDeviceCamParams)
{
    // Vector from the reference camera to the 3d point
    const sycl::float3 v1 = sycl::normalize(rcDeviceCamParams.C - ptch.p);
    // Vector from the target camera to the 3d point
    const sycl::float3 v2 = sycl::normalize(tcDeviceCamParams.C - ptch.p);

    // y has to be orthogonal to the epipolar plane
    // n has to be on the epipolar plane
    // x has to be on the epipolar plane

    ptch.y = sycl::normalize(sycl::cross(v1, v2)); // TODO: v1 & v2 are already normalized

    ptch.n = sycl::normalize(v1 + v2); // ((v1 + v2) / 2.0f); // IMPORTANT !! // TODO: v1 & v2 are already normalized
    // ptch.n = sg_s_r.ZVect; //IMPORTANT !! // Porting note: apparently not? was already commented out

    ptch.x = /*sycl::normalize(*/sycl::cross(ptch.y, ptch.n); // y and n have to be orthogonal and normalized, so their cross product is already normalized
}

inline float computePixSize(const CameraParams& deviceCamParams, const sycl::float3& p)
{
    const sycl::float2 rp = project3DPoint(deviceCamParams.P, p);
    const sycl::float2 rp1 = rp + sycl::float2(1.0f, 0.0f);

    sycl::float3 refvect = sycl::normalize(M3x3mulV2(deviceCamParams.iP, rp1));
    return pointLineDistance3D(p, deviceCamParams.C, refvect);
}

inline void getPixelFor3DPoint(sycl::float2& out, const CameraParams& deviceCamParams, const sycl::float3& X)
{
    const sycl::float3 p = M3x4mulV3(deviceCamParams.P, X);

    if(p.z() <= 0.0f)
        out = sycl::float2(-1.0f, -1.0f);
    else
        out = sycl::float2(p.x(), p.y()) / p.z();
}

inline sycl::float3 get3DPointForPixelAndFrontoParellePlaneRC(const CameraParams& deviceCamParams, const sycl::float2& pix, float fpPlaneDepth)
{
    const sycl::float3 planep = deviceCamParams.C + deviceCamParams.ZVect * fpPlaneDepth;
    const sycl::float3 v = sycl::normalize(M3x3mulV2(deviceCamParams.iP, pix));
    return linePlaneIntersect(deviceCamParams.C, v, planep, deviceCamParams.ZVect);
}

inline sycl::float3 get3DPointForPixelAndDepthFromRC(const CameraParams& deviceCamParams, const sycl::float2& pix, float depth)
{
    const sycl::float3 rpv = sycl::normalize(M3x3mulV2(deviceCamParams.iP, pix));
    return deviceCamParams.C + rpv * depth;
}

inline sycl::float3 triangulateMatchRef(const CameraParams& rcDeviceCamParams,
                                        const CameraParams& tcDeviceCamParams,
                                        const sycl::float2& refpix,
                                        const sycl::float2& tarpix)
{
    const sycl::float3 refvect = sycl::normalize(M3x3mulV2(rcDeviceCamParams.iP, refpix));
    const sycl::float3 refpoint = refvect + rcDeviceCamParams.C;

    const sycl::float3 tarvect = sycl::normalize(M3x3mulV2(tcDeviceCamParams.iP, tarpix));
    const sycl::float3 tarpoint = tarvect + tcDeviceCamParams.C;

    float k;

    {
    float l;
    sycl::float3 lli1, lli2;

    lineLineIntersect(k, l, lli1, lli2, rcDeviceCamParams.C, refpoint, tcDeviceCamParams.C, tarpoint);
    }

    return rcDeviceCamParams.C + refvect * k;
}

/**
 * @brief Subpixel refine by Stereo Matching with Color-Weighted Correlation, Hierarchical Belief Propagation,
 *        and Occlusion Handling Qingxiong pami08.
 *
 * @note Quadratic polynomial interpolation is used to approximate the cost function between
 *       three discrete depth candidates: d, dA, and dB.
 *
 * @see https://pubmed.ncbi.nlm.nih.gov/19147877/
 *
 * @param[in] depths the 3 depths candidates (depth-1, depth, depth+1)
 * @param[in] sims the similarity of the 3 depths candidates
 *
 * @return refined depth value
 */
inline float refineDepthSubPixel(const sycl::float3& depths, const sycl::float3& sims)
{
    /*
     * TODO: get formula back from paper as it has been lost by encoding.
     * Porting note: paper linked above contains no formulas, but references "Accurate and efficient stereo processing by semi-global matching and mutual information, H. Hirschmüller, 2005" to which I don't have accademic access.
     * Anybody updating this code might also want to read the later "Stereo Processing by Semi-Global Matchingand Mutual Information, H. Hirschmüller, 2008"
     * Given my lack of knowledge about this topic this is just a pure translation of the original CUDA implementation
     */
    // d is the discrete depth with the minimal cost, dA ? d A 1, and dB ? d B 1. The cost function is approximated as
    // f?x? ? ax2 B bx B c.

    const sycl::float3 sims_mod = (sims + 1.f) / 2.0f;
    const float simM1 = sims_mod.x();
    const float sim = sims_mod.y();
    const float simP1 = sims_mod.z();

    // sim is supposed to be the best one (so the smallest one)
    if((simM1 < sim) || (simP1 < sim))
        return depths.y(); // return the input

    const float dispStep = -((simP1 - simM1) / (2.0f * (simP1 + simM1 - 2.0f * sim)));

    const float floatDepthM1 = depths.x();
    const float floatDepthP1 = depths.z();

    //-1 : floatDepthM1
    // 0 : floatDepth
    //+1 : floatDepthP1
    // linear function fit
    // f(x)=a*x+b
    // floatDepthM1=-a+b
    // floatDepthP1= a+b
    // a = b - floatDepthM1
    // floatDepthP1=2*b-floatDepthM1
    const float b = (floatDepthP1 + floatDepthM1) / 2.0f;
    const float a = b - floatDepthM1;

    const float interpDepth = a * dispStep + b;

    // Ensure that the interpolated value is isfinite  (i.e. neither infinite nor NaN)
    if(!sycl::isfinite(interpDepth) || interpDepth <= 0.0f)
        return depths.y(); // return the input

    return interpDepth;
}

inline void computeRcTcMipmapLevels(float& out_rcMipmapLevel,
                                    float& out_tcMipmapLevel,
                                    const float mipmapLevel,
                                    const CameraParams& rcDeviceCamParams,
                                    const CameraParams& tcDeviceCamParams,
                                    const sycl::float2& rp0,
                                    const sycl::float2& tp0,
                                    const sycl::float3& p0)
{
    // get p0 depth from the R camera
    const float rcDepth = sycl::distance(rcDeviceCamParams.C, p0);

    // get p0 depth from the T camera
    const float tcDepth = sycl::distance(tcDeviceCamParams.C, p0);

    // get R p0 corresponding pixel + 1x
    const sycl::float2 rp1 = rp0 + sycl::float2(1.f, 0.f);

    // get T p0 corresponding pixel + 1x
    const sycl::float2 tp1 = tp0 + sycl::float2(1.f, 0.f);

    // get rp1 3d point
    const sycl::float3 rpv = sycl::normalize(M3x3mulV2(rcDeviceCamParams.iP, rp1));
    const sycl::float3 prp1 = rcDeviceCamParams.C + rpv * rcDepth;

    // get tp1 3d point
    const sycl::float3 tpv = sycl::normalize(M3x3mulV2(tcDeviceCamParams.iP, tp1));
    const sycl::float3 ptp1 = tcDeviceCamParams.C + tpv * tcDepth;

    // compute 3d distance between p0 and rp1 3d point
    const float rcDist = sycl::distance(p0, prp1);

    // compute 3d distance between p0 and tp1 3d point
    const float tcDist = sycl::distance(p0, ptp1);

    // compute Rc/Tc distance factor
    const float distFactor = rcDist / tcDist;

    // set output R and T mipmap level
    if(distFactor < 1.f)
    {
        // T camera has a lower resolution (1 Rc pixSize < 1 Tc pixSize)
        out_tcMipmapLevel = mipmapLevel - sycl::log2(1.f / distFactor);

        if(out_tcMipmapLevel < 0.f)
        {
            out_rcMipmapLevel = mipmapLevel + sycl::fabs(out_tcMipmapLevel);
            out_tcMipmapLevel = 0.f;
        }
    }
    else
    {
        // T camera has a higher resolution (1 Rc pixSize > 1 Tc pixSize)
        out_rcMipmapLevel = mipmapLevel;
        out_tcMipmapLevel = mipmapLevel + sycl::log2(distFactor);
    }
}

inline int angleBetwUnitV1andUnitV2(const sycl::float3& V1, const sycl::float3& V2)
{
    return (int)sycl::fabs(sycl::acos(sycl::dot(V1, V2)) / (std::numbers::pi / 180.0f));
}

/*
inline float getRefCamPixSize(Patch &ptch)
{
        sycl::float2 rp = project3DPoint(sg_s_r.P,ptch.p);

        float minstep=10000000.0f;
        for (int i=0;i<4;i++) {
                sycl::float2 pix = rp;
                if (i==0) {pix.x() += 1.0f;};
                if (i==1) {pix.x() -= 1.0f;};
                if (i==2) {pix.y() += 1.0f;};
                if (i==3) {pix.y() -= 1.0f;};
                sycl::float3 vect = M3x3mulV2(sg_s_r.iP,pix);
                sycl::float3 lpi = linePlaneIntersect(sg_s_r.C, vect, ptch.p, ptch.n);
                float step = dist(lpi,ptch.p);
                minstep = fminf(minstep,step);
        };

        return minstep;
}

inline float getTarCamPixSize(Patch &ptch)
{
        sycl::float2 tp = project3DPoint(sg_s_t.P,ptch.p);

        float minstep=10000000.0f;
        for (int i=0;i<4;i++) {
                sycl::float2 pix = tp;
                if (i==0) {pix.x() += 1.0f;};
                if (i==1) {pix.x() -= 1.0f;};
                if (i==2) {pix.y() += 1.0f;};
                if (i==3) {pix.y() -= 1.0f;};
                sycl::float3 vect = M3x3mulV2(sg_s_t.iP,pix);
                sycl::float3 lpi = linePlaneIntersect(sg_s_t.C, vect, ptch.p, ptch.n);
                float step = dist(lpi,ptch.p);
                minstep = fminf(minstep,step);
        };

        return minstep;
}

inline float getPatchPixSize(Patch &ptch)
{
        return fmaxf(getTarCamPixSize(ptch),getRefCamPixSize(ptch));
}
*/

inline void computeHomography(sycl::marray<float, 9>& out_H,
                              const CameraParams& rcDeviceCamParams,
                              const CameraParams& tcDeviceCamParams,
                              const sycl::float3& in_p,
                              const sycl::float3& in_n)
{
    // hartley zisserman second edition p.327 (13.2)
    const sycl::float3 _tl = -M3x3mulV3(rcDeviceCamParams.R, rcDeviceCamParams.C);
    const sycl::float3 _tr = -M3x3mulV3(tcDeviceCamParams.R, tcDeviceCamParams.C);

    const sycl::float3 p = M3x3mulV3(rcDeviceCamParams.R, (in_p - rcDeviceCamParams.C));
    const sycl::float3 n = sycl::normalize(M3x3mulV3(rcDeviceCamParams.R, in_n));
    const float d = -sycl::dot(n, p);

    sycl::marray<float, 9> RrT;
    M3x3transpose(RrT, rcDeviceCamParams.R);

    sycl::marray<float, 9> tmpRr;
    M3x3mulM3x3(tmpRr, tcDeviceCamParams.R, RrT);
    const sycl::float3 tr = _tr - M3x3mulV3(tmpRr, _tl);

    sycl::marray<float, 9> tmp;
    sycl::marray<float, 9> tmp1;
    outerMultiply(tmp, tr, n / d);
    tmp = tmpRr - tmp;
    M3x3mulM3x3(tmp1, tcDeviceCamParams.K, tmp);
    M3x3mulM3x3(tmp, tmp1, rcDeviceCamParams.iK);

    out_H = tmp;
}

/*
static float compNCCbyH(const DeviceCameraParams& rcDeviceCamParams,
                                   const DeviceCameraParams& tcDeviceCamParams,
                                   const Patch& ptch,
                                   int wsh)
{
    // get R and T image 2d coordinates from patch center 3d point
    const sycl::float2 rp = project3DPoint(rcDeviceCamParams.P, patch.p);
    //const sycl::float2 tp = project3DPoint(tcDeviceCamParams.P, patch.p);

    float H[9];
    computeHomography(H, rcDeviceCamParams, tcDeviceCamParams, ptch.p, ptch.n);

    simStat sst = simStat();

    for(int xp = -wsh; xp <= wsh; ++xp)
    {
        for(int yp = -wsh; yp <= wsh; ++yp)
        {
            sycl::float2 rpc;
            sycl::float2 tpc;
            rpc.x() = rp.x() + (float)xp;
            rpc.y() = rp.y() + (float)yp;
            tpc = V2M3x3mulV2(H, rpc);

            sycl::float2 g;
            g.x() = 255.0f * tex2D(rtex, rpc.x() + 0.5f, rpc.y() + 0.5f);
            g.y() = 255.0f * tex2D(ttex, tpc.x() + 0.5f, tpc.y() + 0.5f);
            sst.update(g);
        }
    }

    sst.computeSim();

    return sst.sim;
}
*/

/**
 * @brief Compute Normalized Cross-Correlation of a full square patch at given half-width.
 *
 * @tparam TInvertAndFilter invert and filter output similarity value
 *
 * @param[in] rcDeviceCameraParamsId the R camera parameters in device constant memory array
 * @param[in] tcDeviceCameraParamsId the T camera parameters in device constant memory array
 * @param[in] rcMipmapImage the R camera mipmap image
 * @param[in] tcMipmapImage the T camera mipmap image
 * @param[in] rcLevelWidth the R camera image width at given mipmapLevel
 * @param[in] rcLevelHeight the R camera image height at given mipmapLevel
 * @param[in] tcLevelWidth the T camera image width at given mipmapLevel
 * @param[in] tcLevelHeight the T camera image height at given mipmapLevel
 * @param[in] mipmapLevel the workflow current mipmap level (e.g. SGM=1.f, Refine=0.f)
 * @param[in] wsh the half-width of the patch
 * @param[in] invGammaC the inverted strength of grouping by color similarity
 * @param[in] invGammaP the inverted strength of grouping by proximity
 * @param[in] useConsistentScale enable consistent scale patch comparison
 * @param[in] tcLevelWidth the T camera image width at given mipmapLevel
 * @param[in] patch the input patch struct
 *
 * @return similarity value in range (-1.f, 0.f) or (0.f, 1.f) if TinvertAndFilter enabled
 *         special cases:
 *          -> infinite similarity value: 1
 *          -> invalid/uninitialized/masked similarity: std::numeric_limist<float>::infinity()
 */
template<bool TInvertAndFilter>
inline float compNCCby3DptsYK(const CameraParams& rcDeviceCamParams,
                              const CameraParams& tcDeviceCamParams,
                              const MipmapImageAccess& rcMipmapImage,
                              const MipmapImageAccess& tcMipmapImage,
                              const sycl::uint2& rcLevelSize,
                              const sycl::uint2& tcLevelSize,
                              const float mipmapLevel,
                              const int wsh,
                              const float invGammaC,
                              const float invGammaP,
                              const bool useConsistentScale,
                              const Patch& patch)
{
    // get R and T image 2d coordinates from patch center 3d point
    const sycl::float2 rp = project3DPoint(rcDeviceCamParams.P, patch.p);
    const sycl::float2 tp = project3DPoint(tcDeviceCamParams.P, patch.p);

    // image 2d coordinates margin
    const float dd = float(wsh) + 2.0f; // TODO: FACA

    // check R and T image 2d coordinates
    if((rp.x() < dd) || (rp.x() > float(rcLevelSize.x() - 1) - dd) ||
       (tp.x() < dd) || (tp.x() > float(tcLevelSize.x() - 1) - dd) ||
       (rp.y() < dd) || (rp.y() > float(rcLevelSize.y() - 1) - dd) ||
       (tp.y() < dd) || (tp.y() > float(tcLevelSize.y() - 1) - dd))
    {
        return std::numeric_limits<float>::infinity(); // uninitialized
    }

    // compute inverse width / height
    // note: useful to compute normalized coordinates
    const sycl::float2 rcInvLevelSize = 1.f / rcLevelSize.convert<float>();
    const sycl::float2 tcInvLevelSize = 1.f / tcLevelSize.convert<float>();

    // initialize R and T mipmap image level at the given mipmap image level
    float rcMipmapLevel = mipmapLevel;
    float tcMipmapLevel = mipmapLevel;

    // update R and T mipmap image level in order to get consistent scale patch comparison
    if(useConsistentScale)
    {
        computeRcTcMipmapLevels(rcMipmapLevel, tcMipmapLevel, mipmapLevel, rcDeviceCamParams, tcDeviceCamParams, rp, tp, patch.p);
    }

    // create and initialize SimStat struct
    simStat sst;

    // compute patch center color (CIELAB) at R and T mipmap image level
    const sycl::float4 rcCenterColor = rcMipmapImage.trilinear(rp*rcInvLevelSize, rcMipmapLevel).convert<float>();
    const sycl::float4 tcCenterColor = tcMipmapImage.trilinear(tp*tcInvLevelSize, tcMipmapLevel).convert<float>();

    // check the alpha values of the patch pixel center of the R and T cameras
    if(rcCenterColor.a() < ALICEVISION_DEPTHMAP_RC_MIN_ALPHA || tcCenterColor.a() < ALICEVISION_DEPTHMAP_TC_MIN_ALPHA)
    {
        return std::numeric_limits<float>::infinity(); // masked
    }

    // compute patch (wsh*2+1)x(wsh*2+1)
    for(int yp = -wsh; yp <= wsh; ++yp)
    {
        for(int xp = -wsh; xp <= wsh; ++xp)
        {
            // get 3d point
            const sycl::float3 p = patch.p + patch.x * patch.d * float(xp) + patch.y * patch.d * float(yp);

            // get R and T image 2d coordinates from 3d point
            const sycl::float2 rpc = project3DPoint(rcDeviceCamParams.P, p);
            const sycl::float2 tpc = project3DPoint(tcDeviceCamParams.P, p);

            // get R and T image color (CIELAB) from 2d coordinates
            const sycl::float4 rcPatchCoordColor = rcMipmapImage.trilinear(rpc*rcInvLevelSize, rcMipmapLevel).convert<float>();
            const sycl::float4 tcPatchCoordColor = tcMipmapImage.trilinear(tpc*tcInvLevelSize, tcMipmapLevel).convert<float>();

            // compute weighting based on:
            // - color difference to the center pixel of the patch:
            //    - low value (close to 0) means that the color is different from the center pixel (ie. strongly supported surface)
            //    - high value (close to 1) means that the color is close the center pixel (ie. uniform color)
            // - distance in image to the center pixel of the patch:
            //    - low value (close to 0) means that the pixel is close to the center of the patch
            //    - high value (close to 1) means that the pixel is far from the center of the patch
            const float w = CostYKfromLab(xp, yp, rcCenterColor, rcPatchCoordColor, invGammaC, invGammaP) * CostYKfromLab(xp, yp, tcCenterColor, tcPatchCoordColor, invGammaC, invGammaP);

            // update simStat
            sst.update(rcPatchCoordColor.x(), tcPatchCoordColor.x(), w);
        }
    }

    if(TInvertAndFilter)
    {
        // compute patch similarity
        const float fsim = sst.computeWSim();

        // invert and filter similarity
        // apply sigmoid see: https://www.desmos.com/calculator/skmhf1gpyf
        // best similarity value was -1, worst was 0
        // best similarity value is 1, worst is still 0
        return sigmoid(0.0f, 1.0f, 0.7f, -0.7f, fsim);
    }

    // compute output patch similarity
    return sst.computeWSim();
}

/**
 * @brief Compute Normalized Cross-Correlation of a patch with an user custom patch pattern.
 *
 * @tparam TInvertAndFilter invert and filter output similarity value
 *
 * @param[in] rcDeviceCameraParamsId the R camera parameters in device constant memory array
 * @param[in] tcDeviceCameraParamsId the T camera parameters in device constant memory array
 * @param[in] rcMipmapImage_tex the R camera mipmap image texture
 * @param[in] tcMipmapImage_tex the T camera mipmap image texture
 * @param[in] rcLevelWidth the R camera image width at given mipmapLevel
 * @param[in] rcLevelHeight the R camera image height at given mipmapLevel
 * @param[in] tcLevelWidth the T camera image width at given mipmapLevel
 * @param[in] tcLevelHeight the T camera image height at given mipmapLevel
 * @param[in] mipmapLevel the workflow current mipmap level (e.g. SGM=1.f, Refine=0.f)
 * @param[in] invGammaC the inverted strength of grouping by color similarity
 * @param[in] invGammaP the inverted strength of grouping by proximity
 * @param[in] useConsistentScale enable consistent scale patch comparison
 * @param[in] patch the input patch struct
 *
 * @return similarity value in range (-1.f, 0.f) or (0.f, 1.f) if TinvertAndFilter enabled
 *         special cases:
 *          -> infinite similarity value: 1
 *          -> invalid/uninitialized/masked similarity: CUDART_INF_F
 */
template<bool TInvertAndFilter>
inline float compNCCby3DptsYK_customPatchPattern(const CameraParams& rcDeviceCamParams,
                                                 const CameraParams& tcDeviceCamParams,
                                                 const MipmapImageAccess& rcMipmapImage,
                                                 const MipmapImageAccess& tcMipmapImage,
                                                 const sycl::uint2& rcLevelSize,
                                                 const sycl::uint2& tcLevelSize,
                                                 const float mipmapLevel,
                                                 const float invGammaC,
                                                 const float invGammaP,
                                                 const bool useConsistentScale,
                                                 const Patch& patch,
                                                 const PatchPattern& customPatchPattern)
{
    // get R and T image 2d coordinates from patch center 3d point
    const sycl::float2 rp = project3DPoint(rcDeviceCamParams.P, patch.p);
    const sycl::float2 tp = project3DPoint(tcDeviceCamParams.P, patch.p);

    // image 2d coordinates margin
    const float dd = 2.f; // TODO: proper wsh handling

    // check R and T image 2d coordinates
    if((rp.x() < dd) || (rp.x() > float(rcLevelSize.x() - 1) - dd) ||
       (tp.x() < dd) || (tp.x() > float(tcLevelSize.x() - 1) - dd) ||
       (rp.y() < dd) || (rp.y() > float(rcLevelSize.y() - 1) - dd) ||
       (tp.y() < dd) || (tp.y() > float(tcLevelSize.y() - 1) - dd))
    {
        return std::numeric_limits<float>::infinity(); // uninitialized
    }

    // compute inverse width / height
    // note: useful to compute normalized coordinates
    const sycl::float2 rcInvLevelSize = 1.f / rcLevelSize.convert<float>();
    const sycl::float2 tcInvLevelSize = 1.f / tcLevelSize.convert<float>();

    // get patch center pixel alpha at the given mipmap image level
    const float rcAlpha = rcMipmapImage.trilinear(rp*rcInvLevelSize, mipmapLevel).a(); // alpha only
    const float tcAlpha = tcMipmapImage.trilinear(tp*tcInvLevelSize, mipmapLevel).a(); // alpha only

    // check the alpha values of the patch pixel center of the R and T cameras
    if(rcAlpha < ALICEVISION_DEPTHMAP_RC_MIN_ALPHA || tcAlpha < ALICEVISION_DEPTHMAP_TC_MIN_ALPHA)
    {
        return std::numeric_limits<float>::infinity(); // masked
    }

    // initialize R and T mipmap image level at the given mipmap image level
    float rcMipmapLevel = mipmapLevel;
    float tcMipmapLevel = mipmapLevel;

    // update R and T mipmap image level in order to get consistent scale patch comparison
    if(useConsistentScale)
    {
        computeRcTcMipmapLevels(rcMipmapLevel, tcMipmapLevel, mipmapLevel, rcDeviceCamParams, tcDeviceCamParams, rp, tp, patch.p);
    }

    // output similarity initialization
    float fsim = 0.f;
    float wsum = 0.f;

    for(int s = 0; s < customPatchPattern.nbSubparts; ++s)
    {
        // create and initialize patch subpart SimStat
        simStat sst;

        // get patch pattern subpart
        const PatchPatternSubpart& subpart = customPatchPattern.subparts[s];

        // compute patch center color (CIELAB) at subpart level resolution
        const sycl::float4 rcCenterColor = rcMipmapImage.trilinear(rp*rcInvLevelSize, rcMipmapLevel + subpart.level).convert<float>();
        const sycl::float4 tcCenterColor = tcMipmapImage.trilinear(tp*tcInvLevelSize, tcMipmapLevel + subpart.level).convert<float>();

        if(subpart.isCircle)
        {
            for(int c = 0; c < subpart.nbCoordinates; ++c)
            {
                // get patch relative coordinates
                const sycl::float2& relativeCoord = subpart.coordinates[c];

                // get 3d point from relative coordinates
                const sycl::float3 p = patch.p + patch.x * float(patch.d * relativeCoord.x()) + patch.y * float(patch.d * relativeCoord.y());

                // get R and T image 2d coordinates from 3d point
                const sycl::float2 rpc = project3DPoint(rcDeviceCamParams.P, p);
                const sycl::float2 tpc = project3DPoint(tcDeviceCamParams.P, p);

                // get R and T image color (CIELAB) from 2d coordinates
                const sycl::float4 rcPatchCoordColor = rcMipmapImage.trilinear(rpc*rcInvLevelSize, rcMipmapLevel + subpart.level).convert<float>();
                const sycl::float4 tcPatchCoordColor = tcMipmapImage.trilinear(tpc*tcInvLevelSize, tcMipmapLevel + subpart.level).convert<float>();

                // compute weighting based on color difference to the center pixel of the patch:
                // - low value (close to 0) means that the color is different from the center pixel (ie. strongly supported surface)
                // - high value (close to 1) means that the color is close the center pixel (ie. uniform color)
                const float w = CostYKfromLab(rcCenterColor, rcPatchCoordColor, invGammaC) * CostYKfromLab(tcCenterColor, tcPatchCoordColor, invGammaC);

                // update simStat
                sst.update(rcPatchCoordColor.x(), tcPatchCoordColor.x(), w);
            }
        }
        else // full patch pattern
        {
            for(int yp = -subpart.wsh; yp <= subpart.wsh; ++yp)
            {
                for(int xp = -subpart.wsh; xp <= subpart.wsh; ++xp)
                {
                    // get 3d point
                    const sycl::float3 p = patch.p + patch.x * float(patch.d * float(xp) * subpart.downscale) + patch.y * float(patch.d * float(yp) * subpart.downscale);

                    // get R and T image 2d coordinates from 3d point
                    const sycl::float2 rpc = project3DPoint(rcDeviceCamParams.P, p);
                    const sycl::float2 tpc = project3DPoint(tcDeviceCamParams.P, p);

                    // get R and T image color (CIELAB) from 2d coordinates
                    const sycl::float4 rcPatchCoordColor = rcMipmapImage.trilinear(rpc*rcInvLevelSize, rcMipmapLevel + subpart.level).convert<float>();
                    const sycl::float4 tcPatchCoordColor = tcMipmapImage.trilinear(tpc*rcInvLevelSize, tcMipmapLevel + subpart.level).convert<float>();

                    // compute weighting based on:
                    // - color difference to the center pixel of the patch:
                    //    - low value (close to 0) means that the color is different from the center pixel (ie. strongly supported surface)
                    //    - high value (close to 1) means that the color is close the center pixel (ie. uniform color)
                    // - distance in image to the center pixel of the patch:
                    //    - low value (close to 0) means that the pixel is close to the center of the patch
                    //    - high value (close to 1) means that the pixel is far from the center of the patch
                    const float w = CostYKfromLab(xp, yp, rcCenterColor, rcPatchCoordColor, invGammaC, invGammaP) * CostYKfromLab(xp, yp, tcCenterColor, tcPatchCoordColor, invGammaC, invGammaP);

                    // update simStat
                    sst.update(rcPatchCoordColor.x(), tcPatchCoordColor.x(), w);
                }
            }
        }

        // compute patch subpart similarity
        const float fsimSubpart = sst.computeWSim();

        // similarity value in range (-1.f, 0.f) or invalid
        if(fsimSubpart < 0.f)
        {
            // add patch pattern subpart similarity to patch similarity
            if(TInvertAndFilter)
            {
                // invert and filter similarity
                // apply sigmoid see: https://www.desmos.com/calculator/skmhf1gpyf
                // best similarity value was -1, worst was 0
                // best similarity value is 1, worst is still 0
                const float fsimInverted = sigmoid(0.0f, 1.0f, 0.7f, -0.7f, fsimSubpart);
                fsim += fsimInverted * subpart.weight;

            }
            else
            {
                // weight and add similarity
                fsim += fsimSubpart * subpart.weight;
            }

            // sum subpart weight
            wsum += subpart.weight;
        }
    }

    // invalid patch similarity
    if(wsum == 0.f)
    {
        return std::numeric_limits<float>::infinity();
    }

    if(TInvertAndFilter)
    {
        // for now, we do not average
        return fsim;
    }

    // output average similarity
    return (fsim / wsum);
}

} // namespace depthMap_sycl
} // namespace aliceVision
