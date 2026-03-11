// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "deviceDepthSimilarityMap.hpp"

#include <aliceVision/depthMap_sycl/sycl/divUp.hpp>
#include <aliceVision/depthMap_sycl/sycl/Patch.hpp>
#include <aliceVision/depthMap_sycl/sycl/eig33.hpp>

//#define ALICEVISION_DEPTHMAP_COMPUTE_PIXSIZEMAP

inline float orientedPointPlaneDistanceNormalizedNormal(const sycl::float3& point,
                                                        const sycl::float3& planePoint,
                                                        const sycl::float3& planeNormalNormalized)
{
    return sycl::dot(point, planeNormalNormalized) - sycl::dot(planePoint, planeNormalNormalized);
}

namespace aliceVision {
namespace depthMap_sycl {

inline sycl::float2 getCellSmoothStepEnergy(const CameraParams& rcDeviceCamParams,
                                            const SyclDevicePitchedAccess<sycl::float2, 2>& in_depth_acc,
                                            const sycl::uint2& cell0,
                                            const sycl::uint2& offsetRoi)
{
    sycl::float2 out = sycl::float2(0.0f, 180.0f);

    // consider the neighbor pixels
    const sycl::int2 max = in_depth_acc.getDims().convert<int>() - 1;
    const sycl::uint2 cellL = sycl::clamp(cell0.convert<int>() + sycl::int2(0, -1),
                                          sycl::int2(0),
                                          max).convert<uint>(); // Left
    const sycl::uint2 cellR = sycl::clamp(cell0.convert<int>() + sycl::int2(0, 1),
                                          sycl::int2(0),
                                          max).convert<uint>(); // Right
    const sycl::uint2 cellT = sycl::clamp(cell0.convert<int>() + sycl::int2(-1, 0),
                                          sycl::int2(0),
                                          max).convert<uint>(); // Top
    const sycl::uint2 cellB = sycl::clamp(cell0.convert<int>() + sycl::int2(1, 0),
                                          sycl::int2(0),
                                          max).convert<uint>(); // Bottom

    // get associated depths from depth texture
    const float dT = __readonly_load(&in_depth_acc(cellT).x());

    // get pixel depth from the depth texture
    const float d0 = __readonly_load(&in_depth_acc(cell0).x());

    // early exit: depth is <= 0
    if(d0 <= 0.0f)
        return out;

    // get rest of associated depths
    const float dB = __readonly_load(&in_depth_acc(cellB).x());
    const float dL = __readonly_load(&in_depth_acc(cellL).x());
    const float dR = __readonly_load(&in_depth_acc(cellR).x());

    // get associated 3D points
    const sycl::float3 p0 = get3DPointForPixelAndDepthFromRC(rcDeviceCamParams, (cell0 + offsetRoi).convert<float>(), d0);
    const sycl::float3 pL = get3DPointForPixelAndDepthFromRC(rcDeviceCamParams, (cellL + offsetRoi).convert<float>(), dL);
    const sycl::float3 pR = get3DPointForPixelAndDepthFromRC(rcDeviceCamParams, (cellR + offsetRoi).convert<float>(), dR);
    const sycl::float3 pT = get3DPointForPixelAndDepthFromRC(rcDeviceCamParams, (cellT + offsetRoi).convert<float>(), dT);
    const sycl::float3 pB = get3DPointForPixelAndDepthFromRC(rcDeviceCamParams, (cellB + offsetRoi).convert<float>(), dB);

    // compute the average point based on neighbors (cg)
    sycl::float3 cg = sycl::float3(0.0f);
    float n = 0.0f;

    if(dL > 0.0f) { cg = cg + pL; n++; }
    if(dR > 0.0f) { cg = cg + pR; n++; }
    if(dT > 0.0f) { cg = cg + pT; n++; }
    if(dB > 0.0f) { cg = cg + pB; n++; }

    // if we have at least one valid depth
    if(n > 1.0f)
    {
        cg = cg / n; // average of x, y, depth
        const sycl::float3 vcn = sycl::normalize(rcDeviceCamParams.C - p0);
        // pS: projection of cg on the line from p0 to camera
        const sycl::float3 pS = closestPointToLine3D(cg, p0, vcn);
        // keep the depth difference between pS and p0 as the smoothing step
        out.x() = sycl::length(rcDeviceCamParams.C - pS) - d0;
    }

    float e = 0.0f;
    n = 0.0f;

    if(dL > 0.0f && dR > 0.0f)
    {
        // large angle between neighbors == flat area => low energy
        // small angle between neighbors == non-flat area => high energy
        e = sycl::fmax(e, (180.0f - angleBetwABandAC(p0, pL, pR)));
        n++;
    }
    if(dT > 0.0f && dB > 0.0f)
    {
        e = sycl::fmax(e, (180.0f - angleBetwABandAC(p0, pT, pB)));
        n++;
    }
    // the higher the energy, the less flat the area
    if(n > 0.0f) out.y() = e;

    return out;
}

sycl::event sycl_depthSimMapCopyDepthOnly(SyclDeviceMemoryPitched<sycl::float2, 2>& out_depthSimMap_dmp,
                                          const SyclDeviceMemoryPitched<sycl::float2, 2>& in_depthSimMap_dmp,
                                          float defaultSim,
                                          sycl::queue& queue, sycl::event prerequisite)
{
    // get output map dimensions
    const SyclSize<2>& depthSimMapDim = out_depthSimMap_dmp.getSize();

    // Accessors
    SyclDevicePitchedAccess out_depthSimMap_acc = SyclDevicePitchedAccess(out_depthSimMap_dmp);
    const SyclDevicePitchedAccess in_depthSimMap_acc = SyclDevicePitchedAccess(in_depthSimMap_dmp);

    // kernel execution
    return queue.submit([&] (sycl::handler& h) {
        h.depends_on(prerequisite);

        h.parallel_for(sycl::range(depthSimMapDim.x(), depthSimMapDim.y()), [=] (sycl::id<2> id) {
            const sycl::uint2 coords = sycl::uint2(id[0], id[1]);
            sycl::float2& out = out_depthSimMap_acc(coords);
            out.x() = __readonly_load(&in_depthSimMap_acc(coords).x());
            out.y() = defaultSim;
        });
    });
}

sycl::event sycl_normalMapUpscale(SyclDeviceMemoryPitched<sycl::float3, 2>& out_upscaledMap_dmp,
                                  const SyclDeviceMemoryPitched<sycl::float3, 2>& in_map_dmp,
                                  const ROI& roi,
                                  sycl::queue& queue, sycl::event prerequisite)
{
    // compute upscale ratio
    const SyclSize<2>& out_mapDim = out_upscaledMap_dmp.getSize();
    const SyclSize<2>& in_mapDim = in_map_dmp.getSize();
    const float ratio = float(in_mapDim.x()) / float(out_mapDim.x());
    const sycl::uint2 dims = sycl::uint2(roi.width(), roi.height());

    // Accessors
    SyclDevicePitchedAccess out_upscaledMap_acc = SyclDevicePitchedAccess(out_upscaledMap_dmp);
    const SyclDevicePitchedAccess in_map_acc = SyclDevicePitchedAccess(in_map_dmp);

    // kernel execution
    return queue.submit([&] (sycl::handler& h) {
        h.depends_on(prerequisite);

        h.parallel_for(sycl::range(roi.width(), roi.height()), [=] (sycl::id<2> id) {
            const sycl::uint2 coords = sycl::uint2(id[0], id[1]);
            const sycl::uint2 nearest = sycl::min(sycl::floor(coords.convert<float>()*ratio).convert<uint>(),
                                                  (dims.convert<float>()*ratio).convert<uint>() - 1);

            out_upscaledMap_acc(coords) = __readonly_load(&in_map_acc(nearest));
        });
    });
}

sycl::event sycl_depthThicknessSmoothThickness(SyclDeviceMemoryPitched<sycl::float2, 2>& inout_depthThicknessMap_dmp,
                                               const depthMapCommon::SgmParams& sgmParams,
                                               const depthMapCommon::RefineParams& refineParams,
                                               const ROI& roi,
                                               sycl::queue& queue, sycl::event prerequisite)
{
    const int sgmScaleStep = sgmParams.scale * sgmParams.stepXY;
    const int refineScaleStep = refineParams.scale * refineParams.stepXY;

    const sycl::uint2 dims = sycl::uint2(roi.width(), roi.height());

    // min/max number of Refine samples in SGM thickness area
    constexpr float minNbRefineSamples = 2.f;
    const float maxNbRefineSamples = sycl::max(sgmScaleStep / float(refineScaleStep), minNbRefineSamples);

    // min/max SGM thickness inflate factor
    const float minThicknessInflate = refineParams.halfNbDepths / maxNbRefineSamples;
    const float maxThicknessInflate = refineParams.halfNbDepths / minNbRefineSamples;

    // Accessor
    SyclDevicePitchedAccess inout_depthThicknessMap_acc = SyclDevicePitchedAccess(inout_depthThicknessMap_dmp);

    // kernel execution
    return queue.submit([&] (sycl::handler& h) {
        h.depends_on(prerequisite);

        h.parallel_for(sycl::range(roi.width(), roi.height()), [=] (sycl::id<2> id) {
            const sycl::uint2 coords = sycl::uint2(id[0], id[1]);

            sycl::float2& inout_depthThickness = inout_depthThicknessMap_acc(coords);

            if(inout_depthThickness.x() <= 0.0f) return;

            const float minThickness = minThicknessInflate * inout_depthThickness.y();
            const float maxThickness = maxThicknessInflate * inout_depthThickness.y();

            // compute average depth distance to the center pixel
            float sumCenterDepthDist = 0.f;
            int nbValidPatchPixels = 0;

            // patch 3x3
#pragma unroll
            for(int yp = -1; yp <= 1; ++yp)
            {
#pragma unroll
                for(int xp = -1; xp <= 1; ++xp)
                {
                    // compute patch coordinates
                    const sycl::uint2 p = (coords.convert<int>() + sycl::int2(xp, yp)).convert<uint>();

                    if((xp == 0 && yp == 0) ||             // avoid pixel center
                       p.x() < 0 || p.x() >= dims.x() ||   // avoid pixel outside the ROI
                       p.y() < 0 || p.y() >= dims.y())     // avoid pixel outside the ROI
                    {
                        continue;
                    }

                    // corresponding path depth/thickness
                    const sycl::float2 in_depthThicknessPatch = inout_depthThicknessMap_acc(coords);

                    // patch depth valid
                    if(in_depthThicknessPatch.x() > 0.0f)
                    {
                        const float depthDistance = sycl::fabs(inout_depthThickness.x() - in_depthThicknessPatch.x());
                        sumCenterDepthDist += sycl::clamp(depthDistance, minThickness, maxThickness);
                        ++nbValidPatchPixels;
                    }
                }
            }

            // we require at least 3 valid patch pixels (over 8)
            if(nbValidPatchPixels < 3) return;

            // write output smooth thickness
            inout_depthThickness.y() = sumCenterDepthDist / nbValidPatchPixels;
        });
    });
}

sycl::event sycl_computeSgmUpscaledDepthPixSizeMap(SyclDeviceMemoryPitched<sycl::float2, 2>& out_upscaledDepthPixSizeMap_dmp,
                                                   const SyclDeviceMemoryPitched<sycl::float2, 2>& in_sgmDepthThicknessMap_dmp,
                                                   const CameraParams& camParams,
                                                   const DeviceMipmapImage& rcDeviceMipmapImage,
                                                   const depthMapCommon::RefineParams& refineParams,
                                                   const ROI& roi,
                                                   sycl::queue& queue, sycl::event prerequisite)
{
    // compute upscale ratio
    const SyclSize<2>& out_mapDim = out_upscaledDepthPixSizeMap_dmp.getSize();
    const SyclSize<2>& in_mapDim = in_sgmDepthThicknessMap_dmp.getSize();
    const float ratio = float(in_mapDim.x()) / float(out_mapDim.x());

    // constants
    const bool interpolate = refineParams.interpolateMiddleDepth; // AdaptiveCPP will automatically generate specialized kernels, so this way we reduce code duplication
    const sycl::uint2 roiBegin = sycl::uint2(roi.x.begin, roi.y.begin);
    const sycl::uint2 roiSize = sycl::uint2(roi.width(), roi.height());
    const uint stepXY = refineParams.stepXY;
    const float halfNbDepths = refineParams.halfNbDepths;

    // get R mipmap image level
    const float rcMipmapLevel = rcDeviceMipmapImage.getLevel(refineParams.scale);
    const SyclSize<2> rcMipmapSize = rcDeviceMipmapImage.getDimensions(refineParams.scale);
    const sycl::float2 invMMSize = 1.f / sycl::uint2(rcMipmapSize[0], rcMipmapSize[1]).convert<float>();

    // Accessors
    SyclDevicePitchedAccess out_upscaledDepthPixSizeMap_acc = SyclDevicePitchedAccess(out_upscaledDepthPixSizeMap_dmp);
    const SyclDevicePitchedAccess in_sgmDepthThicknessMap_acc = SyclDevicePitchedAccess(in_sgmDepthThicknessMap_dmp);
    const MipmapImageAccess rcDeviceMipmapAcc = MipmapImageAccess(rcDeviceMipmapImage);

    // kernel execution
    return queue.submit([&] (sycl::handler& h) {
        h.depends_on(prerequisite);

        h.parallel_for(sycl::range(roi.width(), roi.height()), [=] (sycl::id<2> id) {
            const sycl::uint2 rCoords = sycl::uint2(id[0], id[1]);

            // corresponding image coordinates
            const sycl::uint2 iCoords = (roiBegin + rCoords) * stepXY;

            // corresponding output upscaled depth/pixSize map
            sycl::float2& out_depthPixSize = out_upscaledDepthPixSizeMap_acc(rCoords);

            // filter masked pixels (alpha < 0.9f)
            if(rcDeviceMipmapAcc.trilinear(iCoords.convert<float>() * invMMSize, rcMipmapLevel).a() < ALICEVISION_DEPTHMAP_RC_MIN_ALPHA)
            {
                out_depthPixSize = {-2.f, 0.f};
                return;
            }

            // find corresponding depth/thickness
            sycl::float2 out_depthThickness;
            const sycl::float2 o = rCoords.convert<float>() * ratio;
            const sycl::float2 p = sycl::floor(o);
            if(!interpolate)
            {
                // nearest neighbor, no interpolation
                out_depthThickness = in_sgmDepthThicknessMap_acc.sample_near(p);
            }
            else
            {
                const sycl::float2 lu = in_sgmDepthThicknessMap_acc.sample_near(p);
                const sycl::float2 ru = in_sgmDepthThicknessMap_acc.sample_near(p + sycl::float2(1.f, 0));
                const sycl::float2 ld = in_sgmDepthThicknessMap_acc.sample_near(p + sycl::float2(0, 1.f));
                const sycl::float2 rd = in_sgmDepthThicknessMap_acc.sample_near(p + sycl::float2(1.f, 1.f));

                if(lu.x() <= 0.0f || ru.x() <= 0.0f || rd.x() <= 0.0f || ld.x() <= 0.0f)
                {
                    // at least one corner depth is invalid
                    // average the other corners to get a proper depth/thickness
                    sycl::float2 sumDepthThickness = {0.0f, 0.0f};
                    int count = 0;

                    if(lu.x() > 0.0f)
                    {
                        sumDepthThickness = sumDepthThickness + lu;
                        ++count;
                    }
                    if(ru.x() > 0.0f)
                    {
                        sumDepthThickness = sumDepthThickness + ru;
                        ++count;
                    }
                    if(rd.x() > 0.0f)
                    {
                        sumDepthThickness = sumDepthThickness + rd;
                        ++count;
                    }
                    if(ld.x() > 0.0f)
                    {
                        sumDepthThickness = sumDepthThickness + ld;
                        ++count;
                    }
                    if(count != 0)
                    {
                        out_depthThickness = sumDepthThickness / float(count);
                    }
                    else
                    {
                        // invalid depth
                        out_depthPixSize = {-1.0f, 1.0f};
                        return;
                    }
                }
                else
                {
                    // bilinear interpolation
                    const sycl::float2 i = o - p;
                    const sycl::float2 u = sycl::mix(lu, ru, i.x());
                    const sycl::float2 d = sycl::mix(ld, rd, i.x());
                    out_depthThickness = sycl::mix(u, d, i.y());
                }
            }

#ifdef ALICEVISION_DEPTHMAP_COMPUTE_PIXSIZEMAP
            // get rc 3d point
            const sycl::float3 p = get3DPointForPixelAndDepthFromRC(camParams, iCoords.convert<float>(), out_depthThickness.x());

            // compute and write rc 3d point pixSize
            const float out_pixSize = computePixSize(camParams, p);
#else
            // compute pixSize from depth thickness
            const float out_pixSize = out_depthThickness.y() / halfNbDepths;
#endif

            // write output depth/pixSize
            out_depthPixSize.x() = out_depthThickness.x();
            out_depthPixSize.y() = out_pixSize;
        });
    });
}

sycl::event sycl_depthMapComputeNormal(SyclDeviceMemoryPitched<sycl::float3, 2>& out_normalMap_dmp,
                                       const SyclDeviceMemoryPitched<sycl::float2, 2>& in_depthSimMap_dmp,
                                       const CameraParams& camParams,
                                       const int stepXY,
                                       const ROI& roi,
                                       sycl::queue& queue, sycl::event prerequisite)
{
    // constants
    constexpr int wsh = 3;
    const sycl::uint2 roiBegin = sycl::uint2(roi.x.begin, roi.y.begin);
    const sycl::uint2 roiSize = sycl::uint2(roi.width(), roi.height());

    // Accessors
    SyclDevicePitchedAccess out_normalMap_acc = SyclDevicePitchedAccess(out_normalMap_dmp);
    const SyclDevicePitchedAccess in_depthMap_acc = SyclDevicePitchedAccess(in_depthSimMap_dmp);

    // kernel execution
    return queue.submit([&] (sycl::handler& h) {
        h.depends_on(prerequisite);

        h.parallel_for(sycl::range(roi.width(), roi.height()), [=] (sycl::id<2> id) {
            const sycl::uint2 rCoords = sycl::uint2(id[0], id[1]);

            // corresponding image coordinates
            const sycl::uint2 iCoords = (roiBegin + rCoords) * stepXY;


            // corresponding input depth
            const float in_depth = __readonly_load(&in_depthMap_acc(rCoords).x()); // use only depth

            // corresponding output normal
            sycl::float3& out_normalRef = out_normalMap_acc(rCoords);

            // no depth
            if(in_depth <= 0.0f)
            {
                out_normalRef = sycl::float3(-1.f);
                return;
            }

            const sycl::float3 p = get3DPointForPixelAndDepthFromRC(camParams, iCoords.convert<float>(), in_depth);
            const float pixSize = sycl::length(p - get3DPointForPixelAndDepthFromRC(camParams, iCoords.convert<float>()+sycl::float2(1.f, 0.f), in_depth));

            sycl_stat3d s3d = sycl_stat3d();

#pragma unroll
            for(int yp = -wsh; yp <= wsh; ++yp)
            {
                const int roiYp = int(rCoords.y()) + yp;
                if(roiYp < 0 || roiYp >= roiSize.y()) continue;

#pragma unroll
                for(int xp = -wsh; xp <= wsh; ++xp)
                {
                    const int roiXp = int(rCoords.x()) + xp;
                    if(roiXp < 0 || roiXp >= roiSize.x()) continue;

                    const float depthP = __readonly_load(&in_depthMap_acc(sycl::int2(roiXp, roiYp).convert<uint>()).x());  // use only depth

                    if((depthP > 0.0f) && (sycl::fabs(depthP - in_depth) < 30.0f * pixSize))
                    {
                        constexpr double w = 1.0;
                        const sycl::float2 pixP = (iCoords.convert<int>()+sycl::int2(xp, yp)).convert<float>();
                        const sycl::float3 pP = get3DPointForPixelAndDepthFromRC(camParams, pixP, depthP);
                        s3d.update(pP, w);
                    }
                }
            }

            sycl::float3 pp = p;
            sycl::float3 nn = sycl::float3(-1.f);

            if(!s3d.computePlaneByPCA(pp, nn))
            {
                out_normalRef = sycl::float3(-1.f);
                return;
            }

            const sycl::float3 nc = sycl::normalize(camParams.C - p);

            if(orientedPointPlaneDistanceNormalizedNormal(pp + nn, pp, nc) < 0.0f)
            {
                nn *= -1.f;
            }

            out_normalRef = nn;
        });
    });
}

sycl::event sycl_depthSimMapOptimizeGradientDescent(SyclDeviceMemoryPitched<sycl::float2, 2>& out_optimizeDepthSimMap_dmp,
                                                    SyclDeviceMemoryPitched<float, 2>& inout_imgVariance_dmp,
                                                    const SyclDeviceMemoryPitched<sycl::float2, 2>& in_sgmDepthPixSizeMap_dmp,
                                                    const SyclDeviceMemoryPitched<sycl::float2, 2>& in_refineDepthSimMap_dmp,
                                                    const CameraParams& camParams,
                                                    const DeviceMipmapImage& rcDeviceMipmapImage,
                                                    const depthMapCommon::RefineParams& refineParams,
                                                    const ROI& roi,
                                                    sycl::queue& queue, sycl::event prerequisite)
{
    // get R mipmap image level/size
    const float rcMipmapLevel = rcDeviceMipmapImage.getLevel(refineParams.scale);
    const SyclSize<2> levelSize_h = rcDeviceMipmapImage.getDimensions(refineParams.scale);
    const sycl::uint2 levelSize = sycl::uint2(levelSize_h[0], levelSize_h[1]);
    const sycl::float2 invLevelSize = 1.f / levelSize.convert<float>();

    // initialize depth/sim map optimized with SGM depth/pixSize map
    prerequisite = out_optimizeDepthSimMap_dmp.copyFrom(in_sgmDepthPixSizeMap_dmp, queue, prerequisite);

    // Constants
    const int stepXY = refineParams.stepXY;
    const sycl::uint2 roiBegin = sycl::uint2(roi.x.begin, roi.y.begin);
    const sycl::uint2 roiSize = sycl::uint2(roi.width(), roi.height());

    // Accessors
    SyclDevicePitchedAccess out_optimizeDepthSimMap_acc{out_optimizeDepthSimMap_dmp};
    SyclDevicePitchedAccess inout_imgVariance_acc{inout_imgVariance_dmp};
    const SyclDevicePitchedAccess in_sgmDepthPixSizeMap_acc{in_sgmDepthPixSizeMap_dmp};
    const SyclDevicePitchedAccess in_refineDepthSimMap_acc{in_refineDepthSimMap_dmp};
    const MipmapImageAccess rcMipmap_acc{rcDeviceMipmapImage};

    // initial kernel execution
    prerequisite = queue.submit([&] (sycl::handler& h) {
        h.depends_on(prerequisite);

        h.parallel_for(sycl::range(roi.width(), roi.height()), [=] (sycl::id<2> id) {
            const sycl::uint2 rCoords = sycl::uint2(id[0], id[1]);

            // corresponding image coordinates
            const sycl::uint2 iCoords = (roiBegin + rCoords) * stepXY;

            // compute gradient size of L
            const float xM1 = rcMipmap_acc.trilinear((iCoords.convert<int>() + sycl::int2(-1, 0)).convert<float>()*invLevelSize, rcMipmapLevel).x();
            const float xP1 = rcMipmap_acc.trilinear((iCoords.convert<int>() + sycl::int2(1, 0)).convert<float>()*invLevelSize, rcMipmapLevel).x();
            const float yM1 = rcMipmap_acc.trilinear((iCoords.convert<int>() + sycl::int2(0, -1)).convert<float>()*invLevelSize, rcMipmapLevel).x();
            const float yP1 = rcMipmap_acc.trilinear((iCoords.convert<int>() + sycl::int2(0, 1)).convert<float>()*invLevelSize, rcMipmapLevel).x();


            const sycl::float2 g = sycl::float2(xM1 - xP1, yM1 - yP1); // TODO: not divided by 2?
            const float grad = sycl::length(g);

            // write output
            inout_imgVariance_acc(rCoords) = grad;
        });
    });

    for(int iter = 0; iter < refineParams.optimizationNbIterations; ++iter) // default nb iterations is 100
    {
        const bool firstIter = (iter==0);

        // adjust depth/sim by using previously computed depths
        prerequisite = queue.submit([&] (sycl::handler& h) {
            h.depends_on(prerequisite);

            h.parallel_for(sycl::range(roi.width(), roi.height()), [=] (sycl::id<2> id) {
                const sycl::uint2 rCoords = sycl::uint2(id[0], id[1]);

                // SGM upscale (rough) depth/pixSize
                const sycl::float2 sgmDepthPixSize = in_sgmDepthPixSizeMap_acc(rCoords);
                const float sgmDepth = sgmDepthPixSize.x();
                const float sgmPixSize = sgmDepthPixSize.y();

                // refined and fused (fine) depth/sim
                const sycl::float2 refineDepthSim = in_refineDepthSimMap_acc(rCoords);
                const float refineDepth = refineDepthSim.x();
                const float refineSim = refineDepthSim.y();

                // output optimized depth/sim
                sycl::float2& out_optDepthSimRef = out_optimizeDepthSimMap_acc(rCoords);
                sycl::float2 out_optDepthSim = (firstIter) ? sycl::float2(sgmDepth, refineSim) : out_optDepthSimRef;
                const float depthOpt = out_optDepthSim.x();

                if (depthOpt > 0.0f)
                {
                    const sycl::float2 depthSmoothStepEnergy = getCellSmoothStepEnergy(camParams, out_optimizeDepthSimMap_acc, rCoords, roiBegin); // (smoothStep, energy)
                    float stepToSmoothDepth = depthSmoothStepEnergy.x();
                    stepToSmoothDepth = sycl::copysign(sycl::fmin(sycl::fabs(stepToSmoothDepth), sgmPixSize / 10.0f), stepToSmoothDepth);
                    const float depthEnergy = depthSmoothStepEnergy.y(); // max angle with neighbors
                    float stepToFineDM = refineDepth - depthOpt; // distance to refined/noisy input depth map
                    stepToFineDM = sycl::copysign(sycl::fmin(sycl::fabs(stepToFineDM), sgmPixSize / 10.0f), stepToFineDM);

                    const float stepToRoughDM = sgmDepth - depthOpt; // distance to smooth/robust input depth map
                    const float imgColorVariance = inout_imgVariance_acc(rCoords);
                    const float colorVarianceThresholdForSmoothing = 20.0f;
                    const float angleThresholdForSmoothing = 30.0f; // 30

                    // https://www.desmos.com/calculator/kob9lxs9qf
                    const float weightedColorVariance = sigmoid2(5.0f, angleThresholdForSmoothing, 40.0f, colorVarianceThresholdForSmoothing, imgColorVariance);

                    // https://www.desmos.com/calculator/jwhpjq6ppj
                    const float fineSimWeight = sigmoid(0.0f, 1.0f, 0.7f, -0.7f, refineSim);

                    // if geometry variation is bigger than color variation => the fineDM is considered noisy

                    // if depthEnergy > weightedColorVariance   => energyLowerThanVarianceWeight=0 => smooth
                    // else:                                    => energyLowerThanVarianceWeight=1 => use fineDM
                    // weightedColorVariance max value is 30, so if depthEnergy > 30 (which means depthAngle < 150�) energyLowerThanVarianceWeight will be 0
                    // https://www.desmos.com/calculator/jzbweilb85
                    const float energyLowerThanVarianceWeight = sigmoid(0.0f, 1.0f, 30.0f, weightedColorVariance, depthEnergy); // TODO: 30 => 60

                    // https://www.desmos.com/calculator/ilsk7pthvz
                    const float closeToRoughWeight = 1.0f - sigmoid(0.0f, 1.0f, 10.0f, 17.0f, sycl::fabs(stepToRoughDM / sgmPixSize)); // TODO: 10 => 30

                    // f(z) = c1 * s1(z_rought - z)^2 + c2 * s2(z-z_fused)^2 + coeff3 * s3*(z-z_smooth)^2

                    const float depthOptStep = closeToRoughWeight * stepToRoughDM + // distance to smooth/robust input depth map
                        (1.0f - closeToRoughWeight) * (energyLowerThanVarianceWeight * fineSimWeight * stepToFineDM + // distance to refined/noisy
                                                       (1.0f - energyLowerThanVarianceWeight) * stepToSmoothDepth); // max angle in current depthMap

                    out_optDepthSim.x() = depthOpt + depthOptStep;

                    out_optDepthSim.y() = (1.0f - closeToRoughWeight) * (energyLowerThanVarianceWeight * fineSimWeight * refineSim + (1.0f - energyLowerThanVarianceWeight) * (depthEnergy / 20.0f));
                }

                out_optDepthSimRef = out_optDepthSim;
            });
        });
    }

    return prerequisite;
}

} // namespace depthMap_sycl
} // namespace aliceVision
