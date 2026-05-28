// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "deviceSimilarityVolume.hpp"

#include <aliceVision/depthMap_sycl/sycl/divUp.hpp>
#include <aliceVision/depthMap_sycl/sycl/Patch.hpp>
#include <aliceVision/depthMap_sycl/sycl/matrix.hpp>

#include <map>

namespace aliceVision {
namespace depthMap_sycl {

inline void move3DPointByRcPixSize(sycl::float3& p,
                                   const CameraParams& rcParams,
                                   const float rcPixSize)
{
    sycl::float3 rpv = sycl::normalize(p - rcParams.C);
    p = p + rpv * rcPixSize;
}

inline void volume_computePatch(Patch& patch,
                                const CameraParams& rcParams,
                                const CameraParams& tcParams,
                                const float fpPlaneDepth,
                                const sycl::float2& pix)
{
    patch.p = get3DPointForPixelAndFrontoParellePlaneRC(rcParams, pix, fpPlaneDepth);
    patch.d = computePixSize(rcParams, patch.p);
    computeRotCSEpip(patch, rcParams, tcParams);
}

inline float depthPlaneToDepth(const CameraParams& camParams,
                               const float fpPlaneDepth,
                               const sycl::float2& pix)
{
    const sycl::float3 planep = camParams.C + camParams.ZVect * fpPlaneDepth;
    const sycl::float3 v = sycl::normalize(M3x3mulV2(camParams.iP, pix));
    const sycl::float3 p = linePlaneIntersect(camParams.C, v, planep, camParams.ZVect);
    return sycl::distance(camParams.C, p);
}

sycl::event sycl_volumeAdd(SyclDeviceMemoryPitched<TSimRefine, 3>& inout_volume_dmp,
                           const SyclDeviceMemoryPitched<TSimRefine, 3>& in_volume_dmp,
                           sycl::queue& queue, sycl::event prerequisite)
{
    assert(inout_volume_dmp.getSize() == in_volume_dmp.getSize());

    // Get pointers out here to avoid copying the entire SyclDeviceMemoryPitched struct over to the device
    TSimRefine* inout_v_ptr = inout_volume_dmp.getBuffer();
    const TSimRefine* in_v_ptr = in_volume_dmp.getBuffer();

    return queue.submit([&](sycl::handler& h) {
        h.depends_on(prerequisite);

        h.parallel_for(inout_volume_dmp.getUnitsTotal(), [=](auto idx) {
            inout_v_ptr[idx] += in_v_ptr[idx];
        });
    });
}

sycl::event sycl_volumeUpdateUninitializedSimilarity(const SyclDeviceMemoryPitched<TSim, 3>& in_volBestSim_dmp,
                                                     SyclDeviceMemoryPitched<TSim, 3>& inout_volSecBestSim_dmp,
                                                     sycl::queue& queue, sycl::event prerequisite)
{
    assert(in_volBestSim_dmp.getSize() == inout_volSecBestSim_dmp.getSize());

    // Get pointers out here to avoid copying the entire SyclDevicePitchedAccess struct over to the device
    TSim* inout_v_ptr = inout_volSecBestSim_dmp.getBuffer();
    const TSim* in_v_ptr = in_volBestSim_dmp.getBuffer();

    return queue.submit([&](sycl::handler& h) {
        h.depends_on(prerequisite);

        h.parallel_for(in_volBestSim_dmp.getUnitsTotal(), [=](auto idx) {
            TSim& inout_val = inout_v_ptr[idx];
            if (inout_val >= 255.f)
                inout_val = in_v_ptr[idx];
        });
    });
}

sycl::event sycl_volumeComputeSimilarity(SyclDeviceMemoryPitched<TSim, 3>& out_volBestSim_dmp,
                                         SyclDeviceMemoryPitched<TSim, 3>& out_volSecBestSim_dmp,
                                         const SyclDeviceMemoryPitched<float, 1>& in_depths_dmp,
                                         const CameraParams& rcParams,
                                         const CameraParams& tcParams,
                                         const DeviceMipmapImage& rcDeviceMipmapImage,
                                         const DeviceMipmapImage& tcDeviceMipmapImage,
                                         const depthMapCommon::SgmParams& sgmParams,
                                         const Range& depthRange,
                                         const ROI& roi,
                                         sycl::queue& queue, sycl::event prerequisite)
{
    // get mipmap images level and dimensions
    const float rcMipmapLevel = rcDeviceMipmapImage.getLevel(sgmParams.scale);
    const SyclSize<2> rcLevelDim_h = rcDeviceMipmapImage.getDimensions(sgmParams.scale);
    const sycl::uint2 rcLevelDim = sycl::uint2(rcLevelDim_h[0], rcLevelDim_h[1]);
    const SyclSize<2> tcLevelDim_h = tcDeviceMipmapImage.getDimensions(sgmParams.scale);
    const sycl::uint2 tcLevelDim = sycl::uint2(tcLevelDim_h[0], tcLevelDim_h[1]);

    // get image access
    SyclDevicePitchedAccess<TSim, 3> out_volBestSim_acc(out_volBestSim_dmp);
    SyclDevicePitchedAccess<TSim, 3> out_volSecBestSim_acc(out_volSecBestSim_dmp);
    const SyclDevicePitchedAccess<float, 1> in_depths_acc(in_depths_dmp);
    const MipmapImageAccess rcDeviceMipmapAcc(rcDeviceMipmapImage);
    const MipmapImageAccess tcDeviceMipmapAcc(tcDeviceMipmapImage);

    // bits we can calculate outside of the kernel
    const int stepXY = sgmParams.stepXY;
    const int wsh = sgmParams.wsh;
    const bool useConsistentScale = sgmParams.useConsistentScale;
    const float invGammaC = (1.f / float(sgmParams.gammaC)); // inverted gammaC
    const float invGammaP = (1.f / float(sgmParams.gammaP)); // inverted gammaP

    // kernel launch
    if(sgmParams.useCustomPatchPattern)
    {
        const PatchPattern& patchPattern = PatchPattern::getGlobalPatchPattern();

        return queue.submit([&](sycl::handler& h) {
            h.depends_on(prerequisite);

            h.parallel_for(sycl::range<1>(roi.width() * roi.height() * depthRange.size()), [=](sycl::id<1> id) {

                // corresponding volume coordinates
                const sycl::uint3 v = sycl::uint3(id[0] / (depthRange.size() * roi.height()),
                                                  id[0] / depthRange.size() % roi.height(),
                                                  id[0] % depthRange.size() + depthRange.begin);

                // corresponding image coordinates
                const sycl::float2 icoords = (sycl::uint2(v.x() + roi.x.begin, v.y() + roi.y.begin) * stepXY).convert<float>();

                // corresponding depth plane
                const float depthPlane = __readonly_load(&in_depths_acc(v.z()));

                // compute patch
                Patch patch;
                volume_computePatch(patch, rcParams, tcParams, depthPlane, icoords);

                // we do not need positive and filtered similarity values
                constexpr bool invertAndFilter = false;

                float fsim;

                // compute patch similarity
                fsim = compNCCby3DptsYK_customPatchPattern<invertAndFilter>(rcParams,
                                                                            tcParams,
                                                                            rcDeviceMipmapAcc,
                                                                            tcDeviceMipmapAcc,
                                                                            rcLevelDim,
                                                                            tcLevelDim,
                                                                            rcMipmapLevel,
                                                                            invGammaC,
                                                                            invGammaP,
                                                                            useConsistentScale,
                                                                            patch,
                                                                            patchPattern);

                if(fsim == std::numeric_limits<float>::infinity()) // invalid similarity
                {
                    fsim = 255.0f; // 255 is the invalid similarity value
                }
                else // valid similarity
                {
                    // remap similarity value
                    constexpr const float fminVal = -1.0f;
                    constexpr const float fmaxVal = 1.0f;
                    constexpr const float fmultiplier = 1.0f / (fmaxVal - fminVal);

                    fsim = (fsim - fminVal) * fmultiplier;

#ifdef TSIM_USE_FLOAT
                    // no clamp
#else
                    fsim = sycl::clamp(fsim, 0.f, 1.f);
#endif
                    // convert from (0, 1) to (0, 254)
                    // needed to store in the volume in uchar
                    // 255 is reserved for the similarity initialization, i.e. undefined values
                    fsim *= 254.0f;
                }

                TSim& fsim_1st = out_volBestSim_acc(v);
                TSim& fsim_2nd = out_volSecBestSim_acc(v);

                if(fsim < fsim_1st)
                {
                    fsim_2nd = fsim_1st;
                    fsim_1st = TSim(fsim);
                }
                else if(fsim < fsim_2nd)
                {
                    fsim_2nd = TSim(fsim);
                }
            });
        });
    }
    else
    {
        return queue.submit([&](sycl::handler& h) {
            h.depends_on(prerequisite);

            h.parallel_for(sycl::range<1>(roi.width() * roi.height() * depthRange.size()), [=](sycl::id<1> id) {

                // corresponding volume coordinates
                const sycl::uint3 v = sycl::uint3(id[0] / (depthRange.size() * roi.height()),
                                                  id[0] / depthRange.size() % roi.height(),
                                                  id[0] % depthRange.size() + depthRange.begin);

                // corresponding image coordinates
                const sycl::float2 icoords = (sycl::uint2(v.x() + roi.x.begin, v.y() + roi.y.begin) * stepXY).convert<float>();

                // corresponding depth plane
                const float depthPlane = __readonly_load(&in_depths_acc(v.z()));

                // compute patch
                Patch patch;
                volume_computePatch(patch, rcParams, tcParams, depthPlane, icoords);

                // we do not need positive and filtered similarity values
                constexpr bool invertAndFilter = false;

                float fsim;

                // compute patch similarity
                fsim = compNCCby3DptsYK<invertAndFilter>(rcParams,
                                                         tcParams,
                                                         rcDeviceMipmapAcc,
                                                         tcDeviceMipmapAcc,
                                                         rcLevelDim,
                                                         tcLevelDim,
                                                         rcMipmapLevel,
                                                         wsh,
                                                         invGammaC,
                                                         invGammaP,
                                                         useConsistentScale,
                                                         patch);

                if(fsim == std::numeric_limits<float>::infinity()) // invalid similarity
                {
                    fsim = 255.0f; // 255 is the invalid similarity value
                }
                else // valid similarity
                {
                    // remap similarity value
                    constexpr const float fminVal = -1.0f;
                    constexpr const float fmaxVal = 1.0f;
                    constexpr const float fmultiplier = 1.0f / (fmaxVal - fminVal);

                    fsim = (fsim - fminVal) * fmultiplier;

#ifdef TSIM_USE_FLOAT
                    // no clamp
#else
                    fsim = sycl::clamp(fsim, 0.f, 1.f);
#endif
                    // convert from (0, 1) to (0, 254)
                    // needed to store in the volume in uchar
                    // 255 is reserved for the similarity initialization, i.e. undefined values
                    fsim *= 254.0f;
                }

                TSim& fsim_1st = out_volBestSim_acc(v);
                TSim& fsim_2nd = out_volSecBestSim_acc(v);

                if(fsim < fsim_1st)
                {
                    fsim_2nd = fsim_1st;
                    fsim_1st = TSim(fsim);
                }
                else if(fsim < fsim_2nd)
                {
                    fsim_2nd = TSim(fsim);
                }
            });
        });
    }
}

sycl::event sycl_volumeRefineSimilarity(SyclDeviceMemoryPitched<TSimRefine, 3>& inout_volSim_dmp,
                                        const SyclDeviceMemoryPitched<sycl::float2, 2>& in_sgmDepthPixSizeMap_dmp,
                                        const SyclDeviceMemoryPitched<sycl::float3, 2>* in_sgmNormalMap_dmp,
                                        const CameraParams& rcParams,
                                        const CameraParams& tcParams,
                                        const DeviceMipmapImage& rcDeviceMipmapImage,
                                        const DeviceMipmapImage& tcDeviceMipmapImage,
                                        const depthMapCommon::RefineParams& refineParams,
                                        const Range& depthRange,
                                        const ROI& roi,
                                        sycl::queue& queue, sycl::event prerequisite)
{
    // get mipmap images level and dimensions
    const float rcMipmapLevel = rcDeviceMipmapImage.getLevel(refineParams.scale);
    const SyclSize<2> rcLevelDim_h = rcDeviceMipmapImage.getDimensions(refineParams.scale);
    const sycl::uint2 rcLevelDim = sycl::uint2(rcLevelDim_h[0], rcLevelDim_h[1]);
    const SyclSize<2> tcLevelDim_h = tcDeviceMipmapImage.getDimensions(refineParams.scale);
    const sycl::uint2 tcLevelDim = sycl::uint2(tcLevelDim_h[0], tcLevelDim_h[1]);

    const bool useNormalMap = in_sgmNormalMap_dmp != nullptr;

    // get image access
    SyclDevicePitchedAccess<TSimRefine, 3> inout_volSim_acc(inout_volSim_dmp);
    const SyclDevicePitchedAccess<sycl::float2, 2> in_sgmDepthPixSizeMap_acc(in_sgmDepthPixSizeMap_dmp);
    const SyclDevicePitchedAccess<sycl::float3, 2> in_sgmNormalMap_acc = [&] () {
        if(useNormalMap)
        {
            const SyclDeviceMemoryPitched<sycl::float3, 2>& nrm = *in_sgmNormalMap_dmp;
            return SyclDevicePitchedAccess<sycl::float3, 2>{nrm};
        }
        // else
        return SyclDevicePitchedAccess<sycl::float3, 2>{SyclDeviceMemoryPitched<sycl::float3, 2>{queue}}; // Dummy object that we don't care about. Trying to access it will result in a nullptr dereference
    } ();
    const MipmapImageAccess rcDeviceMipmapAcc(rcDeviceMipmapImage);
    const MipmapImageAccess tcDeviceMipmapAcc(tcDeviceMipmapImage);

    // bits we can calculate outside of the kernel
    const int volDimZ = inout_volSim_dmp.getSize().z();
    const int stepXY = refineParams.stepXY;
    const int wsh = refineParams.wsh;
    const bool useConsistentScale = refineParams.useConsistentScale;
    const float invGammaC = (1.f / float(refineParams.gammaC)); // inverted gammaC
    const float invGammaP = (1.f / float(refineParams.gammaP)); // inverted gammaP

    // kernel launch
    if(refineParams.useCustomPatchPattern)
    {
        const PatchPattern& patchPattern = PatchPattern::getGlobalPatchPattern();

        return queue.submit([&](sycl::handler& h) {
            h.depends_on(prerequisite);

            h.parallel_for(sycl::range<1>(roi.width() * roi.height() * depthRange.size()), [=](sycl::id<1> id) {

                // corresponding volume coordinates
                const sycl::uint3 v = sycl::uint3(id[0] / (depthRange.size() * roi.height()),
                                                  id[0] / depthRange.size() % roi.height(),
                                                  id[0] % depthRange.size() + depthRange.begin);

                // corresponding image coordinates
                const sycl::float2 icoords = sycl::float2(roi.x.begin + v.x(), roi.y.begin + v.y()) * float(stepXY);

                // corresponding input sgm depth/pixSize (middle depth)
                const sycl::float2 in_sgmDepthPixSize = __readonly_load(&in_sgmDepthPixSizeMap_acc(sycl::uint2(v.x(), v.y())));

                // sgm depth (middle depth) invalid or masked
                if(in_sgmDepthPixSize.x() <= 0.0f)
                    return;

                // initialize rc 3d point at sgm depth (middle depth)
                sycl::float3 p = get3DPointForPixelAndDepthFromRC(rcParams, icoords, in_sgmDepthPixSize.x());

                // compute relative depth index offset from z center
                const int relativeDepthIndexOffset = v.z() - ((volDimZ - 1) / 2);

                if(relativeDepthIndexOffset != 0)
                {
                    // not z center
                    // move rc 3d point by relative depth index offset * sgm pixSize
                    const float pixSizeOffset = relativeDepthIndexOffset * in_sgmDepthPixSize.y(); // input sgm pixSize
                    move3DPointByRcPixSize(p, rcParams, pixSizeOffset);
                }

                // compute patch
                Patch patch;
                patch.p = p;
                patch.d = computePixSize(rcParams, p);

                // computeRotCSEpip
                {
                    // vector from the reference camera to the 3d point
                    sycl::float3 v1 = sycl::normalize(rcParams.C - patch.p);
                    // vector from the target camera to the 3d point
                    sycl::float3 v2 = sycl::normalize(tcParams.C - patch.p);

                    // y has to be orthogonal to the epipolar plane
                     // n has to be on the epipolar plane
                    // x has to be on the epipolar plane

                    patch.y = sycl::normalize(sycl::cross(v1, v2));

                    // initialize patch normal from input normal map
                    if(useNormalMap)
                        patch.n = __readonly_load(&in_sgmNormalMap_acc(sycl::uint2(v.x(), v.y())));
                    else
                        patch.n = sycl::normalize(v1 + v2);

                    patch.x = sycl::normalize(sycl::cross(patch.y, patch.n));
                }

                // we need positive and filtered similarity values
                constexpr bool invertAndFilter = true;

                float fsimInvertedFiltered;

                // compute similarity
                fsimInvertedFiltered =
                    compNCCby3DptsYK_customPatchPattern<invertAndFilter>(rcParams,
                                                                         tcParams,
                                                                         rcDeviceMipmapAcc,
                                                                         tcDeviceMipmapAcc,
                                                                         rcLevelDim,
                                                                         tcLevelDim,
                                                                         rcMipmapLevel,
                                                                         invGammaC,
                                                                         invGammaP,
                                                                         useConsistentScale,
                                                                         patch,
                                                                         patchPattern);

                if(fsimInvertedFiltered == std::numeric_limits<float>::infinity()) // invalid similarity
                {
                    // do nothing
                    return;
                }

                // get output similarity pointer
                TSimRefine& outSimRef = inout_volSim_acc(v);

                // add the output similarity value
#ifdef TSIM_REFINE_USE_HALF
                // note: using built-in half addition can give bad results on some gpus
                outSimRef = TSimRefine(float(outSimRef) + fsimInvertedFiltered); // perform the addition in float
#else
                outSimRef += TSimRefine(fsimInvertedFiltered);
#endif
            });
        });
    }
    else // No custom patch pattern
    {
        return queue.submit([&](sycl::handler& h) {
            h.depends_on(prerequisite);

            h.parallel_for(sycl::range<1>(roi.width() * roi.height() * depthRange.size()), [=](sycl::id<1> id) {

                // corresponding volume coordinates
                const sycl::uint3 v = sycl::uint3(id[0] / (depthRange.size() * roi.height()),
                                                  id[0] / depthRange.size() % roi.height(),
                                                  id[0] % depthRange.size() + depthRange.begin);

                // corresponding image coordinates
                const sycl::float2 icoords = sycl::float2(roi.x.begin + v.x(), roi.y.begin + v.y()) * float(stepXY);

                // corresponding input sgm depth/pixSize (middle depth)
                const sycl::float2 in_sgmDepthPixSize = __readonly_load(&in_sgmDepthPixSizeMap_acc(sycl::uint2(v.x(), v.y())));

                // sgm depth (middle depth) invalid or masked
                if(in_sgmDepthPixSize.x() <= 0.0f)
                    return;

                // initialize rc 3d point at sgm depth (middle depth)
                sycl::float3 p = get3DPointForPixelAndDepthFromRC(rcParams, icoords, in_sgmDepthPixSize.x());

                // compute relative depth index offset from z center
                const int relativeDepthIndexOffset = v.z() - ((volDimZ - 1) / 2);

                if(relativeDepthIndexOffset != 0)
                {
                    // not z center
                    // move rc 3d point by relative depth index offset * sgm pixSize
                    const float pixSizeOffset = relativeDepthIndexOffset * in_sgmDepthPixSize.y(); // input sgm pixSize
                    move3DPointByRcPixSize(p, rcParams, pixSizeOffset);
                }

                // compute patch
                Patch patch;
                patch.p = p;
                patch.d = computePixSize(rcParams, p);

                // computeRotCSEpip
                {
                    // vector from the reference camera to the 3d point
                    sycl::float3 v1 = sycl::normalize(rcParams.C - patch.p);
                    // vector from the target camera to the 3d point
                    sycl::float3 v2 = sycl::normalize(tcParams.C - patch.p);

                    // y has to be orthogonal to the epipolar plane
                     // n has to be on the epipolar plane
                    // x has to be on the epipolar plane

                    patch.y = sycl::normalize(sycl::cross(v1, v2));

                    // initialize patch normal from input normal map
                    if(useNormalMap)
                        patch.n = __readonly_load(&in_sgmNormalMap_acc(sycl::uint2(v.x(), v.y())));
                    else
                        patch.n = sycl::normalize(v1 + v2);

                    patch.x = sycl::normalize(sycl::cross(patch.y, patch.n));
                }

                // we need positive and filtered similarity values
                constexpr bool invertAndFilter = true;

                float fsimInvertedFiltered;

                // compute similarity
                fsimInvertedFiltered =
                    compNCCby3DptsYK<invertAndFilter>(rcParams,
                                                      tcParams,
                                                      rcDeviceMipmapAcc,
                                                      tcDeviceMipmapAcc,
                                                      rcLevelDim,
                                                      tcLevelDim,
                                                      rcMipmapLevel,
                                                      wsh,
                                                      invGammaC,
                                                      invGammaP,
                                                      useConsistentScale,
                                                      patch);

                if(fsimInvertedFiltered == std::numeric_limits<float>::infinity()) // invalid similarity
                {
                    // do nothing
                    return;
                }

                // get output similarity pointer
                TSimRefine& outSimRef = inout_volSim_acc(v);

                // add the output similarity value
#ifdef TSIM_REFINE_USE_HALF
                // note: using built-in half addition can give bad results on some gpus
                outSimRef = TSimRefine(float(outSimRef) + fsimInvertedFiltered); // perform the addition in float
#else
                outSimRef += TSimRefine(fsimInvertedFiltered);
#endif
            });
        });
    }
}

// helper function
template<typename inType, typename outType>
inline sycl::event volumeCopyYSlice(SyclDevicePitchedAccess<outType, 2>& out_2dImg,
                                    const SyclDevicePitchedAccess<inType, 3>& in_3dVol,
                                    const unsigned int y,
                                    const sycl::range<2>& range,
                                    const sycl::uint3& axisT,
                                    sycl::queue& queue, sycl::event prerequisite)
{
    return queue.submit([&](sycl::handler& h) {
        h.depends_on(prerequisite);

        h.parallel_for(range, [=](sycl::id<2> id) {
            const uint x = id[0];
            const uint z = id[1];
            sycl::uint3 v;
            v[axisT.x()] = x;
            v[axisT.y()] = y;
            v[axisT.z()] = z;
            out_2dImg(sycl::uint2(x, z)) = __readonly_load(&in_3dVol(v));
        });
     });
}

// helper function
template<typename T>
inline sycl::event volumeInitYSlice(SyclDevicePitchedAccess<T, 3>& volume,
                                    const unsigned int y,
                                    const T& cst,
                                    const sycl::range<2>& range,
                                    const sycl::uint3& axisT,
                                    sycl::queue& queue, sycl::event prerequisite)
{
    return queue.submit([&](sycl::handler& h) {
        h.depends_on(prerequisite);

        h.parallel_for(range, [=](sycl::id<2> id) {
            sycl::uint3 v;
            v[axisT.x()] = id[0];
            v[axisT.y()] = y;
            v[axisT.z()] = id[1];
            volume(v) = cst;
        });
     });
}

sycl::event sycl_volumeAggregatePath(SyclDeviceMemoryPitched<TSim, 3>& out_volAgr_dmp,
                                     SyclDeviceMemoryPitched<TSimAcc, 2>& inout_volSliceAccA_dmp,
                                     SyclDeviceMemoryPitched<TSimAcc, 2>& inout_volSliceAccB_dmp,
                                     SyclDeviceMemoryPitched<TSimAcc, 1>& inout_volAxisAcc_dmp,
                                     const SyclDeviceMemoryPitched<TSim, 3>& in_volSim_dmp,
                                     const DeviceMipmapImage& rcDeviceMipmapImage,
                                     const SyclSize<2>& rcLevelDim_h,
                                     const float rcMipmapLevel,
                                     const SyclSize<3>& axisT_S,
                                     const depthMapCommon::SgmParams& sgmParams,
                                     const int lastDepthIndex,
                                     const int filteringIndex,
                                     const bool invY,
                                     const ROI& roi,
                                     sycl::queue& queue, sycl::event prerequisite)
{
    SyclSize<3> volDim_S = in_volSim_dmp.getSize();
    volDim_S[2] = lastDepthIndex; // override volume depth, use rc depth list last index

    // Various constants, including some we copy to device
    const size_t volDimX = volDim_S[axisT_S[0]];
    const size_t volDimY = volDim_S[axisT_S[1]];
    const size_t volDimZ = volDim_S[axisT_S[2]];

    const sycl::uint3 volDim = sycl::uint3(volDim_S[0], volDim_S[1], volDim_S[2]);
    const sycl::uint3 axisT = sycl::uint3(axisT_S[0], axisT_S[1], axisT_S[2]);
    const int ySign = (invY ? -1 : 1);

    const uint roiXbegin = roi.x.begin;
    const uint roiYbegin = roi.y.begin;

    const float P1 = sgmParams.p1;
    const float _P2 = sgmParams.p2Weighting;
    const int step = sgmParams.stepXY;

    // find texture offset
    const int beginX = (axisT.x() == 0) ? roiXbegin : roiYbegin;
    const int beginY = (axisT.x() == 0) ? roiYbegin : roiXbegin;

    // get mipmap images level and dimensions
    const sycl::uint2 rcLevelDim = sycl::uint2(rcLevelDim_h[0], rcLevelDim_h[1]);
    const sycl::float2 rcInvLevelDim = 1.f / rcLevelDim.convert<float>();

    // setup ranges
    const sycl::range<2> VolXZ(volDimX, volDimZ);
    const sycl::range<1> ColZ(volDimX);

    // Pointers
    SyclDeviceMemoryPitched<TSimAcc, 2>* xzSliceForY_dmpPtr   = &inout_volSliceAccA_dmp; // Y slice
    SyclDeviceMemoryPitched<TSimAcc, 2>* xzSliceForYm1_dmpPtr = &inout_volSliceAccB_dmp; // Y-1 slice
    SyclDeviceMemoryPitched<TSimAcc, 1>& bestSimInYm1_dmp  = inout_volAxisAcc_dmp;   // best sim score along the Y axis for each Z value

    // Accessors
    // Use a map so we can swap pointers without recreating device accessor objects
    SyclDevicePitchedAccess out_volAgr_acc = SyclDevicePitchedAccess(out_volAgr_dmp);
    std::map<SyclDeviceMemoryPitched<TSimAcc, 2>*, SyclDevicePitchedAccess<TSimAcc, 2>> xzSliceAccs {
        { xzSliceForY_dmpPtr, SyclDevicePitchedAccess(*xzSliceForY_dmpPtr) },
        { xzSliceForYm1_dmpPtr, SyclDevicePitchedAccess(*xzSliceForYm1_dmpPtr) },
    };
    SyclDevicePitchedAccess bestSimInYm1_acc = SyclDevicePitchedAccess(bestSimInYm1_dmp);
    const SyclDevicePitchedAccess in_volSim_acc = SyclDevicePitchedAccess(in_volSim_dmp);
    const MipmapImageAccess rcMipmapImage_acc = MipmapImageAccess(rcDeviceMipmapImage);

    // Copy the first XZ plane (at Y=0) from 'in_volSim_dmp' into 'xzSliceForYm1_dmpPtr'
    prerequisite = volumeCopyYSlice(xzSliceAccs.at(xzSliceForYm1_dmpPtr),
                                    in_volSim_acc,
                                    0, /* Y = 0 */
                                    VolXZ,
                                    axisT,
                                    queue, prerequisite);

    // Set the first Y plane from 'out_volAgr_dmp' to 255
    prerequisite = volumeInitYSlice(out_volAgr_acc,
                                    0, /* Y = 0 */
                                    TSim(255),
                                    VolXZ,
                                    axisT,
                                    queue, prerequisite);

    for(int iy = 1; iy < volDimY; ++iy)
    {
        const int y = invY ? volDimY - 1 - iy : iy;
        SyclDevicePitchedAccess<TSimAcc, 2>& yAcc = xzSliceAccs.at(xzSliceForY_dmpPtr);
        SyclDevicePitchedAccess<TSimAcc, 2>& ym1Acc = xzSliceAccs.at(xzSliceForYm1_dmpPtr);

        // For each column: compute the best score
        // Foreach x:
        //   bestSimInYm1[x] = min(d_xzSliceForY[1:height])
        prerequisite = queue.submit([&](sycl::handler& h) {
            h.depends_on(prerequisite);

            h.parallel_for(ColZ, [=](sycl::id<1> id) {
                TSimAcc bestCst = __readonly_load(&ym1Acc(sycl::uint2(id[0], 0)));

                for(int z = 1; z < volDimZ; ++z)
                {
                    const TSimAcc cst = __readonly_load(&ym1Acc(sycl::uint2(id[0], z)));
                    bestCst = sycl::min(bestCst, cst);
                }
                bestSimInYm1_acc(id[0]) = bestCst;
            });
        });

        // Copy the 'y' plane from 'in_volSim_dmp' into 'xzSliceForY'
        prerequisite = volumeCopyYSlice(yAcc,
                                        in_volSim_acc,
                                        y,
                                        VolXZ,
                                        axisT,
                                        queue, prerequisite);

        prerequisite = queue.submit([&](sycl::handler& h) {
            h.depends_on(prerequisite);

            h.parallel_for(VolXZ, [=](sycl::id<2> id) {
                const int x = id[0];
                const int z = id[1];
                sycl::uint3 v;
                v[axisT.x()] = x;
                v[axisT.y()] = y;
                v[axisT.z()] = z;
                const sycl::uint2 c(x, z);

                TSimAcc& sim_xz = yAcc(c);
                float pathCost = 255.0f;

                float P2 = 0;

                if(z >= 1 && z < volDimZ - 1)
                {
                    if(_P2 < 0)
                    {
                        // _P2 convention: use negative value to skip the use of deltaC.
                        P2 = sycl::fabs(_P2);
                    }
                    else
                    {
                        const int imX0 = (beginX + v.x()) * step; // current
                        const int imY0 = (beginY + v.y()) * step;

                        const int imX1 = imX0 - ySign * step * (axisT.y() == 0); // M1
                        const int imY1 = imY0 - ySign * step * (axisT.y() == 1);

                        const sycl::float4 gcr0 = rcMipmapImage_acc.trilinear(sycl::float2(float(imX0), float(imY0))*rcInvLevelDim, rcMipmapLevel).convert<float>();
                        const sycl::float4 gcr1 = rcMipmapImage_acc.trilinear(sycl::float2(float(imX1), float(imY1))*rcInvLevelDim, rcMipmapLevel).convert<float>();
                        const float deltaC = sycl::distance(sycl::float3(gcr0.x(), gcr0.y(), gcr0.z()),
                                                            sycl::float3(gcr1.x(), gcr1.y(), gcr1.z()));

                        // sigmoid f(x) = i + (a - i) * (1 / ( 1 + e^(10 * (x - P2) / w)))
                        // see: https://www.desmos.com/calculator/1qvampwbyx
                        // best values found from tests: i = 80, a = 255, w = 80, P2 = 100
                        // historical values: i = 15, a = 255, w = 80, P2 = 20
                        P2 = sigmoid(80.f, 255.f, 80.f, _P2, deltaC);
                    }

                    const TSimAcc bestCostInColM1 = __readonly_load(&bestSimInYm1_acc(x));
                    const TSimAcc pathCostMDM1 = __readonly_load(&ym1Acc(sycl::uint2(x, z - 1))); // M1: minus 1 over depths
                    const TSimAcc pathCostMD = __readonly_load(&ym1Acc(c));
                    const TSimAcc pathCostMDP1 = __readonly_load(&ym1Acc(sycl::uint2(x, z + 1))); // P1: plus 1 over depths
                    const float minCost = multi_fmin(float(pathCostMD),
                                                     float(pathCostMDM1 + P1),
                                                     float(pathCostMDP1 + P1),
                                                     float(bestCostInColM1 + P2));

                    // if 'pathCostMD' is the minimal value of the depth
                    pathCost = sim_xz + minCost - bestCostInColM1;
                }

                // fill the current slice with the new similarity score
                sim_xz = TSimAcc(pathCost);

#ifndef TSIM_USE_FLOAT
                // clamp if TSim = uchar (TSimAcc = unsigned int)
                pathCost = sycl::clamp(pathCost, 0.0f, 255.0f);
#endif

                // aggregate into the final output
                TSim& volume_xyz = out_volAgr_acc(v);
                volume_xyz = TSim(float(volume_xyz) * float(filteringIndex) + pathCost) / float(filteringIndex + 1);
            });
        });

        std::swap(xzSliceForYm1_dmpPtr, xzSliceForY_dmpPtr);
    }
    return prerequisite;
}

sycl::event sycl_volumeOptimize(SyclDeviceMemoryPitched<TSim, 3>& out_volSimFiltered_dmp,
                                SyclDeviceMemoryPitched<TSimAcc, 2>& inout_volSliceAccA_dmp,
                                SyclDeviceMemoryPitched<TSimAcc, 2>& inout_volSliceAccB_dmp,
                                SyclDeviceMemoryPitched<TSimAcc, 1>& inout_volAxisAcc_dmp,
                                const SyclDeviceMemoryPitched<TSim, 3>& in_volSim_dmp,
                                const DeviceMipmapImage& rcDeviceMipmapImage,
                                const depthMapCommon::SgmParams& sgmParams,
                                const int lastDepthIndex,
                                const ROI& roi,
                                sycl::queue& queue, sycl::event prerequisite)
{
    // get R mipmap image level and dimensions
    const float rcMipmapLevel = rcDeviceMipmapImage.getLevel(sgmParams.scale);
    const SyclSize<2> rcLevelDim = rcDeviceMipmapImage.getDimensions(sgmParams.scale);

    // update aggregation volume
    int npaths = 0;
    const auto updateAggrVolume = [&](const SyclSize<3>& axisT, bool invX)
    {
        prerequisite = sycl_volumeAggregatePath(out_volSimFiltered_dmp,
                                                inout_volSliceAccA_dmp,
                                                inout_volSliceAccB_dmp,
                                                inout_volAxisAcc_dmp,
                                                in_volSim_dmp,
                                                rcDeviceMipmapImage,
                                                rcLevelDim,
                                                rcMipmapLevel,
                                                axisT,
                                                sgmParams,
                                                lastDepthIndex,
                                                npaths,
                                                invX,
                                                roi,
                                                queue, prerequisite);
        npaths++;
    };

    // filtering is done on the last axis
    const std::map<char, SyclSize<3>> mapAxes = {
        {'X', {1, 0, 2}}, // XYZ -> YXZ
        {'Y', {0, 1, 2}}, // XYZ
    };

    constexpr char filteringAxes[2] = {'Y', 'X'};
    for(char axis : filteringAxes)
    {
        const SyclSize<3>& axisT = mapAxes.at(axis);
        updateAggrVolume(axisT, false); // without transpose
        updateAggrVolume(axisT, true);  // with transpose of the last axis
    }
    return prerequisite;
}

sycl::event sycl_volumeRetrieveBestDepthUseSim(SyclDeviceMemoryPitched<sycl::float2, 2>& out_sgmDepthThicknessMap_dmp,
                                               SyclDeviceMemoryPitched<sycl::float2, 2>& out_sgmDepthSimMap_dmp,
                                               const SyclDeviceMemoryPitched<float, 1>& in_depths_dmp,
                                               const SyclDeviceMemoryPitched<TSim, 3>& in_volSim_dmp,
                                               const CameraParams& rcParams,
                                               const depthMapCommon::SgmParams& sgmParams,
                                               const Range& depthRange,
                                               const ROI& roi,
                                               sycl::queue& queue, sycl::event prerequisite)
{
    // constant kernel inputs
    const int scaleStep = sgmParams.scale * sgmParams.stepXY;
    const float thicknessMultFactor = 1.f + float(sgmParams.depthThicknessInflate);
    const float maxSimilarity = float(sgmParams.maxSimilarity) * 254.f; // convert from (0, 1) to (0, 254)
    const uint roiXbegin = roi.x.begin;
    const uint roiYbegin = roi.y.begin;
    const int volDimZ = int(in_volSim_dmp.getSize().z());

    // Accessors
    SyclDevicePitchedAccess out_sgmDepthThicknessMap_acc = SyclDevicePitchedAccess(out_sgmDepthThicknessMap_dmp);
    SyclDevicePitchedAccess out_sgmDepthSimMap_acc = SyclDevicePitchedAccess(out_sgmDepthSimMap_dmp);
    const SyclDevicePitchedAccess in_depths_acc = SyclDevicePitchedAccess(in_depths_dmp);
    const SyclDevicePitchedAccess in_volSim_acc = SyclDevicePitchedAccess(in_volSim_dmp);

    // kernel execution
    return queue.submit([&](sycl::handler& h) {
        h.depends_on(prerequisite);

        h.parallel_for(sycl::range(roi.width(), roi.height()), [=](sycl::id<2> id) {
            const sycl::uint2 v = sycl::uint2(id[0], id[1]);

            // corresponding image coordinates
            const sycl::float2 pix = ((sycl::uint2(roiXbegin, roiYbegin) + v)*scaleStep).convert<float>();

            // corresponding output depth/thickness pointer
            sycl::float2& out_bestDepthThicknessRef = out_sgmDepthThicknessMap_acc(v);

            // corresponding output depth/sim pointer
            sycl::float2& out_bestDepthSimRef = out_sgmDepthSimMap_acc(v);

            // find the best depth plane index for the current pixel
            // the best depth plane has the best similarity value
            // - best possible similarity value is 0
            // - worst possible similarity value is 254
            // - invalid similarity value is 255
            float bestSim = 255.f;
            int bestZIdx = -1;

            for(int vz = depthRange.begin; vz < depthRange.end; ++vz)
            {
                const sycl::uint3 v3 = sycl::uint3(v.x(), v.y(), vz);
                const float simAtZ = __readonly_load(&in_volSim_acc(v3));

                if(simAtZ < bestSim)
                {
                    bestSim = simAtZ;
                    bestZIdx = vz;
                }
            }

            // filtering out invalid values and values with a too bad score (above the user maximum similarity threshold)
            // note: this helps to reduce following calculations and also the storage volume of the depth maps.
            if((bestZIdx == -1) || (bestSim > maxSimilarity))
            {
                out_bestDepthThicknessRef[0] = -1.f; // invalid depth
                out_bestDepthThicknessRef[1] = -1.f; // invalid thickness

                out_bestDepthSimRef[0] = -1.f; // invalid depth
                out_bestDepthSimRef[1] =  1.f; // worst similarity value

                return;
            }

            // find best depth plane previous and next indexes
            const int bestZIdx_m1 = sycl::max(0, bestZIdx - 1);           // best depth plane previous index
            const int bestZIdx_p1 = sycl::min(volDimZ - 1, bestZIdx + 1); // best depth plane next index

            // get best best depth current, previous and next plane depth values
            // note: float3 struct is useful for depth interpolation
            sycl::float3 depthPlanes;
            depthPlanes.x() = __readonly_load(&in_depths_acc(bestZIdx_m1));  // best depth previous plane
            depthPlanes.y() = __readonly_load(&in_depths_acc(bestZIdx));     // best depth plane
            depthPlanes.z() = __readonly_load(&in_depths_acc(bestZIdx_p1));  // best depth next plane

            const float bestDepth_m1 = depthPlaneToDepth(rcParams, depthPlanes.x(), pix); // previous best depth
            const float bestDepth    = depthPlaneToDepth(rcParams, depthPlanes.y(), pix); // best depth
            const float bestDepth_p1 = depthPlaneToDepth(rcParams, depthPlanes.z(), pix); // next best depth

#ifdef ALICEVISION_DEPTHMAP_RETRIEVE_BEST_Z_INTERPOLATION
            // with depth/sim interpolation
            // note: disable by default

            sycl::float3 sims;
            sims.x() = __readonly_load(&in_volSim_acc(sycl::vec3(v.x(), v.y(), bestZIdx_m1)));
            sims.y() = __readonly_load(&bestSim);
            sims.z() = __readonly_load(&in_volSim_acc(sycl::vec3(v.x(), v.y(), bestZIdx_p1)));

            // convert sims from (0, 255) to (-1, +1)
            sims = (sims / 255.0f) * 2.0f - 1.0f;

            // interpolation between the 3 depth planes candidates
            const float refinedDepthPlane = refineDepthSubPixel(depthPlanes, sims);

            const float out_bestDepth = depthPlaneToDepth(rcParams, refinedDepthPlane, pix);
            const float out_bestSim = sims.y();
#else
            // without depth interpolation
            const float out_bestDepth = bestDepth;
            const float out_bestSim = (bestSim / 255.0f) * 2.0f - 1.0f; // convert from (0, 255) to (-1, +1)
#endif

            // compute output best depth thickness
            // thickness is the maximum distance between output best depth and previous or next depth
            // thickness can be inflate with thicknessMultFactor
            const float out_bestDepthThickness = sycl::fmax(bestDepth_p1 - out_bestDepth, out_bestDepth - bestDepth_m1) * thicknessMultFactor;

            // write output depth/thickness
            out_bestDepthThicknessRef.x() = out_bestDepth;
            out_bestDepthThicknessRef.y() = out_bestDepthThickness;

            // write output depth/sim
            out_bestDepthSimRef.x() = out_bestDepth;
            out_bestDepthSimRef.y() = out_bestSim;
        });
    });
}

sycl::event sycl_volumeRetrieveBestDepthNoSim(SyclDeviceMemoryPitched<sycl::float2, 2>& out_sgmDepthThicknessMap_dmp,
                                              const SyclDeviceMemoryPitched<float, 1>& in_depths_dmp,
                                              const SyclDeviceMemoryPitched<TSim, 3>& in_volSim_dmp,
                                              const CameraParams& rcParams,
                                              const depthMapCommon::SgmParams& sgmParams,
                                              const Range& depthRange,
                                              const ROI& roi,
                                              sycl::queue& queue, sycl::event prerequisite)
{
    // constant kernel inputs
    const int scaleStep = sgmParams.scale * sgmParams.stepXY;
    const float thicknessMultFactor = 1.f + float(sgmParams.depthThicknessInflate);
    const float maxSimilarity = float(sgmParams.maxSimilarity) * 254.f; // convert from (0, 1) to (0, 254)
    const uint roiXbegin = roi.x.begin;
    const uint roiYbegin = roi.y.begin;
    const int volDimZ = int(in_volSim_dmp.getSize().z());

    // Accessors
    SyclDevicePitchedAccess out_sgmDepthThicknessMap_acc = SyclDevicePitchedAccess(out_sgmDepthThicknessMap_dmp);
    const SyclDevicePitchedAccess in_depths_acc = SyclDevicePitchedAccess(in_depths_dmp);
    const SyclDevicePitchedAccess in_volSim_acc = SyclDevicePitchedAccess(in_volSim_dmp);

    // kernel execution
    return queue.submit([&](sycl::handler& h) {
        h.depends_on(prerequisite);

        h.parallel_for(sycl::range(roi.width(), roi.height()), [=](sycl::id<2> id) {
            const sycl::uint2 v = sycl::uint2(id[0], id[1]);

            // corresponding image coordinates
            const sycl::float2 pix = ((sycl::uint2(roiXbegin, roiYbegin) + v)*scaleStep).convert<float>();

            // corresponding output depth/thickness pointer
            sycl::float2& out_bestDepthThicknessRef = out_sgmDepthThicknessMap_acc(v);

            // find the best depth plane index for the current pixel
            // the best depth plane has the best similarity value
            // - best possible similarity value is 0
            // - worst possible similarity value is 254
            // - invalid similarity value is 255
            float bestSim = 255.f;
            int bestZIdx = -1;

            for(int vz = depthRange.begin; vz < depthRange.end; ++vz)
            {
                const sycl::uint3 v3 = sycl::uint3(v.x(), v.y(), vz);
                const float simAtZ = in_volSim_acc(v3);

                if(simAtZ < bestSim)
                {
                    bestSim = simAtZ;
                    bestZIdx = vz;
                }
            }

            // filtering out invalid values and values with a too bad score (above the user maximum similarity threshold)
            // note: this helps to reduce following calculations and also the storage volume of the depth maps.
            if((bestZIdx == -1) || (bestSim > maxSimilarity))
            {
                out_bestDepthThicknessRef[0] = -1.f; // invalid depth
                out_bestDepthThicknessRef[1] = -1.f; // invalid thickness

                return;
            }

            // find best depth plane previous and next indexes
            const int bestZIdx_m1 = sycl::max(0, bestZIdx - 1);           // best depth plane previous index
            const int bestZIdx_p1 = sycl::min(volDimZ - 1, bestZIdx + 1); // best depth plane next index

            // get best best depth current, previous and next plane depth values
            // note: float3 struct is useful for depth interpolation
            sycl::float3 depthPlanes;
            depthPlanes.x() = in_depths_acc(bestZIdx_m1);  // best depth previous plane
            depthPlanes.y() = in_depths_acc(bestZIdx);     // best depth plane
            depthPlanes.z() = in_depths_acc(bestZIdx_p1);  // best depth next plane

            const float bestDepth    = depthPlaneToDepth(rcParams, depthPlanes.y(), pix); // best depth
            const float bestDepth_m1 = depthPlaneToDepth(rcParams, depthPlanes.x(), pix); // previous best depth
            const float bestDepth_p1 = depthPlaneToDepth(rcParams, depthPlanes.z(), pix); // next best depth

#ifdef ALICEVISION_DEPTHMAP_RETRIEVE_BEST_Z_INTERPOLATION
            // with depth/sim interpolation
            // note: disable by default

            sycl::float3 sims;
            sims.x() = in_volSim_acc(sycl::vec3(v.x(), v.y(), bestZIdx_m1));
            sims.y() = bestSim;
            sims.z() = in_volSim_acc(sycl::vec3(v.x(), v.y(), bestZIdx_p1));

            // convert sims from (0, 255) to (-1, +1)
            sims = (sims / 255.0f) * 2.0f - 1.0f;

            // interpolation between the 3 depth planes candidates
            const float refinedDepthPlane = refineDepthSubPixel(depthPlanes, sims);

            const float out_bestDepth = depthPlaneToDepth(rcParams, refinedDepthPlane, pix);
#else
            // without depth interpolation
            const float out_bestDepth = bestDepth;
#endif

            // compute output best depth thickness
            // thickness is the maximum distance between output best depth and previous or next depth
            // thickness can be inflate with thicknessMultFactor
            const float out_bestDepthThickness = sycl::fmax(bestDepth_p1 - out_bestDepth, out_bestDepth - bestDepth_m1) * thicknessMultFactor;

            // write output depth/thickness
            out_bestDepthThicknessRef.x() = out_bestDepth;
            out_bestDepthThicknessRef.y() = out_bestDepthThickness;
        });
    });
}

sycl::event sycl_volumeRefineBestDepth(SyclDeviceMemoryPitched<sycl::float2, 2>& out_refineDepthSimMap_dmp,
                                       const SyclDeviceMemoryPitched<sycl::float2, 2>& in_sgmDepthPixSizeMap_dmp,
                                       const SyclDeviceMemoryPitched<TSimRefine, 3>& in_volSim_dmp,
                                       const depthMapCommon::RefineParams& refineParams,
                                       const ROI& roi,
                                       sycl::queue& queue, sycl::event prerequisite)
{
    // constant kernel inputs
    const int halfNbSamples = refineParams.nbSubsamples * refineParams.halfNbDepths;
    const int halfNbDepths = refineParams.halfNbDepths;
    const int samplesPerPixSize = refineParams.nbSubsamples;
    const float invTwoTimesSigmaPowerTwo = 1.f / float(2.0 * refineParams.sigma * refineParams.sigma);
    const int volDimZ = in_volSim_dmp.getSize().z();

    // Accessors
    SyclDevicePitchedAccess out_refineDepthSimMap_acc = SyclDevicePitchedAccess(out_refineDepthSimMap_dmp);
    const SyclDevicePitchedAccess in_sgmDepthPixSizeMap_acc = SyclDevicePitchedAccess(in_sgmDepthPixSizeMap_dmp);
    const SyclDevicePitchedAccess in_volSim_acc = SyclDevicePitchedAccess(in_volSim_dmp);

    // kernel execution
    return queue.submit([&](sycl::handler& h) {
        h.depends_on(prerequisite);

        h.parallel_for(sycl::range(roi.width(), roi.height()), [=](sycl::id<2> id) {
            const sycl::uint2 v = sycl::uint2(id[0], id[1]);

            // corresponding input sgm depth/pixSize (middle depth)
            const sycl::float2 in_sgmDepthPixSize = in_sgmDepthPixSizeMap_acc(v);

            // corresponding output depth/sim pointer
            sycl::float2& out_bestDepthSimRef = out_refineDepthSimMap_acc(v);

            // sgm depth (middle depth) invalid or masked
            if(in_sgmDepthPixSize.x() <= 0.0f)
            {
                out_bestDepthSimRef[0] = in_sgmDepthPixSize.x();  // -1 (invalid) or -2 (masked)
                out_bestDepthSimRef[1] = 1.0f;                    // similarity between (-1, +1)
                return;
            }

            // find best z sample per pixel
            float bestSampleSim = 0.f;      // all sample sim <= 0.f
            int bestSampleOffsetIndex = 0;  // default is middle depth (SGM)

            // sliding gaussian window
            for(int sample = -halfNbSamples; sample <= halfNbSamples; ++sample)
            {
                float sampleSim = 0.f;

                for(int vz = 0; vz < volDimZ; ++vz)
                {
                    const int rz = (vz - halfNbDepths);    // relative depth index offset
                    const int zs = rz * samplesPerPixSize; // relative sample offset

                    // get the inverted similarity sum value
                    // best value is the HIGHEST
                    // worst value is 0
                    const float invSimSum = in_volSim_acc(sycl::uint3(v.x(), v.y(), vz));

                    // reverse the inverted similarity sum value
                    // best value is the LOWEST
                    // worst value is 0
                    const float simSum = -invSimSum;

                    // apply gaussian
                    // see: https://www.desmos.com/calculator/ribalnoawq
                    sampleSim += simSum * sycl::exp(-sycl::pown(float(zs - sample), 2) * invTwoTimesSigmaPowerTwo);
                }

                if(sampleSim < bestSampleSim)
                {
                    bestSampleOffsetIndex = sample;
                    bestSampleSim = sampleSim;
                }
            }

            // compute sample size
            const float sampleSize = in_sgmDepthPixSize.y() / samplesPerPixSize; // input sgm pixSize / samplesPerPixSize

            // compute sample size offset from z center
            const float sampleSizeOffset = bestSampleOffsetIndex * sampleSize;

            // compute best depth
            // input sgm depth (middle depth) + sample size offset from z center
            const float bestDepth = in_sgmDepthPixSize.x() + sampleSizeOffset;

            // write output best depth/sim
            out_bestDepthSimRef.x() = bestDepth;
            out_bestDepthSimRef.y() = bestSampleSim;
        });
    });
}

} // namespace depthMap_sycl
} // namespace aliceVision
