// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/mvsData/Color.hpp>
#include <aliceVision/mvsData/geometry.hpp>
#include <aliceVision/mvsData/Pixel.hpp>
#include <aliceVision/mvsData/Point2d.hpp>
#include <aliceVision/mvsData/Point3d.hpp>
#include <aliceVision/mvsData/Point4d.hpp>
#include <aliceVision/mvsData/Rgb.hpp>
#include <aliceVision/mvsData/StaticVector.hpp>
#include <aliceVision/mvsData/Voxel.hpp>

#include <aliceVision/mvsUtils/ImagesCache.hpp>

#include <aliceVision/depthMap/SgmParams.hpp>
#include <aliceVision/depthMap/RefineParams.hpp>
#include <aliceVision/depthMap/DepthSimMap.hpp>

#include <aliceVision/depthMap/cuda/memory.hpp>
#include <aliceVision/depthMap/cuda/LRUCache.hpp>
#include <aliceVision/depthMap/cuda/normalMapping/DeviceNormalMapper.hpp>
#include <aliceVision/depthMap/cuda/planeSweeping/similarity.hpp>

namespace aliceVision {
namespace depthMap {

/**
 * @class PlaneSweepingCuda
 * @brief Performing plane sweeping for some images.
 */
class PlaneSweepingCuda
{
public:

    mvsUtils::MultiViewParams& _mp;
    mvsUtils::ImagesCache<ImageRGBAf>& _ic;

    PlaneSweepingCuda(mvsUtils::ImagesCache<ImageRGBAf>& _ic, mvsUtils::MultiViewParams& _mp);
    ~PlaneSweepingCuda();

    void computeDepthSimMapVolume(int rc,
        CudaDeviceMemoryPitched<TSim, 3>& volBestSim_dmp,
        CudaDeviceMemoryPitched<TSim, 3>& volSecBestSim_dmp, 
        const std::vector<int>& tCams, 
        const std::vector<Pixel>& rcDepthsTcamsLimits,
        const std::vector<float>& rcDepths,
        const SgmParams& sgmParams);

    bool sgmOptimizeSimVolume(int rc, 
        CudaDeviceMemoryPitched<TSim, 3>& volSimFiltered_dmp, 
        const CudaDeviceMemoryPitched<TSim, 3>& volSim_dmp,
        const CudaSize<3>& volDim,
        const SgmParams& sgmParams);

    void sgmRetrieveBestDepth(int rc, 
        DepthSimMap& bestDepth, 
        const CudaDeviceMemoryPitched<TSim, 3>& volSim_dmp, 
        const CudaSize<3>& volDim,
        const StaticVector<float>& rcDepths, 
        const SgmParams& sgmParams);

    bool refineRcTcDepthMap(int rc, int tc, 
                            StaticVector<float>& inout_depthMap, 
                            StaticVector<float>& out_simMap,
                            const RefineParams& refineParams,
                            int xFrom, int wPart);

    bool fuseDepthSimMapsGaussianKernelVoting(int wPart, int hPart, 
                                              StaticVector<DepthSim>& out_depthSimMap,
                                              const StaticVector<StaticVector<DepthSim>*>& dataMaps,
                                              const RefineParams& refineParams);

    bool optimizeDepthSimMapGradientDescent(int rc, 
                                            StaticVector<DepthSim>& out_depthSimMapOptimized,
                                            const StaticVector<DepthSim>& depthSimMapSgmUpscale,
                                            const StaticVector<DepthSim>& depthSimMapRefinedFused,
                                            const RefineParams& refineParams,
                                            int yFrom, int hPart);

    /* create object to store intermediate data for repeated use */
    DeviceNormalMapper* createNormalMapping();

    /* delete object to store intermediate data for repeated use */
    void deleteNormalMapping(DeviceNormalMapper* m);

    bool computeNormalMap(DeviceNormalMapper* mapping,
                           const std::vector<float>& depthMap,
                           std::vector<ColorRGBf>&   normalMap,
                           int rc, int scale,
                           float igammaC, float igammaP, int wsh);
};

} // namespace depthMap
} // namespace aliceVision
