// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "NormalMapEstimator.hpp"

#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/Timer.hpp>
#include <aliceVision/mvsUtils/fileIO.hpp>
#include <aliceVision/mvsUtils/mapIO.hpp>
#include <aliceVision/depthMap/depthMapUtils.hpp>
#include <aliceVision/depthMap/cuda/host/utils.hpp>
#include <aliceVision/depthMap/cuda/host/DeviceCache.hpp>
#include <aliceVision/depthMap/cuda/planeSweeping/deviceDepthSimilarityMap.hpp>

#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;

namespace aliceVision {
namespace depthMap {

NormalMapEstimator::NormalMapEstimator(const mvsUtils::MultiViewParams &mp)
  : _mp(mp)
{}

void NormalMapEstimator::compute(int cudaDeviceId, const std::vector<int>& cams)
{
    // set the device to use for GPU executions
    // the CUDA runtime API is thread-safe, it maintains per-thread state about the current device 
    setCudaDeviceId(cudaDeviceId);

    DeviceCache& deviceCache = DeviceCache::getInstance();
    deviceCache.build(0, 1); // 0 mipmap image, 1 camera parameters

    for(const int rc : cams)
    {
        const std::string normalMapFilepath = getFileNameFromIndex(_mp, rc, mvsUtils::EFileType::normalMapFiltered);

        if(!fs::exists(normalMapFilepath))
        {
            const system::Timer timer;

            ALICEVISION_LOG_INFO("Compute normal map (rc: " << rc << ")");

            // add R camera parameters to the device cache (device constant memory)
            // no aditional downscale applied, we are working at input depth map resolution
            deviceCache.addCameraParams(rc, 1 /*downscale*/, _mp);

            // get R camera parameters id in device constant memory array
            const int rcDeviceCameraParamsId = deviceCache.requestCameraParamsId(rc, 1 /*downscale*/, _mp);

            // read input depth map
            image::Image<float> in_depthMap;
            mvsUtils::readMap(rc, _mp, mvsUtils::EFileType::depthMapFiltered, in_depthMap);

            // get input depth map width / height
            const int width  = in_depthMap.Width();
            const int height = in_depthMap.Height();

            // default tile parameters, no tiles
            const mvsUtils::TileParams tileParams;

            // fullsize roi
            const ROI roi(0, _mp.getWidth(rc), 0, _mp.getHeight(rc));

            // copy input depth map into depth/sim map in device memory
            // note: we don't need similarity for normal map computation
            //       we use depth/sim map in order to avoid code duplication
            CudaDeviceMemoryPitched<float2, 2> in_depthSimMap_dmp({size_t(width), size_t(height)});
            {
                CudaHostMemoryHeap<float2, 2> in_depthSimMap_hmh(in_depthSimMap_dmp.getSize());

                for(int x = 0; x < width; ++x)
                    for(int y = 0; y < height; ++y)
                        in_depthSimMap_hmh(size_t(x), size_t(y)) = make_float2(in_depthMap(y, x), 1.f);

                in_depthSimMap_dmp.copyFrom(in_depthSimMap_hmh);
            }

            // allocate normal map buffer in device memory
            CudaDeviceMemoryPitched<float3, 2> out_normalMap_dmp(in_depthSimMap_dmp.getSize());

            // compute normal map
            cuda_depthSimMapComputeNormal(out_normalMap_dmp, in_depthSimMap_dmp, rcDeviceCameraParamsId, 1 /*step*/, roi, 0 /*stream*/);

            // write output normal map
            writeNormalMapFiltered(rc, _mp, tileParams, roi, out_normalMap_dmp);

            ALICEVISION_LOG_INFO("Compute normal map (rc: " << rc << ") done in: " << timer.elapsedMs() << " ms.");
        }
    }

    // device cache countains CUDA objects
    // this objects should be destroyed before the end of the program (i.e. the end of the CUDA context)
    DeviceCache::getInstance().clear();
}

} // namespace depthMap
} // namespace aliceVision
