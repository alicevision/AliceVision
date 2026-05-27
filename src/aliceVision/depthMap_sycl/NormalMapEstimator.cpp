// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "NormalMapEstimator.hpp"

#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/Timer.hpp>
#include <aliceVision/utils/filesIO.hpp>
#include <aliceVision/mvsUtils/fileIO.hpp>
#include <aliceVision/mvsUtils/mapIO.hpp>
#include <aliceVision/depthMap_sycl/depthMapUtils.hpp>
#include <aliceVision/depthMap_sycl/sycl/DeviceCache.hpp>
#include <aliceVision/depthMap_sycl/sycl/CameraParams.hpp>
#include <aliceVision/depthMap_sycl/sycl/planeSweeping/deviceDepthSimilarityMap.hpp>

#include <filesystem>

namespace fs = std::filesystem;

namespace aliceVision {
namespace depthMap_sycl {

NormalMapEstimator::NormalMapEstimator(const mvsUtils::MultiViewParams& mp)
  : _mp(mp)
{}

void NormalMapEstimator::compute(const sycl::device& device, const std::vector<int>& cams)
{
    sycl::queue queue = constructQueue(device);
    for (const int rc : cams)
    {
        const std::string normalMapFilepath = getFileNameFromIndex(_mp, rc, mvsUtils::EFileType::normalMapFiltered);

        if (!utils::exists(normalMapFilepath))
        {
            const system::Timer timer;

            ALICEVISION_LOG_INFO("Compute normal map (rc: " << rc << ")");

            // get R camera parameters
            CameraParams camParams = getCameraParameters(rc, 1 /*downscale*/, _mp);

            // init prerequisite
            sycl::event event{};

            // read input depth map
            image::Image<float> in_depthMap;
            mvsUtils::readMap(rc, _mp, mvsUtils::EFileType::depthMapFiltered, in_depthMap);

            // get input depth map width / height
            const int width = in_depthMap.width();
            const int height = in_depthMap.height();

            // default tile parameters, no tiles
            const mvsUtils::TileParams tileParams;

            // fullsize roi
            const ROI roi(0, _mp.getWidth(rc), 0, _mp.getHeight(rc));

            // keep track of memory success
            bool allocSuccess = true;

            // allocate normal map buffer in device memory
            SyclDeviceMemoryPitched<sycl::float3, 2> out_normalMap_dmp({size_t(width), size_t(height)}, allocSuccess, queue);

            // copy input depth map into depth/sim map in device memory
            // note: we don't need similarity for normal map computation
            //       we use depth/sim map in order to avoid code duplication
            SyclDeviceMemoryPitched<sycl::float2, 2> in_depthMap_dmp(out_normalMap_dmp.getSize(), allocSuccess, queue);

            if (!allocSuccess) ALICEVISION_THROW_ERROR("Not enough device memory to compute normal map!")

            {
                SyclHostMemoryHeap<sycl::float2, 2> in_depthMap_hmh(in_depthMap_dmp.getSize(), queue);

                for (int y = 0; y < height; ++y)
                    for (int x = 0; x < width; ++x)
                        in_depthMap_hmh(size_t(x), size_t(y)).x() = in_depthMap(y, x);

                event = in_depthMap_dmp.copyFrom(in_depthMap_hmh, queue, event);
            }

            // compute normal map synchronosly
            event = sycl_depthMapComputeNormal(out_normalMap_dmp, in_depthMap_dmp, camParams, 1 /*step*/, roi, queue, event);
            event.wait();

            // write output normal map
            writeNormalMapFiltered(rc, _mp, tileParams, roi, out_normalMap_dmp, queue);

            ALICEVISION_LOG_INFO("Compute normal map (rc: " << rc << ") done in: " << timer.elapsedMs() << " ms.");
        }
    }
}

}  // namespace depthMap_sycl
}  // namespace aliceVision
