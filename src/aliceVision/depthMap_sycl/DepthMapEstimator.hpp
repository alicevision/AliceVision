// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/mvsUtils/MultiViewParams.hpp>
#include <aliceVision/mvsUtils/TileParams.hpp>
#include <aliceVision/mvsUtils/fileIO.hpp>
#include <aliceVision/depthMapCommon/DepthMapParams.hpp>
#include <aliceVision/depthMapCommon/SgmParams.hpp>
#include <aliceVision/depthMapCommon/RefineParams.hpp>
#include <aliceVision/depthMap_sycl/computeOnMultiDevices.hpp>
#include <aliceVision/depthMap_sycl/Tile.hpp>
#include <aliceVision/depthMap_sycl/Sgm.hpp>
#include <aliceVision/depthMap_sycl/Refine.hpp>
#include <aliceVision/depthMap_sycl/sycl/DeviceCache.hpp>

#include <vector>

namespace aliceVision {
namespace depthMap_sycl {

/**
 * @class Depth Map Estimator
 * @brief Wrap depth maps estimation computation.
 * @note Allows muli-GPUs computation (interface IGPUJob)
 */
class DepthMapEstimator : public IDeviceJob
{
  public:
    /**
     * @brief Depth Map Estimator constructor.
     * @param[in] mp the multi-view parameters
     * @param[in] tileParams tile workflow parameters
     * @param[in] depthMapParams the depth map estimation parameters
     * @param[in] sgmParams the Semi Global Matching parameters
     * @param[in] refineParams the Refine parameters
     */
    DepthMapEstimator(const mvsUtils::MultiViewParams& mp,
                      const mvsUtils::TileParams& tileParams,
                      const depthMapCommon::DepthMapParams& depthMapParams,
                      const depthMapCommon::SgmParams& sgmParams,
                      const depthMapCommon::RefineParams& refineParams);

    // no copy constructor
    DepthMapEstimator(DepthMapEstimator const&) = delete;

    // no copy operator
    void operator=(DepthMapEstimator const&) = delete;

    // destructor
    ~DepthMapEstimator() = default;

    /**
     * @brief Compute depth/similarity maps of the given cameras.
     * @param[in] queue the SYCL queue
     * @param[in] cams the list of cameras
     */
    void compute(const sycl::device& device, const std::vector<int>& cams) override;

  private:
    // private methods

    /**
     * @brief Build tile list from the given camera
     * @param[in] cam the camera id
     * @param[in,out] tiles the output tilelist. Must already be correctly sized
     */
    void getTiles(int cam, std::vector<Tile>& tiles);

    // private class to reppresent all the objects we need to compute a tile
    struct ComputeObject
    {
        sycl::queue queue;
        Sgm sgm;
        std::optional<Refine> refineOpt;

        std::mutex lock;
        std::atomic<uint> users; // number of threads waiting to use this compute object, for when we have insufficient memory

        inline void release() { lock.unlock(); users--; };

        template<class... SgmArgs, class... RefineOptArgs>
        inline ComputeObject(sycl::queue& queue, std::tuple<SgmArgs...> sgmArgs, std::tuple<RefineOptArgs...> refineArgs) :
            queue(queue),
            sgm(std::make_from_tuple<Sgm>(sgmArgs)),
            refineOpt(std::make_from_tuple<std::optional<Refine>>(refineArgs)) {};

        ~ComputeObject() = default;
    };

    // private class to manage compute objects
    class ComputeObjectBuffer
    {
      public:
        // construct refine object
        ComputeObject& getComputeObject(const mvsUtils::MultiViewParams& mp,
                                        const mvsUtils::TileParams& tileParams,
                                        const depthMapCommon::SgmParams& sgmParams,
                                        const depthMapCommon::RefineParams& refineParams,
                                        const bool constructRefine,
                                        const bool computeDepthSimMap,
                                        const bool computeNormalMap,
                                        const sycl::device& device);

      private:
        std::list<ComputeObject> objects;
        std::shared_mutex containerLock; // lock to prevent concurrent modification of the container
    };

    // private members

    const mvsUtils::MultiViewParams& _mp;                       //< multi-view parameters
    const mvsUtils::TileParams& _tileParams;                    //< tiling parameters
    const depthMapCommon::DepthMapParams& _depthMapParams;      //< depth map estimation parameters
    const depthMapCommon::SgmParams& _sgmParams;                //< parameters of Sgm process
    const depthMapCommon::RefineParams& _refineParams;          //< parameters of Refine process
    PerDevice<ComputeObjectBuffer> _objectBuffers;              //< objects used in the various stages of computing a tile
    std::vector<ROI> _tileRoiList;                              //< depth maps region-of-interest list
    mvsUtils::ImagesCache<image::Image<image::RGBAfColor>> _ic; //< host cache for loading images
    DeviceCache _deviceCache;                                   //< device cache for storing images
};

}  // namespace depthMap_sycl
}  // namespace aliceVision
