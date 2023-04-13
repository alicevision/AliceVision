// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/mvsUtils/MultiViewParams.hpp>
#include <aliceVision/mvsUtils/TileParams.hpp>
#include <aliceVision/depthMap/DepthMapParams.hpp>
#include <aliceVision/depthMap/SgmParams.hpp>
#include <aliceVision/depthMap/RefineParams.hpp>
#include <aliceVision/depthMap/computeOnMultiGPUs.hpp>
#include <aliceVision/depthMap/Tile.hpp>

#include <vector>

namespace aliceVision {
namespace depthMap {

/**
 * @class Depth Map Estimator
 * @brief Wrap depth maps estimation computation.
 * @note Allows muli-GPUs computation (interface IGPUJob)
 */
class DepthMapEstimator : public IGPUJob
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
                      const DepthMapParams& depthMapParams,
                      const SgmParams& sgmParams,
                      const RefineParams& refineParams);

    // no copy constructor
    DepthMapEstimator(DepthMapEstimator const&) = delete;

    // no copy operator
    void operator=(DepthMapEstimator const&) = delete;

    // destructor
    ~DepthMapEstimator() = default;

    /**
     * @brief Compute depth/similarity maps of the given cameras.
     * @param[in] cudaDeviceId the CUDA device id
     * @param[in] cams the list of cameras
     */
    void compute(int cudaDeviceId, const std::vector<int>& cams) override;

private:

    // private methods

    /**
     * @brief Compute the maximum number of tiles (volumes, buffer, images, ...)
     *        that fit in GPU memory and can be computed simultaneously.
     * @return number of tiles
     */
    int getNbSimultaneousTiles() const;

    /**
     * @brief Build tile list from the given cameras.
     * @param[in] cams the list of cameras
     * @param[in,out] tiles the output tiles list
     */
   void getTilesList(const std::vector<int>& cams, std::vector<Tile>& tiles) const;

    // private members

    const mvsUtils::MultiViewParams& _mp;      //< multi-view parameters
    const mvsUtils::TileParams& _tileParams;   //< tiling parameters
    const DepthMapParams& _depthMapParams;     //< depth map estimation parameters
    const SgmParams& _sgmParams;               //< parameters of Sgm process
    const RefineParams& _refineParams;         //< parameters of Refine process
    std::vector<ROI> _tileRoiList;             //< depth maps region-of-interest list
};

} // namespace depthMap
} // namespace aliceVision
