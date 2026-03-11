// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/mvsData/ROI.hpp>
#include <aliceVision/mvsUtils/MultiViewParams.hpp>
#include <aliceVision/mvsUtils/TileParams.hpp>
#include <aliceVision/depthMap_sycl/Tile.hpp>
#include <aliceVision/depthMapCommon/RefineParams.hpp>
#include <aliceVision/depthMapCommon/SgmParams.hpp>
#include <aliceVision/depthMap_sycl/SgmDepthList.hpp>
#include <aliceVision/depthMap_sycl/sycl/memory.hpp>
#include <aliceVision/depthMap_sycl/sycl/planeSweeping/similarity.hpp>

#include <vector>
#include <string>

namespace aliceVision {
namespace depthMap_sycl {

/**
 * @class Depth map estimation Semi-Global Matching
 * @brief Manages the calculation of the Semi-Global Matching step.
 */
class Sgm
{
  public:
    /**
     * @brief Sgm constructor.
     * @param[in] mp the multi-view parameters
     * @param[in] tileParams tile workflow parameters
     * @param[in] sgmParams the Semi Global Matching parameters
     * @param[in] computeDepthSimMap Enable final depth/sim map computation
     * @param[in] computeNormalMap Enable final normal map computation
     * @param[in,out] allocationSuccess whether we successfully allocate memory
     * @param[in] stream the stream for gpu execution
     */
    Sgm(const mvsUtils::MultiViewParams& mp,
        const mvsUtils::TileParams& tileParams,
        const depthMapCommon::SgmParams& sgmParams,
        bool computeDepthSimMap,
        bool computeNormalMap,
        bool& allocationSuccess,
        sycl::queue& queue);

    // no default constructor
    Sgm() = delete;

    // default destructor
    ~Sgm() = default;

    // final depth/thickness map getter
    inline const SyclDeviceMemoryPitched<sycl::float2, 2>& getDeviceDepthThicknessMap() const { return _depthThicknessMap_dmp; }

    // final depth/similarity map getter (optional: could be empty)
    inline const SyclDeviceMemoryPitched<sycl::float2, 2>& getDeviceDepthSimMap() const { return _depthSimMap_dmp; }

    // final normal map getter (optional: could be empty)
    inline const SyclDeviceMemoryPitched<sycl::float3, 2>& getDeviceNormalMap() const { return _normalMap_dmp; }

    /**
     * @brief Compute for a single R camera the Semi-Global Matching.
     * @param[in] tile The given tile for SGM computation
     * @param[in] tileDepthList the tile SGM depth list
     */
    sycl::event sgmRc(const Tile& tile, const SgmDepthList& tileDepthList, sycl::event prerequisite);

    /**
     * @brief Smooth SGM result thickness map
     * @note Important to be a proper Refine input parameter.
     * @param[in] tile The given tile for SGM computation
     * @param[in] refineParams the Refine parameters
     */
    sycl::event smoothThicknessMap(const Tile& tile, const depthMapCommon::RefineParams& refineParams, sycl::event prerequisite);

  private:
    // private methods

    /**
     * @brief Compute for each RcTc the best / second best similarity volumes.
     * @param[in] tile The given tile for SGM computation
     * @param[in] tileDepthList the tile SGM depth list
     */
    sycl::event computeSimilarityVolumes(const Tile& tile, const SgmDepthList& tileDepthList, sycl::event prerequisite);

    /**
     * @brief Optimize the given similarity volume.
     * @note  Filter on the 3D volume to weight voxels based on their neighborhood strongness.
     *        So it downweights local minimums that are not supported by their neighborhood.
     * @param[in] tile The given tile for SGM computation
     * @param[in] tileDepthList the tile SGM depth list
     */
    sycl::event optimizeSimilarityVolume(const Tile& tile, const SgmDepthList& tileDepthList, sycl::event prerequisite);

    /**
     * @brief Retrieve the best depths in the given similarity volume.
     * @note  For each pixel, choose the voxel with the minimal similarity value.
     * @param[in] tile The given tile for SGM computation
     * @param[in] tileDepthList the tile SGM depth list
     */
    sycl::event retrieveBestDepth(const Tile& tile, const SgmDepthList& tileDepthList, sycl::event prerequisite);

    /**
     * @brief Export volume alembic files and 9 points csv file.
     * @param[in] tile The given tile for SGM computation
     * @param[in] tileDepthList the tile SGM depth list
     * @param[in] in_volume_dmp the input volume
     * @param[in] name the export filename
     */
    void exportVolumeInformation(const Tile& tile,
                                 const SgmDepthList& tileDepthList,
                                 const SyclDeviceMemoryPitched<TSim, 3>& in_volume_dmp,
                                 const std::string& name,
                                 sycl::event prerequisite);

    // private members

    const mvsUtils::MultiViewParams& _mp;     //< Multi-view parameters
    const mvsUtils::TileParams& _tileParams;  //< tile workflow parameters
    const depthMapCommon::SgmParams& _sgmParams;              //< Semi Global Matching parameters
    const bool _computeDepthSimMap;           //< needs to compute a final depth/sim map
    const bool _computeNormalMap;             //< needs to compute a final normal map

    // private members in device memory

    SyclHostMemoryHeap<float, 1> _depths_hmh;                   //< rc depth data host memory
    SyclDeviceMemoryPitched<float, 1> _depths_dmp;              //< rc depth data device memory
    SyclDeviceMemoryPitched<sycl::float2, 2> _depthThicknessMap_dmp;  //< rc result depth thickness map
    SyclDeviceMemoryPitched<sycl::float2, 2> _depthSimMap_dmp;        //< rc result depth/sim map
    SyclDeviceMemoryPitched<sycl::float3, 2> _normalMap_dmp;          //< rc normal map
    SyclDeviceMemoryPitched<TSim, 3> _volumeBestSim_dmp;        //< rc best similarity volume
    SyclDeviceMemoryPitched<TSim, 3> _volumeSecBestSim_dmp;     //< rc second best similarity volume
    SyclDeviceMemoryPitched<TSimAcc, 2> _volumeSliceAccA_dmp;   //< for optimization: volume accumulation slice A
    SyclDeviceMemoryPitched<TSimAcc, 2> _volumeSliceAccB_dmp;   //< for optimization: volume accumulation slice B
    SyclDeviceMemoryPitched<TSimAcc, 1> _volumeAxisAcc_dmp;     //< for optimization: volume accumulation axis
    sycl::queue _queue;                                       //< queue for device execution
};

}  // namespace depthMap_sycl
}  // namespace aliceVision
