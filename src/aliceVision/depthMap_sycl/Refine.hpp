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
#include <aliceVision/depthMap_sycl/sycl/memory.hpp>
#include <aliceVision/depthMap_sycl/sycl/planeSweeping/similarity.hpp>

#include <vector>
#include <string>

namespace aliceVision {
namespace depthMap_sycl {

/**
 * @class Depth map estimation Refine
 * @brief Manages the calculation of the Refine step.
 */
class Refine
{
  public:
    /**
     * @brief Refine constructor.
     * @param[in] mp the multi-view parameters
     * @param[in] tileParams tile workflow parameters
     * @param[in] refineParams the Refine parameters
     * @param[in,out] allocationSuccess whether we successfully allocate memory
     * @param[in] queue the queue for device execution
     */
    Refine(const mvsUtils::MultiViewParams& mp, const mvsUtils::TileParams& tileParams, const depthMapCommon::RefineParams& refineParams, bool& allocationSuccess, sycl::queue& queue);

    // no default constructor
    Refine() = delete;

    // default destructor
    ~Refine() = default;

    // final depth/similarity map getter
    inline const SyclDeviceMemoryPitched<sycl::float2, 2>& getDeviceDepthSimMap() const { return _optimizedDepthSimMap_dmp; }

    /**
     * @brief Refine for a single R camera the Semi-Global Matching depth/sim map.
     * @param[in] tile The given tile for Refine computation
     * @param[in] in_sgmDepthThicknessMap_dmp the SGM result depth/thickness map in device memory
     * @param[in] in_sgmNormalMap_dmp the SGM result normal map in device memory (or empty)
     */
    sycl::event refineRc(const Tile& tile,
                         const SyclDeviceMemoryPitched<sycl::float2, 2>& in_sgmDepthThicknessMap_dmp,
                         const SyclDeviceMemoryPitched<sycl::float3, 2>& in_sgmNormalMap_dmp,
                         sycl::event prerequisite);

  private:
    // private methods

    /**
     * @brief Refine and fuse the given depth/sim map using volume strategy.
     * @param[in] tile The given tile for Refine computation
     */
    sycl::event refineAndFuseDepthSimMap(const Tile& tile, sycl::event prerequisite);

    /**
     * @brief Optimize the refined depth/sim maps.
     * @param[in] tile The given tile for Refine computation
     */
    sycl::event optimizeDepthSimMap(const Tile& tile, sycl::event prerequisite);

    /**
     * @brief Compute and write the normal map from the input depth/sim map.
     * @param[in] tile The given tile for Refine computation
     * @param[in] in_depthSimMap_dmp the input depth/sim map in device memory
     * @param[in] name the export filename
     */
    void computeAndWriteNormalMap(const Tile& tile, const SyclDeviceMemoryPitched<sycl::float2, 2>& in_depthSimMap_dmp, sycl::event prerequisite, const std::string& name = "");

    /**
     * @brief Export volume cross alembic file and 9 points csv file.
     * @param[in] tile The given tile for Refine computation
     * @param[in] name the export filename
     */
    void exportVolumeInformation(const Tile& tile, const std::string& name);

    // private members

    const mvsUtils::MultiViewParams& _mp;     //< Multi-view parameters
    const mvsUtils::TileParams& _tileParams;  //< tile workflow parameters
    const depthMapCommon::RefineParams& _refineParams;        //< Refine parameters

    // private members in device memory

    SyclDeviceMemoryPitched<sycl::float2, 2> _sgmDepthPixSizeMap_dmp;    //< rc upscaled SGM depth/pixSize map
    SyclDeviceMemoryPitched<sycl::float2, 2> _refinedDepthSimMap_dmp;    //< rc refined and fused depth/sim map
    SyclDeviceMemoryPitched<sycl::float2, 2> _optimizedDepthSimMap_dmp;  //< rc optimized depth/sim map
    SyclDeviceMemoryPitched<sycl::float3, 2> _sgmNormalMap_dmp;          //< rc upscaled SGM normal map (for experimentation purposes)
    SyclDeviceMemoryPitched<sycl::float3, 2> _normalMap_dmp;             //< rc normal map (for debug / intermediate results purposes)
    SyclDeviceMemoryPitched<TSimRefine, 3> _volumeRefineSim_dmp;   //< rc refine similarity volume
    SyclDeviceMemoryPitched<float, 2> _optImgVariance_dmp;         //< for color optimization: image variance buffer
    sycl::queue _queue;                                            //< queue for device execution
};

}  // namespace depthMap_sycl
}  // namespace aliceVision
