// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/mvsData/ROI.hpp>
#include <aliceVision/mvsUtils/MultiViewParams.hpp>
#include <aliceVision/mvsUtils/TileParams.hpp>
#include <aliceVision/depthMap/Tile.hpp>
#include <aliceVision/depthMap/RefineParams.hpp>
#include <aliceVision/depthMap/SgmParams.hpp>
#include <aliceVision/depthMap/SgmDepthList.hpp>
#include <aliceVision/depthMap/cuda/host/memory.hpp>
#include <aliceVision/depthMap/cuda/planeSweeping/similarity.hpp>

#include <vector>
#include <string>

namespace aliceVision {
namespace depthMap {

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
     * @param[in] stream the stream for gpu execution
     */
    Sgm(const mvsUtils::MultiViewParams& mp,
        const mvsUtils::TileParams& tileParams,
        const SgmParams& sgmParams,
        bool computeDepthSimMap,
        bool computeNormalMap,
        cudaStream_t stream);

    // no default constructor
    Sgm() = delete;

    // default destructor
    ~Sgm() = default;

    // final depth/thickness map getter
    inline const CudaDeviceMemoryPitched<float2, 2>& getDeviceDepthThicknessMap() const { return _depthThicknessMap_dmp; }

    // final depth/similarity map getter (optional: could be empty)
    inline const CudaDeviceMemoryPitched<float2, 2>& getDeviceDepthSimMap() const { return _depthSimMap_dmp; }

    // final normal map getter (optional: could be empty)
    inline const CudaDeviceMemoryPitched<float3, 2>& getDeviceNormalMap() const { return _normalMap_dmp; }

    /**
     * @brief Get memory consumpyion in device memory.
     * @return device memory consumpyion (in MB)
     */
    double getDeviceMemoryConsumption() const;

    /**
     * @brief Get unpadded memory consumpyion in device memory.
     * @return unpadded device memory consumpyion (in MB)
     */
    double getDeviceMemoryConsumptionUnpadded() const;

    /**
     * @brief Compute for a single R camera the Semi-Global Matching.
     * @param[in] tile The given tile for SGM computation
     * @param[in] tileDepthList the tile SGM depth list
     */
    void sgmRc(const Tile& tile, const SgmDepthList& tileDepthList);

    /**
     * @brief Smooth SGM result thickness map
     * @note Important to be a proper Refine input parameter.
     * @param[in] tile The given tile for SGM computation
     * @param[in] refineParams the Refine parameters
     */
    void smoothThicknessMap(const Tile& tile, const RefineParams& refineParams);

  private:
    // private methods

    /**
     * @brief Compute for each RcTc the best / second best similarity volumes.
     * @param[in] tile The given tile for SGM computation
     * @param[in] tileDepthList the tile SGM depth list
     */
    void computeSimilarityVolumes(const Tile& tile, const SgmDepthList& tileDepthList);

    /**
     * @brief Optimize the given similarity volume.
     * @note  Filter on the 3D volume to weight voxels based on their neighborhood strongness.
     *        So it downweights local minimums that are not supported by their neighborhood.
     * @param[in] tile The given tile for SGM computation
     * @param[in] tileDepthList the tile SGM depth list
     */
    void optimizeSimilarityVolume(const Tile& tile, const SgmDepthList& tileDepthList);

    /**
     * @brief Retrieve the best depths in the given similarity volume.
     * @note  For each pixel, choose the voxel with the minimal similarity value.
     * @param[in] tile The given tile for SGM computation
     * @param[in] tileDepthList the tile SGM depth list
     */
    void retrieveBestDepth(const Tile& tile, const SgmDepthList& tileDepthList);

    /**
     * @brief Export volume alembic files and 9 points csv file.
     * @param[in] tile The given tile for SGM computation
     * @param[in] tileDepthList the tile SGM depth list
     * @param[in] in_volume_dmp the input volume
     * @param[in] name the export filename
     */
    void exportVolumeInformation(const Tile& tile,
                                 const SgmDepthList& tileDepthList,
                                 const CudaDeviceMemoryPitched<TSim, 3>& in_volume_dmp,
                                 const std::string& name) const;

    // private members

    const mvsUtils::MultiViewParams& _mp;     //< Multi-view parameters
    const mvsUtils::TileParams& _tileParams;  //< tile workflow parameters
    const SgmParams& _sgmParams;              //< Semi Global Matching parameters
    const bool _computeDepthSimMap;           //< needs to compute a final depth/sim map
    const bool _computeNormalMap;             //< needs to compute a final normal map

    // private members in device memory

    CudaHostMemoryHeap<float, 2> _depths_hmh;                   //< rc depth data host memory
    CudaDeviceMemoryPitched<float, 2> _depths_dmp;              //< rc depth data device memory
    CudaDeviceMemoryPitched<float2, 2> _depthThicknessMap_dmp;  //< rc result depth thickness map
    CudaDeviceMemoryPitched<float2, 2> _depthSimMap_dmp;        //< rc result depth/sim map
    CudaDeviceMemoryPitched<float3, 2> _normalMap_dmp;          //< rc normal map
    CudaDeviceMemoryPitched<TSim, 3> _volumeBestSim_dmp;        //< rc best similarity volume
    CudaDeviceMemoryPitched<TSim, 3> _volumeSecBestSim_dmp;     //< rc second best similarity volume
    CudaDeviceMemoryPitched<TSimAcc, 2> _volumeSliceAccA_dmp;   //< for optimization: volume accumulation slice A
    CudaDeviceMemoryPitched<TSimAcc, 2> _volumeSliceAccB_dmp;   //< for optimization: volume accumulation slice B
    CudaDeviceMemoryPitched<TSimAcc, 2> _volumeAxisAcc_dmp;     //< for optimization: volume accumulation axis
    cudaStream_t _stream;                                       //< stream for gpu execution
};

}  // namespace depthMap
}  // namespace aliceVision
