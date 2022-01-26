// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/mvsData/StaticVector.hpp>
#include <aliceVision/mvsData/Pixel.hpp>
#include <aliceVision/mvsUtils/MultiViewParams.hpp>
#include <aliceVision/mvsUtils/ImagesCache.hpp>
#include <aliceVision/depthMap/DepthSimMap.hpp>
#include <aliceVision/depthMap/cuda/planeSweeping/similarity.hpp>

namespace aliceVision {
namespace depthMap {

struct SgmParams;

template <class Type, unsigned Dim>
class CudaDeviceMemoryPitched;

/**
 * @brief Depth Map Estimation Semi-Global Matching
 */
class Sgm
{
public:
    Sgm(const SgmParams& sgmParams, const mvsUtils::MultiViewParams& mp, mvsUtils::ImagesCache<ImageRGBAf>& ic, int rc);
    ~Sgm() = default;

    bool sgmRc();

    const StaticVector<int>& getTCams() const { return _tCams; }
    const StaticVector<float>& getDepths() const { return _depths; }
    const DepthSimMap& getDepthSimMap() const { return _depthSimMap; }

private:

    void logRcTcDepthInformation() const;
    void checkStartingAndStoppingDepth() const;

    /**
     * @brief Compute for each RcTc the best / second best similarity volumes.
     * @param[out] out_volBestSim_dmp the best similarity volume in device memory
     * @param[out] out_volSecBestSim_dmp the second best similarity volume in device memory
     */
    void computeSimilarityVolumes(CudaDeviceMemoryPitched<TSim, 3>& out_volBestSim_dmp, CudaDeviceMemoryPitched<TSim, 3>& out_volSecBestSim_dmp) const;

    /**
     * @brief Optimize the given similarity volume.
     * @note  Filter on the 3D volume to weight voxels based on their neighborhood strongness.
     *        So it downweights local minimums that are not supported by their neighborhood.
     * @param[out] out_volSimOptimized_dmp the output optimized similarity volume in device memory
     * @param[in] in_volSim_dmp the input similarity volume in device memory
     */
    void optimizeSimilarityVolume(CudaDeviceMemoryPitched<TSim, 3>& out_volSimOptimized_dmp, const CudaDeviceMemoryPitched<TSim, 3>& in_volSim_dmp) const;

    /**
     * @brief Retrieve the best depths in the given similarity volume.
     * @note  For each pixel, choose the voxel with the minimal similarity value.
     * @param[out] out_bestDepthSimMap the output best depth/sim map
     * @param[in] in_volSim_dmp the input similarity volume in device memory
     */
    void retrieveBestDepth(DepthSimMap& out_bestDepthSimMap, const CudaDeviceMemoryPitched<TSim, 3>& in_volSim_dmp) const;

    /**
     * @brief Export volume alembic file and 9 points csv file.
     * @param[in] in_volSim_dmp the given similarity volume in device memory
     * @param[in] name the export filename
     */
    void exportVolumeInformation(const CudaDeviceMemoryPitched<TSim, 3>& in_volSim_dmp, const std::string& name) const;

    void computeDepthsAndResetTCams();

    /**
     * @brief Compute depths of the principal ray of reference camera rc visible by a pixel in a target camera tc
     *        providing meaningful 3d information.
     */
    StaticVector<StaticVector<float>*>* computeAllDepthsAndResetTCams(float midDepth);

    /**
     * @brief Fill depthsTcamsLimits member variable with index range of depths to sweep
     */
    void computeDepthsTcamsLimits(StaticVector<StaticVector<float>*>* alldepths);

    /**
     * @brief Fill the list of "best" depths (_depths) for rc, from all tc cameras depths
     */
    void computeDepths(float minDepth, float maxDepth, float scaleFactor, const StaticVector<StaticVector<float>*>* alldepths);

    void getMinMaxDepths(float& minDepth, float& midDepth, float& maxDepth);

    StaticVector<float>* getDepthsByPixelSize(float minDepth, float midDepth, float maxDepth);
    StaticVector<float>* getDepthsTc(int tc, float midDepth);

    bool selectBestDepthsRange(int nDepthsThr, StaticVector<float>* rcSeedsDistsAsc);
    bool selectBestDepthsRange(int nDepthsThr, StaticVector<StaticVector<float>*>* alldepths);

    const SgmParams& _sgmParams;
    const mvsUtils::MultiViewParams& _mp;
    mvsUtils::ImagesCache<ImageRGBAf>& _ic;
    const int _rc;

    StaticVector<int> _tCams;
    StaticVector<float> _depths;
    StaticVector<Pixel> _depthsTcamsLimits;
    DepthSimMap _depthSimMap;
};

} // namespace depthMap
} // namespace aliceVision
