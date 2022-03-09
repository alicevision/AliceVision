// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/mvsData/StaticVector.hpp>
#include <aliceVision/mvsData/Pixel.hpp>
#include <aliceVision/mvsData/ROI.hpp>
#include <aliceVision/mvsUtils/MultiViewParams.hpp>

namespace aliceVision {
namespace depthMap {

struct SgmParams;

/**
 * @brief Semi-Global Matching Depth List
 */
class SgmDepthList
{
public:

    SgmDepthList(const SgmParams& sgmParams, const mvsUtils::MultiViewParams& mp, int rc, const ROI& roi);
    ~SgmDepthList() = default;

    const StaticVector<int>& getTCams() const { return _tCams; }
    const StaticVector<float>& getDepths() const { return _depths; }
    const StaticVector<Pixel>& getDepthsTcLimits() const { return _depthsTcLimits; }

    /**
     * @brief Compute R camera depth list / depth limits from T cameras
     */
    void computeListRc();

    /**
     * @brief Log depth information 
     */
    void logRcTcDepthInformation() const;

    /**
     * @brief check the starting and stopping depth
     */
    void checkStartingAndStoppingDepth() const;

private:

    // private methods

    /**
     * @brief Compute min/max/mid/nb depth observation for R camera from SfM 
     * @param[out] min The minimum depth observation
     * @param[out] max The maximum depth observation
     * @param[out] mid The middle depth observation
     * @param[out] nbDepths The number of depth observation
     */
    void getMinMaxMidNbDepthFromSfM(float& min, float& max, float& mid, std::size_t& nbDepths) const;

    /**
     * @brief Compute depths of the principal ray of reference camera rc visible by a pixel in a target camera tc
     *        providing meaningful 3d information.
     * 
     * @param[in] midDepth The middle depth observation
     * @return all depths array
     */
    StaticVector<StaticVector<float>*>* computeAllDepthsAndResetTcs(float midDepth);

    /**
     * @brief Fill the list of "best" depths (_depths) for rc, from all tc cameras depths
     * @param[in] minDepth The minimum depth observation
     * @param[in] maxDepth The maximum depth observation
     * @param[in] scaleFactor The scale factor to apply between each depth
     * @param[in] alldepths The all depths array
     */
    void computeDepths(float minDepth, float maxDepth, float scaleFactor, const StaticVector<StaticVector<float>*>* alldepths);

    /**
     * @brief Compute pre-matching min/max/mid depth for R camera
     * @param[out] minDepth The minimum depth 
     * @param[out] midDepth The middle depth 
     * @param[out] maxDepth The maximum depth 
     */
    void getPreMatchingMinMaxDepths(float& minDepth, float& midDepth, float& maxDepth);

    StaticVector<float>* getDepthsByPixelSize(float minDepth, float midDepth, float maxDepth);
    StaticVector<float>* getDepthsTc(int tc, float midDepth);

    bool selectBestDepthsRange(int nDepthsThr, StaticVector<float>* rcSeedsDistsAsc);
    bool selectBestDepthsRange(int nDepthsThr, StaticVector<StaticVector<float>*>* alldepths);

    // private members

    const int _rc;                         // related R camera index
    const ROI& _roi;                       // 2d region of interest of the R image without any downscale apply
    const SgmParams& _sgmParams;           // Semi Global Matching parameters
    const mvsUtils::MultiViewParams& _mp;  // Multi-view parameters
    StaticVector<float> _depths;           // R camera depth list
    StaticVector<Pixel> _depthsTcLimits;   // T camera depth limits
    StaticVector<int> _tCams;              // T camera indexes, computed in the constructor
};

} // namespace depthMap
} // namespace aliceVision
