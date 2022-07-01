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
#include <aliceVision/depthMap/Tile.hpp>

namespace aliceVision {
namespace depthMap {

struct SgmParams;

/**
 * @brief Semi-Global Matching Depth List
 */
class SgmDepthList
{
public:

    /**
     * @brief SgmDepthList constructor.
     * @param[in,out] tile The given tile for SGM depth list computation
     */
    SgmDepthList(Tile& tile);

    // default destructor
    ~SgmDepthList() = default;

    // final R camera depth list getter
    inline const StaticVector<float>& getDepths() const { return _depths; }

    // final T camera depth limits getter
    inline const StaticVector<Pixel>& getDepthsTcLimits() const { return _depthsTcLimits; }

    // final R camera first/last depth getter
    inline const std::pair<float, float> getMinMaxDepths() const { return {_depths.front(), _depths.back()}; }

    /**
     * @brief Compute R camera depth list / depth limits from T cameras
     * @param[in] mp the multi-view parameters
     * @param[in] sgmParams the Semi Global Matching parameters
     */
    void computeListRc(const mvsUtils::MultiViewParams& mp, const SgmParams& sgmParams);

    /**
     * @brief Log depth informationinformation
     * @param[in] mp the multi-view parameters
     */
    void logRcTcDepthInformation(const mvsUtils::MultiViewParams& mp) const;

    /**
     * @brief check the starting and stopping depth
     */
    void checkStartingAndStoppingDepth() const;

private:

    // private methods

    /**
     * @brief Compute min/max/mid/nb depth observation for R camera from SfM 
     * @param[in] mp the multi-view parameters
     * @param[in] sgmParams the Semi Global Matching parameters
     * @param[out] out_min The minimum depth observation
     * @param[out] out_max The maximum depth observation
     * @param[out] out_mid The middle depth observation
     * @param[out] out_nbDepths The number of depth observation
     */
    void getMinMaxMidNbDepthFromSfM(const mvsUtils::MultiViewParams& mp,
                                    const SgmParams& sgmParams,
                                    float& out_min,
                                    float& out_max,
                                    float& out_mid,
                                    std::size_t& out_nbDepths) const;

    /**
     * @brief Compute depths of the principal ray of reference camera rc visible by a pixel in a target camera tc
     *        providing meaningful 3d information.
     *
     * @param[in] mp the multi-view parameters
     * @param[in] sgmParams the Semi Global Matching parameters
     * @param[in] midDepth The middle depth observation
     * @return all depths array
     */
    StaticVector<StaticVector<float>*>* computeAllDepthsAndResetTcs(const mvsUtils::MultiViewParams& mp,
                                                                    const SgmParams& sgmParams,
                                                                    float midDepth);

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
     * @param[in] mp the multi-view parameters
     * @param[in] sgmParams the Semi Global Matching parameters
     * @param[out] out_minDepth The minimum depth
     * @param[out] out_midDepth The middle depth
     * @param[out] out_maxDepth The maximum depth
     */
    void getPreMatchingMinMaxDepths(const mvsUtils::MultiViewParams& mp,
                                    const SgmParams& sgmParams,
                                    float& out_minDepth,
                                    float& out_midDepth,
                                    float& out_maxDepth);

    StaticVector<float>* getDepthsByPixelSize(const mvsUtils::MultiViewParams& mp,
                                              const SgmParams& sgmParams,
                                              float minDepth,
                                              float midDepth,
                                              float maxDepth);

    StaticVector<float>* getDepthsTc(const mvsUtils::MultiViewParams& mp,
                                     const SgmParams& sgmParams,
                                     int tc,
                                     float midDepth);

    bool selectBestDepthsRange(int nDepthsThr, StaticVector<float>* rcSeedsDistsAsc);
    bool selectBestDepthsRange(int nDepthsThr, StaticVector<StaticVector<float>*>* alldepths);

    // private members

    Tile& _tile;                           //< related tile
    StaticVector<float> _depths;           //< R camera depth list
    StaticVector<Pixel> _depthsTcLimits;   //< T camera depth limits
};

} // namespace depthMap
} // namespace aliceVision
