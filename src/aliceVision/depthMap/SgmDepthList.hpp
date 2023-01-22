// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/mvsData/Pixel.hpp>
#include <aliceVision/mvsUtils/MultiViewParams.hpp>
#include <aliceVision/depthMap/SgmParams.hpp>
#include <aliceVision/depthMap/Tile.hpp>

namespace aliceVision {
namespace depthMap {

/**
 * @brief Semi-Global Matching Depth List
 */
class SgmDepthList
{
public:

    /**
     * @brief SgmDepthList constructor.
     * @param[in] mp the multi-view parameters
     * @param[in] sgmParams the Semi Global Matching parameters
     * @param[in] tile The given tile for depth list computation
     */
    SgmDepthList(const mvsUtils::MultiViewParams& mp, const SgmParams& sgmParams, const Tile& tile);

    // default destructor
    ~SgmDepthList() = default;

    // final R camera depth list getter
    inline const std::vector<float>& getDepths() const { return _depths; }

    // final T camera depth limits getter
    inline const std::vector<Pixel>& getDepthsTcLimits() const { return _depthsTcLimits; }

    // final R camera first/last depth getter
    inline const std::pair<float, float> getMinMaxDepths() const { return {_depths.front(), _depths.back()}; }

    /**
     * @brief Compute R camera depth list / depth limits from T cameras
     * @param[in,out] tile The given tile for depth list computation
     */
    void computeListRc();

    /**
     * @brief Remove tile tcs with no depth
     * @note also remove depthsTcLimits with no depth
     */
    void removeTcWithNoDepth(Tile& tile);

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
     * @brief Compute min/max/mid/nb depth observation for R camera from SfM.
     * @param[out] out_min The minimum depth observation
     * @param[out] out_max The maximum depth observation
     * @param[out] out_mid The middle depth observation
     * @param[out] out_nbDepths The number of depth observation
     */
    void getMinMaxMidNbDepthFromSfM(float& out_min,
                                    float& out_max,
                                    float& out_mid,
                                    std::size_t& out_nbDepths) const;

    /**
     * @brief Compute min/max depth from common Rc/Tc SfM observations.
     * @param[in] tc The T camera index
     * @param[out] out_zmin The minimum depth
     * @param[out] out_zmax The maximum depth
     */
    void getRcTcDepthRangeFromSfM(int tc,
                                  double& out_zmin,
                                  double& out_zmax) const;

    /**
     * @brief Compute depths of the principal ray of reference camera rc visible by a pixel in a target camera tc
     *        providing meaningful 3d information.
     * @param[in] tc the T camera index
     * @param[in] midDepth The middle depth observation
     * @param[out] out_depths the output depth list
     */
    void computeRcTcDepths(int tc, 
                           float midObsDepth,
                           std::vector<float>& out_depths) const;

    /**
     * @brief Compute a depth list from R camera pixel size.
     * @param[in] minObsDepth The min depth observation
     * @param[in] midObsDepth The middle depth observation
     * @param[in] maxObsDepth The max depth observation
     * @param[out] out_depths the output depth list
     */
    void computePixelSizeDepths(float minObsDepth,
                                float midObsDepth,
                                float maxObsDepth, 
                                std::vector<float>& out_depths) const;

    /**
     * @brief Fill the list of "best" depths (_depths) for rc, from all tc cameras depths.
     * @param[in] firstDepth The first depth 
     * @param[in] lastDepth The last depth
     * @param[in] scaleFactor The scale factor to apply between each depth
     * @param[in] dephtsPerTc The depth list per T camera
     */
    void computeRcDepthList(float firstDepth, 
                            float lastDepth, 
                            float scaleFactor, 
                            const std::vector<std::vector<float>>& dephtsPerTc);


    /**
     * @brief Export multiple intermediate depth list txt files.
     * @param[in] dephtsPerTc The depth list per T camera
     */
    void exportTxtFiles(const std::vector<std::vector<float>>& dephtsPerTc) const;

    // private members

    const mvsUtils::MultiViewParams& _mp;    //< Multi-view parameters
    const SgmParams& _sgmParams;             //< Semi Global Matching parameters
    const Tile& _tile;                       //< Tile for depth list computation

    std::vector<float> _depths;              //< R camera depth list
    std::vector<Pixel> _depthsTcLimits;      //< T camera depth limits
};

} // namespace depthMap
} // namespace aliceVision
