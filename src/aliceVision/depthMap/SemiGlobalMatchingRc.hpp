// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/mvsData/Pixel.hpp>
#include <aliceVision/mvsData/StaticVector.hpp>
#include <aliceVision/depthMap/SemiGlobalMatchingParams.hpp>

namespace aliceVision {
namespace depthMap {

class SemiGlobalMatchingRc
{
public:
    SemiGlobalMatchingRc(int rc, int scale, int step, SemiGlobalMatchingParams& sp);
    ~SemiGlobalMatchingRc();

    bool sgmrc(bool checkIfExists = true);

protected:

    const int _rc;
    const int _scale;
    const int _step;

    int _width;
    int _height;
    int _sgmWsh = 4;
    float _sgmGammaC = 5.5;
    float _sgmGammaP = 8.0;

    std::string _filteringAxes = "YX";

    StaticVector<int> _sgmTCams;
    StaticVector<Pixel> _depthsTcamsLimits;
    DepthSimMap _sgmDepthSimMap;
    StaticVector<float> _depths;

    SemiGlobalMatchingParams& _sp;

private:

    float getMinTcStepAtDepth(float depth, float minDepth, float maxDepth, StaticVector<StaticVector<float>*>* alldepths);
    bool selectBestDepthsRange(int nDepthsThr, StaticVector<float>* rcSeedsDistsAsc);
    bool selectBestDepthsRange(int nDepthsThr, StaticVector<StaticVector<float>*>* alldepths);

    /**
     * @brief Compute depths of the principal ray of reference camera rc visible by a pixel in a target camera tc
     *        providing meaningful 3d information.
     */
    StaticVector<StaticVector<float>*>* computeAllDepthsAndResetTCams(float midDepth);

    /**
     * @brief Fill depthsTcamsLimits member variable with index range of depths to sweep
     */
    void computeDepthsTcamsLimits(StaticVector<StaticVector<float>*>* alldepths);
    void computeDepths(float minDepth, float maxDepth, float scaleFactor, StaticVector<StaticVector<float>*>* alldepths);
    void computeDepthsAndResetTCams();

protected:
    std::string outDir;
    std::string tmpDir;
    std::string tcamsFileName;
    std::string depthsFileName;
    std::string depthsTcamsLimitsFileName;
    std::string SGM_depthMapFileName;
    std::string SGM_simMapFileName;
    std::string SGM_idDepthMapFileName;
};

} // namespace depthMap
} // namespace aliceVision
