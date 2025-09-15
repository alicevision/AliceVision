// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/types.hpp>
#include <aliceVision/sfmData/SfMData.hpp>

namespace aliceVision {
namespace sfm {

class poseFilter
{
public:
    using uptr = std::unique_ptr<poseFilter>;

public:
    /**
     * @brief Apply a temporal filter to the poses
     * @param sfmData the scene description
     * @param filterPosition specify whether to filter camera positions
     * @param filterRotation specify whether to filter camera orientations
     * @param scaleFactor integer factor to increase the filter range
     * @param iterationCount the number of filter iterations
     * @return false if an error occurred
    */
    bool process(sfmData::SfMData& sfmData, const bool filterPosition, const bool filterRotation, const int scaleFactor, const int iterationCount);

private:
    bool getOrderedViewIds(sfmData::SfMData& sfmData, std::vector<IndexT>& viewIdsVec);
};

} // namespace sfm
} // namespace aliceVision


class tempFilter
{
public:
    bool init();
    bool applyCoreFilter(Eigen::MatrixXd& inputSignal, Eigen::MatrixXd& filteredSignal, bool diffSignal);
    Eigen::MatrixXd apply(Eigen::MatrixXd& inputSignal, bool isAngle);
    Eigen::MatrixXd applyMultiscale(Eigen::MatrixXd& inputSignal, const unsigned int scaleFactor, const int iterationCount, bool isAngle);

private:
    bool initialized;
    const int kernelSize = 9;
    Eigen::VectorXd filterCoeff;
    Eigen::MatrixXd tailFilter;
    Eigen::MatrixXd headFilter;
    Eigen::VectorXd diffFilterCoeff;
    Eigen::MatrixXd tailDiffFilter;
    Eigen::MatrixXd headDiffFilter;
};