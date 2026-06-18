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
     * @param iterationCount the number of filter iterations
     * @param scaleFactor integer factor to increase the filter range
     * @return false if an error occurred
    */
    bool process(sfmData::SfMData& sfmData, const bool filterPosition, const bool filterRotation, const int iterationCount, const int scaleFactor);

    /**
     * @brief Interpolate poses for views without poses using temporal filtering.
     * @param sfmData The scene description containing the views and poses.
     * @param ignoreFirstAndLast If true, the first and last views without poses are not interpolated.
     * @return false if an error occurred during interpolation, true otherwise.
     */
    bool interpolateMissingPoses(sfmData::SfMData& sfmData, const bool ignoreFirstAndLast);

private:
    bool getOrderedViewIds(sfmData::SfMData& sfmData, const IndexT imageGroupID, std::vector<IndexT>& viewIdsVec, std::map<IndexT, IndexT>& viewIdIndices);
};

/**
 * @brief Retrieve the ordered list of pose IDs from the given SfMData.
 *
 * This function extracts the pose IDs of views that have associated poses, orders them according to their frameId,
 * and identifies the first and last view IDs with valid poses.
 *
 * @param[in]  sfmData            The scene description containing views and poses.
 * @param[in]  imageGroupID       The imageGroupID of the views to consider.
 * @param[out] poseIdsVec         Vector to be filled with the ordered pose IDs.
 * @param[out] firstViewWithPose  The ID of the first view with a valid pose.
 * @param[out] lastViewWithPose   The ID of the last view with a valid pose.
 * @return true if at least one pose was found and the output parameters were set, false otherwise.
 */
bool getOrderedPoseIds(const sfmData::SfMData& sfmData, const IndexT imageGroupID, std::vector<IndexT>& poseIdsVec, IndexT& firstViewWithPose, IndexT& lastViewWithPose);
} // namespace sfm
} // namespace aliceVision


class tempFilter
{
public:
    bool init();
    bool applyCoreFilter(Eigen::MatrixXd& inputSignal, Eigen::MatrixXd& filteredSignal, bool diffSignal);
    Eigen::MatrixXd apply(Eigen::MatrixXd& inputSignal, bool isAngle);
    Eigen::MatrixXd applyMultiscale(Eigen::MatrixXd& inputSignal, const unsigned int scaleFactor, const int iterationCount, bool isAngle, const Eigen::VectorX<bool>& posesMask=Eigen::VectorX<bool>(0));

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
