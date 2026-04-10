// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <vector>
#include <Eigen/Dense>
#include <aliceVision/types.hpp>

namespace aliceVision {

namespace sfmData {
class SfMData;
}  // namespace sfmData

namespace sfm {

/**
 * @brief Compute the Root Mean Square Error of the residuals
 * @param[in] sfmData The given input SfMData
 * @return RMSE value
 */
double RMSE(const sfmData::SfMData& sfmData);

/**
 * @brief Compute area based score
 * score is the ratio of the area of the convex hull of the points over the image area
 * @param refPts the reference image points
 * @param nextPts the next image points
 * @param refWidth the refereince image width
 * @param refHeight the reference image height
 * @param nextWidth the next image width
 * @param nextHeight the next image height
 * @return score
 */
double computeAreaScore(const std::vector<Eigen::Vector2d>& refPts, const std::vector<Eigen::Vector2d>& nextPts, size_t refWidth, size_t refHeight, size_t nextWidth, size_t nextHeight);

/**
 * @brief Compute the mean focal length for the views belonging to the specified imageGroup.
 * @param sfmData The sfm data containing the views and the associated camera intrinsics.
 * @param imageGroupID The imageGroupID of the views to consider.
 * @return The mean focal length in physical units (not pixels). Returns 1.0 if no intrinsics are present.
 */
const double meanFocalLength(const sfmData::SfMData& sfmData, const IndexT imageGroupID);

/**
 * @brief Compute the total length of the camera trajectory defined by the given poses.
 * @param[in] poses Vector of pointers to camera pose arrays
 * @return The total trajectory length as the sum of distances between consecutive camera positions.
 */
const double cameraTrajectoryLength(const std::vector<double*> poses);

/**
 * @brief Compute the mean offset vector between the centroid of camera poses and the centroid of landmarks.
 * @param[in]  landmarks  Vector of pointers to 3D landmark positions
 * @param[in]  poses      Vector of pointers to camera pose arrays
 * @param[out] meanL2V    Output 3D vector to store the mean offset: poses centroid minus landmarks centroid.
 */
void meanLandmarks2viewsPose(const std::vector<double*> landmarks, const std::vector<double*> poses, std::vector<double>& meanL2V);

}  // namespace sfm
}  // namespace aliceVision
