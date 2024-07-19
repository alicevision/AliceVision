// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <vector>
#include <Eigen/Dense>

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
 * 
*/
double computeAreaScore(const std::vector<Eigen::Vector2d>& refPts, const std::vector<Eigen::Vector2d>& nextPts, size_t refWidth, size_t refHeight, size_t nextWidth, size_t nextHeight);

}  // namespace sfm
}  // namespace aliceVision
