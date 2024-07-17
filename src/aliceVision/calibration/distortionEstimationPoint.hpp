// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/calibration/distortionEstimation.hpp>

namespace aliceVision {
namespace calibration {


/**
 * @brief a pair of points coordinates
 * 
 * One vector for the distorted coordinates
 * One vector from the undistorted coordinates
*/
struct PointPair
{
    Vec2 distortedPoint;
    Vec2 undistortedPoint;
    double scale;
};


/**
 * @brief Estimate the undistortion parameters of a camera using a set of pair of undistorted/distorted points.
 *
 * This algorithms minimizes a distance between points and points using distortion.
 *
 * @param[out] undistortionToEstimate Undistortion object with the parameters to estimate.
 * @param[out] statistics Statistics on the estimation error.
 * @param[in] pointpairs Set of pair of points used to estimate distortion.
 * @param[in] lockCenter Lock the distortion offset during optimization.
 * @param[in] lockDistortions Distortion parameters to lock during optimization.
 * @return False if the estimation failed, otherwise true.
 */
bool estimate(std::shared_ptr<camera::Undistortion> undistortionToEstimate,
              Statistics& statistics,
              const std::vector<PointPair>& pointpairs,
              const bool lockCenter,
              const std::vector<bool>& lockDistortions);

}  // namespace calibration
}  // namespace aliceVision
