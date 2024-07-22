// This file is part of the AliceVision project.
// Copyright (c) 2021 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/calibration/distortionEstimation.hpp>

namespace aliceVision {
namespace calibration {

/**
 * Coordinates plus scale space information
*/
struct PointWithScale
{
    Vec2 center;
    double scale;
};

/**
 * @brief Set of 2D points that belong to the same line.
 *
 * The 2D points correspond to real-world points that are aligned and evenly spaced
 * observed with a camera which applies some distortion on them.
 *
 * Therefore these 2D points may not actually be aligned and evenly spaced,
 * and this difference with the ideal line model is used to estimate distortion.
 */
struct LineWithPoints
{
    double angle;
    double dist;
    std::vector<PointWithScale> points;
};


/**
 * @brief Estimate the undistortion parameters of a camera using a set of line aligned points.
 *
 * This algorithms minimizes a distance between points and lines using distortion.
 *
 * @param[out] undistortionToEstimate Undistortion object with the parameters to estimate.
 * @param[out] statistics Statistics on the estimation error.
 * @param[in] lines Set of line aligned points used to estimate distortion.
 * @param[in] lockCenter Lock the distortion offset during optimization.
 * @param[in] lockDistortions Distortion parameters to lock during optimization.
 * @return False if the estimation failed, otherwise true.
 */
bool estimate(std::shared_ptr<camera::Undistortion> undistortionToEstimate,
              Statistics& statistics,
              std::vector<LineWithPoints>& lines,
              const bool lockCenter,
              const bool lockAngles,
              const std::vector<bool>& lockDistortions);



}  // namespace calibration
}  // namespace aliceVision
