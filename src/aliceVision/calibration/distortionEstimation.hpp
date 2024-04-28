// This file is part of the AliceVision project.
// Copyright (c) 2021 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/numeric/numeric.hpp>
#include <aliceVision/camera/Pinhole.hpp>
#include <aliceVision/camera/Undistortion.hpp>

#include <vector>
#include <memory>

namespace aliceVision {
namespace calibration {

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
    std::vector<Vec2> points;
};

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
};

/**
 * @brief Statistics on distortion parameters estimation error.
 */
struct Statistics
{
    double mean;
    double stddev;
    double median;
    double max;
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
              bool lockCenter,
              const std::vector<bool>& lockDistortions);


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
              bool lockCenter,
              const std::vector<bool>& lockDistortions);

}  // namespace calibration
}  // namespace aliceVision
