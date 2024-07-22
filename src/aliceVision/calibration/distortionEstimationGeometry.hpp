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
 * @brief struct to store per view variables for distortion estimation
 * many parameters are used per views such as the camera pose
*/
struct DistortionEstimationView
{
    SO3::Matrix R;
    Eigen::Vector3d t;
    double scale;
    double offsetx;
    double offsety;

    Vec2 distortionOffset;
    std::vector<double> parameters;
    std::shared_ptr<camera::Undistortion> undistortion;
    std::vector<PointPair> pointpairs;
};

/**
 * @brief Class to estimate undistortion
 * Assumes the parameters have been initialized previously using another method
 * Calibration is done in 3D to be able to estimate some parameters  for complex lenses
 * This class support multiple views which is useful to estimate e.g. the lens breathing
*/
class DistortionEstimationGeometry
{
public:
    /**
     * @brief Constructor
     * @param sharedParams list of boolean values stating if this particular 
     * distortion value should be shared among the views or not. This vector must be the size of the 
     * distortion vector for the particular model used.
    */
    DistortionEstimationGeometry(const std::vector<bool> & sharedParams) 
    : _sharedParams(sharedParams)
    {
    }

    /**
     * @brief Add a view to the set of estimated views
     * @param undistortion the undistortion object associated to this view
     * @param pointpairs the point pairs measured on this view
     * Assume the undistorted Point coordinates have been normalized on [-0.5; 5]
    */
    void addView(std::shared_ptr<camera::Undistortion> undistortion, const std::vector<PointPair>& pointpairs);

    /**
     * @brief perform computation
     * @param statistics the statistics for the estimation results
     * @param lockCenter do we need to estimate distortion center ?
     * @param lockDistortions per distortion parameter lock (do we need to estimate it ?)
     * Should be the same size than the number of distortion parameters
     * @return false on error
    */
    bool compute(calibration::Statistics & statistics, const bool lockCenter, const std::vector<bool>& lockDistortions);

private:
    const std::vector<bool> _sharedParams;
    std::vector<DistortionEstimationView> _views;
};

}  // namespace calibration
}  // namespace aliceVision
