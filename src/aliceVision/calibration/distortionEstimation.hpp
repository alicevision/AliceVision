// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/numeric/numeric.hpp>
#include <aliceVision/camera/Pinhole.hpp>

#include <vector>

namespace aliceVision{
namespace calibration{

struct LineWithPoints
{
    double angle;
    double dist;
    std::vector<Vec2> points;
};

struct PointPair
{
    Vec2 distortedPoint;
    Vec2 undistortedPoint;
};

struct Statistics
{
    double mean;
    double stddev;
    double median;
};

bool estimate(std::shared_ptr<camera::Pinhole> & cameraToEstimate, Statistics & statistics, std::vector<LineWithPoints> & lines, bool lockScale, bool lockCenter, const std::vector<bool> & lockDistortions);

bool estimate(std::shared_ptr<camera::Pinhole> & cameraToEstimate, Statistics & statistics, std::vector<PointPair> & points, bool lockScale, bool lockCenter, const std::vector<bool> & lockDistortions);




}//namespace calibration
}//namespace aliceVision
