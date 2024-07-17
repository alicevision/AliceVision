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
 * @brief Statistics on distortion parameters estimation error.
 */
struct Statistics
{
    double mean;
    double stddev;
    double median;
    double lastDecile;
    double max;
};

}  // namespace calibration
}  // namespace aliceVision
