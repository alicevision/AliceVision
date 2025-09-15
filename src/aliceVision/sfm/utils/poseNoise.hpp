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

/**
 * @brief Apply a temporal filter to the poses
 * @param sfmData the scene description
 * @param positionNoise noise level to add to camera positions
 * @param rotationNoise noise level to add to camera orientations
 * @return false if an error occurred
*/
bool addPoseNoise(sfmData::SfMData& sfmData, const float positionNoise, const float rotationNoise);

} // namespace sfm
} // namespace aliceVision
