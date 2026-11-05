// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/sfmData/SfMData.hpp>

#include <random>

namespace aliceVision {
namespace sfm {

/**
 * @brief Apply a temporal filter to the poses
 * @param sfmData the scene description
 * @param positionNoise noise level to add to camera positions
 * @param rotationNoise noise level to add to camera orientations
 * @param randomSeed the seed of the random number generator
 * @return false if an error occurred
*/
bool addPoseNoise(sfmData::SfMData& sfmData,
                  const float positionNoise,
                  const float rotationNoise,
                  const std::mt19937::result_type randomSeed = std::mt19937::default_seed);

} // namespace sfm
} // namespace aliceVision
