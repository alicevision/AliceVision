// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

namespace aliceVision {
namespace sfm {

class SfMData;

/**
 * @brief Compute the Root Mean Square Error of the residuals
 * @param[in] sfmData The given input SfMData
 * @return RMSE value
 */
double RMSE(const SfMData& sfmData);


} // namespace sfm
} // namespace aliceVision
