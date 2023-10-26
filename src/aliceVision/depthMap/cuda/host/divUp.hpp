// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

namespace aliceVision {
namespace depthMap {

/**
 * @brief Round a / b to nearest higher integer value.
 * @param[in] a an integer value
 * @param[in] b an integer value
 * @return nearest higher integer value of round a / b.
 */
__host__ inline unsigned int divUp(unsigned int a, unsigned int b) { return (a % b != 0) ? (a / b + 1) : (a / b); }

}  // namespace depthMap
}  // namespace aliceVision
