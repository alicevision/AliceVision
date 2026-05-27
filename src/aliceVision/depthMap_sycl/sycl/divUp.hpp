// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

namespace aliceVision {
namespace depthMap_sycl {

/**
 * @brief Round a / b to nearest higher integer value.
 * @param[in] a an integer value
 * @param[in] b an integer value
 * @return nearest higher integer value of round a / b.
 */
inline constexpr unsigned int divUp(const unsigned int a, const unsigned int b) { return a / b + (a % b != 0); } // quozient and remainder should be calculated in the same instruction

}  // namespace depthMap_sycl
}  // namespace aliceVision
