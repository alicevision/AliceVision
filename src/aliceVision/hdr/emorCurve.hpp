// This file is part of the AliceVision project.
// Copyright (c) 2019 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <algorithm>
#include <cstddef>
#include <cassert>
#include <cmath>
#include <vector>
#include <array>
#include <iostream>

namespace aliceVision {
namespace hdr {

const std::size_t kEmorMDimension = 26;
const std::size_t kEmorQuantization = 1024;

/**
 * @brief hcurve accessor
 * @param[in] index of the emor curve
 * @return a pointer to the first element of the ith array of kEmorInv
 */
const double* getEmorInvCurve(int i);

/**
 * @brief hcurve accessor
 * @param[in] index of the emor curve
 * @return a pointer to the first element of the ith array of kEmorInv
 */
const double* getEmorCurve(int i);


} // namespace hdr
} // namespace aliceVision
