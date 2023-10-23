// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <iomanip>
#include <sstream>
#include <string>

namespace aliceVision {
namespace utils {

inline std::string toStringZeroPadded(std::size_t i, std::size_t zeroPadding)
{
    std::stringstream ss;
    ss << std::setw(zeroPadding) << std::setfill('0') << i;
    return ss.str();
}

}  // namespace utils
}  // namespace aliceVision
