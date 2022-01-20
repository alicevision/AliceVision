// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <time.h>

namespace aliceVision {
namespace depthMap {

// Round a / b to nearest higher integer value.
inline
unsigned int divUp(unsigned int a, unsigned int b)
{
  return (a % b != 0) ? (a / b + 1) : (a / b);
}

inline
clock_t tic()
{
    return clock();
}

// returns the ms passed after last call to tic()
inline
float toc(clock_t ticClk)
{
    return (float)((clock() - ticClk) * 1000.0 / CLOCKS_PER_SEC);
}

} // namespace depthMap
} // namespace aliceVision

