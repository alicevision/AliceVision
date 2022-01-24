// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

namespace aliceVision {
namespace depthMap {

/*
 * @note TSim is the similarity type for volume in device memory.
 * @note TSimAcc is the similarity accumulation type for volume in device memory.
 */

#ifdef TSIM_USE_FLOAT
    using TSim = float;
    using TSimAcc = float;
#else
    using TSim = unsigned char;
    using TSimAcc = unsigned int; // TSimAcc is the similarity accumulation type
#endif

} // namespace depthMap
} // namespace aliceVision

