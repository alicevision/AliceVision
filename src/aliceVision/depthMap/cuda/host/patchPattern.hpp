// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/depthMap/CustomPatchPatternParams.hpp>

namespace aliceVision {
namespace depthMap {

/**
 * @brief Build user custom patch pattern in CUDA constant memory.
 * @param[in] patchParams the user custom patch pattern parameters
 */
void buildCustomPatchPattern(const CustomPatchPatternParams& patchParams);

} // namespace depthMap
} // namespace aliceVision
