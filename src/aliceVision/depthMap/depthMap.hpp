// This file is part of the AliceVision project.
// Copyright (c) 2021 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <vector>

namespace aliceVision {

// MultiViewParams forward declaration
namespace mvsUtils { class MultiViewParams; }

namespace depthMap {

void estimateAndRefineDepthMaps(int cudaDeviceIndex, mvsUtils::MultiViewParams& mp, const std::vector<int>& cams);
void computeNormalMaps(int cudaDeviceIndex, mvsUtils::MultiViewParams& mp, const std::vector<int>& cams);

} // namespace depthMap
} // namespace aliceVision
