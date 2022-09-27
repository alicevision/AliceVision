// This file is part of the extention to AliceVision project.
// Copyright (c) 2018 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/mvsData/StaticVector.hpp>
#include <aliceVision/mvsUtils/MultiViewParams.hpp>

namespace aliceVision {
namespace depthMap {

typedef void (*GPUJob)(int cudaDeviceNo, mvsUtils::MultiViewParams& mp, const std::vector<int>& cams);

void computeOnMultiGPUs(mvsUtils::MultiViewParams& mp, const std::vector<int>& cams, GPUJob gpujob, int nbGPUsToUse);

} // namespace depthMap
} // namespace aliceVision
