// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/depthMap/cuda/normalMapping/DeviceNormalMapper.hpp>

namespace aliceVision {
namespace depthMap {

extern void cuda_computeNormalMap(DeviceNormalMapper* mapping,
                                  int width,
                                  int height,
                                  int wsh, 
                                  float gammaC, 
                                  float gammaP);

} // namespace depthMap
} // namespace aliceVision

