// This file is part of the AliceVision project.
// Copyright (c) 2018 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/depthMap/cuda/commonStructures.hpp>

namespace aliceVision {
namespace depthMap {

#define MAX_CONSTANT_GAUSS_SCALES   10
#define MAX_CONSTANT_GAUSS_MEM_SIZE 128

extern void ps_create_gaussian_arr( int deviceId, int scales );

extern void ps_downscale_gauss( Pyramid& ps_texs_arr,
                                int camId, int scale,
                                int w, int h, int radius );

} // namespace depthMap
} // namespace aliceVision

