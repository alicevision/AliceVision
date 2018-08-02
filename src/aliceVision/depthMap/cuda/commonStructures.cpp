// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "commonStructures.hpp"

#include <aliceVision/depthMap/cuda/planeSweeping/plane_sweeping_cuda.hpp>

namespace aliceVision {
namespace depthMap {

cameraStruct::~cameraStruct()
{
    delete cam;
}

} // namespace depthMap
} // namespace aliceVision

