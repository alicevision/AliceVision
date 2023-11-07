// This file is part of the AliceVision project.
// Copyright (c) 2020 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/image/all.hpp>

#include "coordinatesMap.hpp"

namespace aliceVision {

bool distanceToCenter(aliceVision::image::Image<float>& _weights, const CoordinatesMap& map, int width, int height);
bool computeDistanceMap(image::Image<int>& distance, const image::Image<unsigned char>& mask);

}  // namespace aliceVision