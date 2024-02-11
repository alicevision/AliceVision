// This file is part of the AliceVision project.
// Copyright (c) 2020 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/image/all.hpp>

#include <aliceVision/panorama/cachedImage.hpp>

namespace aliceVision {

bool feathering(aliceVision::image::Image<image::RGBfColor>& output,
                const aliceVision::image::Image<image::RGBfColor>& color,
                const aliceVision::image::Image<unsigned char>& inputMask);

bool feathering(CachedImage<image::RGBfColor>& input_output, CachedImage<unsigned char>& inputMask);

}  // namespace aliceVision