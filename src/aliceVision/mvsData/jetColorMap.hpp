// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/mvsData/Color.hpp>
#include <aliceVision/mvsData/Rgb.hpp>

namespace aliceVision {

/**
 * @brief Get the RGB color from the jet colormap for the given value.
 * @param[in] value from range 0.0 1.0
 * @return RGB value :
 *          - 0.0f > 'value' > 1.0f: color from jet colormap
 *          - 'value' <= 0.0f: black
 *          - 'value' >= 1.0f: white
 */
rgb getRGBFromJetColorMap(float value);

/**
 * @brief Get the float color from the jet colormap for the given value.
 * @param[in] value from range 0.0 1.0
 * @return float color value :
 *          - 0.0f > 'value' > 1.0f: color from jet colormap
 *          - 'value' <= 0.0f: black
 *          - 'value' >= 1.0f: white
 */
ColorRGBf getColorFromJetColorMap(float value);

} // namespace aliceVision
