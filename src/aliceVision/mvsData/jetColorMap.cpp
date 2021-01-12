// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "jetColorMap.hpp"

namespace aliceVision {

static float jetr[64] = {0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,
                         0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,
                         0,      0,      0.0625, 0.1250, 0.1875, 0.2500, 0.3125, 0.3750, 0.4375, 0.5000, 0.5625,
                         0.6250, 0.6875, 0.7500, 0.8125, 0.8750, 0.9375, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000,
                         1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000,
                         1.0000, 0.9375, 0.8750, 0.8125, 0.7500, 0.6875, 0.6250, 0.5625, 0.5000};

static float jetg[64] = {0,      0,      0,      0,      0,      0,      0,      0,      0.0625, 0.1250, 0.1875,
                         0.2500, 0.3125, 0.3750, 0.4375, 0.5000, 0.5625, 0.6250, 0.6875, 0.7500, 0.8125, 0.8750,
                         0.9375, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000,
                         1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 0.9375, 0.8750, 0.8125, 0.7500,
                         0.6875, 0.6250, 0.5625, 0.5000, 0.4375, 0.3750, 0.3125, 0.2500, 0.1875, 0.1250, 0.0625,
                         0,      0,      0,      0,      0,      0,      0,      0,      0};

static float jetb[64] = {0.5625, 0.6250, 0.6875, 0.7500, 0.8125, 0.8750, 0.9375, 1.0000, 1.0000, 1.0000, 1.0000,
                         1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000,
                         1.0000, 1.0000, 0.9375, 0.8750, 0.8125, 0.7500, 0.6875, 0.6250, 0.5625, 0.5000, 0.4375,
                         0.3750, 0.3125, 0.2500, 0.1875, 0.1250, 0.0625, 0,      0,      0,      0,      0,
                         0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,
                         0,      0,      0,      0,      0,      0,      0,      0,      0};


ColorRGBf getColorFromJetColorMap(float value)
{
    if(value <= 0.0f)
        return ColorRGBf(0, 0, 0);
    if(value >= 1.0f)
        return ColorRGBf(1.0f, 1.0f, 1.0f);
    const float idx_f = value * 63.0f;
    float integral;
    const float fractB = std::modf(idx_f, &integral);
    const float fractA = 1.0f - fractB;
    const int idx = static_cast<int>(integral);
    ColorRGBf c;
    c.r = jetr[idx] * fractA + jetr[idx + 1] * fractB;
    c.g = jetg[idx] * fractA + jetg[idx + 1] * fractB;
    c.b = jetb[idx] * fractA + jetb[idx + 1] * fractB;
    return c;
}

rgb getRGBFromJetColorMap(float value)
{
    const ColorRGBf color = getColorFromJetColorMap(value);
    return {static_cast<unsigned char>(color.r * 255.0f),
            static_cast<unsigned char>(color.g * 255.0f),
            static_cast<unsigned char>(color.b * 255.0f)};
}

} // namespace aliceVision
