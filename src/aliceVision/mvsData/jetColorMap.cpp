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


rgb getRGBFromJetColorMap(float value)
{
    if(value <= 0.0f)
        return rgb(0, 0, 0);
    if(value >= 1.0f)
        return rgb(1, 1, 1);
    float idx_f = value * 63.0f;
    float fractA, fractB, integral;
    fractB = std::modf(idx_f, &integral);
    fractA = 1.0f - fractB;
    int idx = static_cast<int>(integral);
    rgb c;
    c.r = static_cast<unsigned char>((jetr[idx] * fractA + jetr[idx + 1] * fractB) * 255.0f);
    c.g = static_cast<unsigned char>((jetg[idx] * fractA + jetg[idx + 1] * fractB) * 255.0f);
    c.b = static_cast<unsigned char>((jetb[idx] * fractA + jetb[idx + 1] * fractB) * 255.0f);
    return c;
}

Color getColorFromJetColorMap(float value)
{
    if(value <= 0.0f)
        return Color(0, 0, 0);
    if(value >= 1.0f)
        return Color(1.0f, 1.0f, 1.0f);
    float idx_f = value * 63.0f;
    float fractA, fractB, integral;
    fractB = std::modf(idx_f, &integral);
    fractA = 1.0f - fractB;
    int idx = static_cast<int>(integral);
    Color c;
    c.r = jetr[idx] * fractA + jetr[idx + 1] * fractB;
    c.g = jetg[idx] * fractA + jetg[idx + 1] * fractB;
    c.b = jetb[idx] * fractA + jetb[idx + 1] * fractB;
    return c;
}

} // namespace aliceVision
