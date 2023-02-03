// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "DistortionFisheye1.hpp"

namespace aliceVision {
namespace camera {

Vec2 DistortionFisheye1::addDistortion(const Vec2 & p) const
{
    const double& k1 = _distortionParams.at(0);
    const double r = std::hypot(p(0), p(1));
    const double coef = (std::atan(2.0 * r * std::tan(0.5 * k1)) / k1) / r;
    return  p * coef;
}

Vec2 DistortionFisheye1::removeDistortion(const Vec2& p) const
{
    const double& k1 = _distortionParams.at(0);
    const double r = std::hypot(p(0), p(1));
    const double coef = 0.5 * std::tan(r * k1) / (std::tan(0.5 * k1) * r);
    return  p * coef;
}

} // namespace camera
} // namespace aliceVision
