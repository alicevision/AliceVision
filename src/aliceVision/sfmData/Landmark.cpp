// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.


#include <aliceVision/sfmData/Landmark.hpp>
#include <aliceVision/numeric/NumericFunctions.hpp>

namespace aliceVision {
namespace sfmData {

bool Landmark::operator==(const Landmark& other) const
{
    return AreVecNearEqual(X, other.X, 1e-3) && AreVecNearEqual(rgb, other.rgb, 1e-3) && _observations == other._observations &&
            descType == other.descType;
}

}  // namespace sfmData
}  // namespace aliceVision
