// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Observation.hpp"

namespace aliceVision {
namespace sfmData {

bool Observation::operator==(const Observation& other) const
{
    return AreVecNearEqual(_coordinates, other._coordinates, 1e-6) && _idFeature == other._idFeature;
}

}  // namespace sfmData
}  // namespace aliceVision
