// This file is part of the AliceVision project.
// Copyright (c) 2020 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "boundingBox.hpp"

namespace aliceVision {

std::ostream& operator<<(std::ostream& os, const BoundingBox& in)
{
    os << in.left << ",";
    os << in.top << " ";
    os << in.width << "x";
    os << in.height;

    return os;
}

}  // namespace aliceVision