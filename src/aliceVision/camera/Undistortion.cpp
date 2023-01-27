// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Undistortion.hpp"
#include "Undistortion3DE.hpp"

namespace aliceVision {
namespace camera {

std::shared_ptr<Undistortion> Undistortion::create(const Type& type, int width, int height)
{
    if (type == Type::ANAMORPHIC4)
    {
        return std::shared_ptr<Undistortion>(new Undistortion3DEAnamorphic4(width, height));
    } 
     
    return nullptr;
}

} // namespace camera
} // namespace aliceVision