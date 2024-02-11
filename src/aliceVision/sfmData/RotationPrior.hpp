// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/sfmData/Landmark.hpp>

namespace aliceVision {
namespace sfmData {

/**
 * @brief RotationPrior is an information prior about the rotation.
 */
struct RotationPrior
{
    RotationPrior() = default;

    RotationPrior(IndexT view_first,
                  IndexT view_second,
                  Eigen::Matrix3d second_R_first)
      : ViewFirst(view_first),
        ViewSecond(view_second),
        _second_R_first(second_R_first)
    {}

    IndexT ViewFirst = UndefinedIndexT;
    IndexT ViewSecond = UndefinedIndexT;
    Eigen::Matrix3d _second_R_first = Eigen::Matrix3d::Identity();

    bool operator==(const RotationPrior& other) const
    {
        return (ViewFirst == other.ViewFirst) && (ViewSecond == other.ViewSecond) && (_second_R_first == other._second_R_first);
    }
};

}  // namespace sfmData
}  // namespace aliceVision
