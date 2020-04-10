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
 * @brief Constraint2D is a 3D point with its 2d observations.
 */
struct Constraint2D
{
  Constraint2D() = default;
  
  Constraint2D(IndexT view_first = UndefinedIndexT, const Observation & observation_first = Observation(),
            IndexT view_second = UndefinedIndexT, const Observation & observation_second = Observation())
    : ViewFirst(view_first)
    , ObservationFirst(observation_first)
    , ViewSecond(view_second)
    , ObservationSecond(observation_second)
  {}

  IndexT ViewFirst;
  IndexT ViewSecond;
  Observation ObservationFirst;
  Observation ObservationSecond;
  
  bool operator==(const Constraint2D& other) const
  {
    return (ViewFirst == other.ViewFirst) && 
          (ViewSecond == other.ViewSecond) && 
          (ObservationFirst == other.ObservationFirst) && 
          (ObservationSecond == other.ObservationSecond);
  }
};

} // namespace sfmData
} // namespace aliceVision
