// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/sfmData/Observation.hpp>

namespace aliceVision {
namespace sfmData {

/**
 * @brief Landmark is a 3D point with its 2d observations.
 */
struct PBLandmark
{
  PBLandmark() = default;
  explicit PBLandmark(feature::EImageDescriberType descType): descType(descType) {}
  PBLandmark(const Vec4& parameters,
           feature::EImageDescriberType descType = feature::EImageDescriberType::UNINITIALIZED,
           const Observations& observations = Observations(),
           const image::RGBColor &color = image::WHITE)
    : X(parameters)
    , descType(descType)
    , observations(observations)
    , rgb(color)
  {}

  Vec4 X;
  feature::EImageDescriberType descType = feature::EImageDescriberType::UNINITIALIZED;
  Observations observations;
  image::RGBColor rgb = image::WHITE;    //!> the color associated to the point
  
  IndexT primaryView;
  IndexT secondaryView;
  
  bool operator==(const PBLandmark& other) const
  {
    return AreVecNearEqual(X, other.X, 1e-3) &&
           AreVecNearEqual(rgb, other.rgb, 1e-3) &&
           observations == other.observations &&
           descType == other.descType;
  }

  inline bool operator!=(const PBLandmark& other) const 
  { 
    return !(*this == other); 
  }
};



} // namespace sfmData
} // namespace aliceVision
