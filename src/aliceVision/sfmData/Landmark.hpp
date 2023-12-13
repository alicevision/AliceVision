// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/feature/imageDescriberCommon.hpp>
#include <aliceVision/image/pixelTypes.hpp>
#include <aliceVision/numeric/numeric.hpp>
#include <aliceVision/stl/FlatMap.hpp>
#include <aliceVision/types.hpp>
#include <aliceVision/sfmData/Observation.hpp>

namespace aliceVision {
namespace sfmData {

/**
 * @brief Landmark is a 3D point with its 2d observations.
 */
struct Landmark
{
    Landmark() = default;
    explicit Landmark(feature::EImageDescriberType descType)
      : descType(descType)
    {}
    Landmark(const Vec3& pos3d,
             feature::EImageDescriberType descType = feature::EImageDescriberType::UNINITIALIZED,
             const Observations& observations = Observations(),
             const image::RGBColor& color = image::WHITE)
      : X(pos3d),
        descType(descType),
        observations(observations),
        rgb(color)
    {}

    Vec3 X;
    feature::EImageDescriberType descType = feature::EImageDescriberType::UNINITIALIZED;
    Observations observations;
    image::RGBColor rgb = image::WHITE;  //!> the color associated to the point
    EEstimatorParameterState state = EEstimatorParameterState::REFINED;

    bool operator==(const Landmark& other) const
    {
        return AreVecNearEqual(X, other.X, 1e-3) && AreVecNearEqual(rgb, other.rgb, 1e-3) && observations == other.observations &&
               descType == other.descType;
    }
    inline bool operator!=(const Landmark& other) const { return !(*this == other); }
};

}  // namespace sfmData
}  // namespace aliceVision
