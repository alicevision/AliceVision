// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/numeric/numeric.hpp>

namespace aliceVision {
namespace sfmData {

/**
 * @brief SurveyPoint is a surveyed 3D point on an image
 */
struct SurveyPoint
{
    SurveyPoint() = default;

    SurveyPoint(const Vec3 & paramPoint3d, const Vec2 & paramSurvey, const Vec2 & paramResidual)
      : point3d(paramPoint3d), 
        survey(paramSurvey),
        residual(paramResidual)
    {}

    Vec3 point3d;
    Vec2 survey;
    Vec2 residual;

    bool operator==(const SurveyPoint& other) const
    {
        return (point3d == other.point3d) && (survey == other.survey) && (residual == other.residual);
    }

    inline bool operator!=(const SurveyPoint& other) const { return !(*this == other); }
};

}  // namespace sfmData
}  // namespace aliceVision
