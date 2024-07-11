// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/camera/Undistortion.hpp>
#include <aliceVision/numeric/numeric.hpp>

#include <vector>
#include <cmath>

namespace aliceVision {
namespace camera {

class Undistortion3DERadial4 : public Undistortion
{
  public:
    /**
     * @brief Default constructor, no distortion.
     */
    Undistortion3DERadial4(int width, int height)
      : Undistortion(width, height)
    {
        _undistortionParams = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    }

    EUNDISTORTION getType() const override { return EUNDISTORTION::UNDISTORTION_3DERADIAL4; }

    Undistortion* clone() const override { return new Undistortion3DERadial4(*this); }

    Vec2 undistortNormalized(const Vec2& p) const override;

    Eigen::Matrix<double, 2, 2> getDerivativeUndistortNormalizedwrtPoint(const Vec2& p) const override;
    Eigen::Matrix<double, 2, Eigen::Dynamic> getDerivativeUndistortNormalizedwrtParameters(const Vec2& p) const override;

    /// add distortion (return p' such that undisto(p') = p)
    Vec2 inverseNormalized(const Vec2& p) const override;

    virtual ~Undistortion3DERadial4() = default;
};

}  // namespace camera
}  // namespace aliceVision
