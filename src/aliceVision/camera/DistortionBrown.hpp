// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/camera/Distortion.hpp>

namespace aliceVision {
namespace camera {

class DistortionBrown : public Distortion
{
  public:
    DistortionBrown() { _distortionParams = {0.0, 0.0, 0.0, 0.0, 0.0}; }

    DistortionBrown(double p1, double p2, double p3, double p4, double p5) { _distortionParams = {p1, p2, p3, p4, p5}; }

    EDISTORTION getType() const override { return EDISTORTION::DISTORTION_BROWN; }

    DistortionBrown* clone() const override { return new DistortionBrown(*this); }

    /// Add distortion to the point p (assume p is in the camera frame [normalized coordinates])
    Vec2 addDistortion(const Vec2& p) const override;

    /// Remove distortion (return p' such that disto(p') = p)
    Vec2 removeDistortion(const Vec2& p) const override;

    Eigen::Matrix2d getDerivativeAddDistoWrtPt(const Vec2& p) const override;

    Eigen::MatrixXd getDerivativeAddDistoWrtDisto(const Vec2& p) const override;

    Eigen::Matrix2d getDerivativeRemoveDistoWrtPt(const Vec2& p) const override;

    Eigen::MatrixXd getDerivativeRemoveDistoWrtDisto(const Vec2& p) const override;

    ~DistortionBrown() override = default;
};

}  // namespace camera
}  // namespace aliceVision
