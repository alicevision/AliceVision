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

class DistortionFisheye : public Distortion
{
  public:
    DistortionFisheye() { _distortionParams = {0.0, 0.0, 0.0, 0.0}; }

    DistortionFisheye(double p1, double p2, double p3, double p4) { _distortionParams = {p1, p2, p3, p4}; }

    EDISTORTION getType() const override { return EDISTORTION::DISTORTION_FISHEYE; }

    DistortionFisheye* clone() const override { return new DistortionFisheye(*this); }

    /// Add distortion to the point p (assume p is in the camera frame [normalized coordinates])
    Vec2 addDistortion(const Vec2& p) const override;

    Eigen::Matrix2d getDerivativeAddDistoWrtPt(const Vec2& p) const override;

    Eigen::MatrixXd getDerivativeAddDistoWrtDisto(const Vec2& p) const override;

    /// Remove distortion (return p' such that disto(p') = p)
    Vec2 removeDistortion(const Vec2& p) const override;

    Eigen::Matrix2d getDerivativeRemoveDistoWrtPt(const Vec2& p) const override;

    Eigen::MatrixXd getDerivativeRemoveDistoWrtDisto(const Vec2& p) const override;

    ~DistortionFisheye() override = default;
};

}  // namespace camera
}  // namespace aliceVision
