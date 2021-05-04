// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/numeric/numeric.hpp>
#include <aliceVision/camera/cameraCommon.hpp>
#include <aliceVision/camera/DistortionBrown.hpp>

#include <vector>

namespace aliceVision {
namespace camera {

/// Implement a Pinhole camera with a 3 radial distortion coefficients and 2 tangential distortion coefficients.
/// x_d = x_u (1 + K_1 r^2 + K_2 r^4 + K_3 r^6) + (T_2 (r^2 + 2 x_u^2) + 2 T_1 x_u y_u)
/// y_d = y_u (1 + K_1 r^2 + K_2 r^4 + K_3 r^6) + (T_1 (r^2 + 2 y_u^2) + 2 T_2 x_u y_u)
class PinholeBrownT2 : public Pinhole
{
    public:

    explicit PinholeBrownT2(int w = 0, int h = 0, double focalLengthPixX = 0.0, double focalLengthPixY = 0.0, double ppx = 0, double ppy = 0, double k1 = 0.0, double k2 = 0.0, double k3 = 0.0, double t1 = 0.0, double t2 = 0.0)
    : Pinhole(w, h, focalLengthPixX, focalLengthPixY, ppx, ppy, std::shared_ptr<Distortion>(new DistortionBrown(k1, k2, k3, t1, t2)))
    {
    }

    PinholeBrownT2* clone() const override { return new PinholeBrownT2(*this); }
    void assign(const IntrinsicBase& other) override { *this = dynamic_cast<const PinholeBrownT2&>(other); }
  
    EINTRINSIC getType() const override { return EINTRINSIC::PINHOLE_CAMERA_BROWN; }

    ~PinholeBrownT2() override = default;
};

} // namespace camera
} // namespace aliceVision
