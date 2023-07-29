// This file is part of the AliceVision project.
// Copyright (c) 2021 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/camera/cameraCommon.hpp>
#include <aliceVision/camera/Pinhole.hpp>
#include <aliceVision/camera/Distortion3DE.hpp>
#include <aliceVision/camera/Undistortion3DE.hpp>

#include <memory>

namespace aliceVision {
namespace camera {

/// Implement a Pinhole camera with 3DE radial4 model
class Pinhole3DERadial4 : public Pinhole
{
public:

    explicit Pinhole3DERadial4(int w = 0, int h = 0,
                               double focalLengthPixX = 0.0, double focalLengthPixY = 0.0,
                               double offsetX = 0, double offsetY = 0,
                               double c2 = 0.0, double c4 = 0.0,
                               double u1 = 0.0, double v1 = 0.0,
                               double u2 = 0.0, double v2 = 0.0,
                               EInitMode distortionInitializationMode = EInitMode::NONE) :
    Pinhole(w, h, focalLengthPixX, focalLengthPixY, offsetX, offsetY,
            std::shared_ptr<Distortion>(new Distortion3DERadial4(c2, c4, u1, v1, u2, v2)),
            nullptr,
            distortionInitializationMode)
    {
    }

    Pinhole3DERadial4* clone() const override { return new Pinhole3DERadial4(*this); }
    void assign(const IntrinsicBase& other) override { *this = dynamic_cast<const Pinhole3DERadial4&>(other); }

    EINTRINSIC getType() const override { return EINTRINSIC::PINHOLE_CAMERA_3DERADIAL4; }

    ~Pinhole3DERadial4() override = default;
};

/// Implement a Pinhole camera with anamorphic distortion.
class Pinhole3DEAnamorphic4 : public Pinhole
{
public:

    explicit Pinhole3DEAnamorphic4(int w = 0, int h = 0,
                                   double focalLengthPixX = 0.0, double focalLengthPixY = 0.0,
                                   double offsetX = 0, double offsetY = 0,
                                   EInitMode distortionInitializationMode = EInitMode::NONE) :
    Pinhole(w, h, focalLengthPixX, focalLengthPixY, offsetX, offsetY,
            nullptr,
            std::make_shared<Undistortion3DEAnamorphic4>(w, h),
            distortionInitializationMode)
    {
    }

    Pinhole3DEAnamorphic4 * clone() const override { return new Pinhole3DEAnamorphic4(*this); }
    void assign(const IntrinsicBase& other) override { *this = dynamic_cast<const Pinhole3DEAnamorphic4&>(other); }

    EINTRINSIC getType() const override { return EINTRINSIC::PINHOLE_CAMERA_3DEANAMORPHIC4; }

    ~Pinhole3DEAnamorphic4() override = default;
};

/// Implement a Pinhole camera with a 10 anamorphic distortion coefficients.
class Pinhole3DEClassicLD : public Pinhole
{
public:

    explicit Pinhole3DEClassicLD(int w = 0, int h = 0,
                                 double focalLengthPixX = 0.0, double focalLengthPixY = 0.0,
                                 double offsetX = 0, double offsetY = 0,
                                 double delta = 0.0, double epsilon = 1.0,
                                 double mux = 0.0, double muy = 0.0,
                                 double q = 0.0,
                                 EInitMode distortionInitializationMode = EInitMode::NONE) :
    Pinhole(w, h, focalLengthPixX, focalLengthPixY, offsetX, offsetY,
            std::shared_ptr<Distortion>(new Distortion3DEClassicLD(delta, epsilon, mux, muy, q)),
            nullptr,
            distortionInitializationMode)
    {
    }

    Pinhole3DEClassicLD * clone() const override { return new Pinhole3DEClassicLD(*this); }
    void assign(const IntrinsicBase& other) override { *this = dynamic_cast<const Pinhole3DEClassicLD&>(other); }

    EINTRINSIC getType() const override { return EINTRINSIC::PINHOLE_CAMERA_3DECLASSICLD; }

    ~Pinhole3DEClassicLD() override = default;
};

} // namespace camera
} // namespace aliceVision
