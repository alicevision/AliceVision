// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

// Shared helpers for the distortion estimation unit tests (point, line and geometry).

#include <aliceVision/camera/camera.hpp>
#include <aliceVision/camera/UndistortionRadial.hpp>
#include <aliceVision/numeric/numeric.hpp>

#include <cmath>
#include <memory>
#include <vector>

namespace aliceVision {
namespace calibration {
namespace test {

/**
 * @brief Ground truth radial (k1, k2, k3) distortion of an anamorphic sensor, written independently of camera::Undistortion.
 *
 * Conventions:
 * - The sensor squeezes the width by the pixel aspect ratio: physical (X, Y) lands at sensor pixel (X / pa, Y).
 * - Distortion is radial in physical space. Physical space is normalized by the half diagonal of the image desqueezed
 *   by reducing its height (H / pa), which is a uniform scale of physical space.
 * - An already desqueezed image has square pixels.
 * - The distortion center is the image center plus an offset.
 *
 * Why only a radial k1/k2/k3 model:
 * The purpose of this ground truth is to check the pixel aspect ratio convention (which axis is squeezed,
 * how the image is normalized, what changes on a desqueezed image) against a reference that does not reuse
 * the code under test. This convention is implemented once in the camera::Undistortion base class
 * and is common to all undistortion models, so a single model is enough to check it.
 * The radial model is the one that can be written independently at almost no cost: a few lines,
 * and its inverse is a one dimensional bisection on the radius.
 * Writing the 3DE models (classic LD, radial 4, anamorphic 4) independently would mean rewriting them entirely,
 * including a two dimensional inverse, which would duplicate the model code and its potential mistakes.
 * Model specific behavior is tested separately, with each model class used as its own ground truth
 * (*_recover_allModels tests), and the model formulas are tested in camera/undistortion_test.cpp.
 */
struct RadialSensor
{
    double width;
    double height;
    double pixelAspectRatio;
    bool isDesqueezed;
    Vec2 offset;
    double k1, k2, k3;

    /// Pixel aspect ratio applied to the image: 1 if the image is already desqueezed.
    double squeeze() const
    {
        return isDesqueezed ? 1.0 : pixelAspectRatio;
    }

    double halfDiagonal() const
    {
        const double h = height / squeeze();
        return 0.5 * std::sqrt(width * width + h * h);
    }

    Vec2 center() const
    {
        return Vec2(width / 2.0 + offset.x(), height / 2.0 + offset.y());
    }

    bool inside(const Vec2& p) const
    {
        return p.x() >= 0.0 && p.y() >= 0.0 && p.x() < width && p.y() < height;
    }

    /// Normalized desqueezed coordinates of a sensor pixel.
    Vec2 normalize(const Vec2& p) const
    {
        const Vec2 c = center();
        const double d = halfDiagonal();
        return Vec2((p.x() - c.x()) / d, (p.y() - c.y()) / (squeeze() * d));
    }

    /// Sensor pixel of normalized desqueezed coordinates.
    Vec2 unnormalize(const Vec2& n) const
    {
        const Vec2 c = center();
        const double d = halfDiagonal();
        return Vec2(n.x() * d + c.x(), n.y() * d * squeeze() + c.y());
    }

    /// Undistorted radius of a distorted radius (normalized coordinates).
    double undistortRadius(double rd) const
    {
        const double r2 = rd * rd;
        return rd * (1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);
    }

    /// Undistorted sensor pixel of a distorted sensor pixel.
    Vec2 undistort(const Vec2& distorted) const
    {
        const Vec2 n = normalize(distorted);
        const double rd = n.norm();
        if (rd == 0.0)
        {
            return distorted;
        }
        return unnormalize(n * (undistortRadius(rd) / rd));
    }

    /// Distorted sensor pixel of an undistorted sensor pixel.
    Vec2 distort(const Vec2& undistorted) const
    {
        const Vec2 n = normalize(undistorted);
        const double ru = n.norm();
        if (ru == 0.0)
        {
            return undistorted;
        }

        // undistortRadius is monotonic on the image domain: bisection on the distorted radius
        double lo = 0.0;
        double hi = 2.0 * ru;
        for (int i = 0; i < 200; ++i)
        {
            const double mid = 0.5 * (lo + hi);
            (undistortRadius(mid) < ru ? lo : hi) = mid;
        }

        return unnormalize(n * (0.5 * (lo + hi) / ru));
    }

    /// Undistortion object to estimate: same sensor, zero parameters and zero center offset.
    std::shared_ptr<camera::Undistortion> makeUndistortion() const
    {
        auto undistortion = std::make_shared<camera::UndistortionRadialK3>(static_cast<int>(width), static_cast<int>(height));
        undistortion->setPixelAspectRatio(pixelAspectRatio);
        undistortion->setDesqueezed(isDesqueezed);
        return undistortion;
    }
};

/// 1600x1200 sensor with a fixed radial distortion.
inline RadialSensor makeSensor(double pixelAspectRatio, bool isDesqueezed, const Vec2& offset = Vec2::Zero())
{
    return RadialSensor{1600.0, 1200.0, pixelAspectRatio, isDesqueezed, offset, -0.12, 0.025, -0.004};
}

/// No radial parameter locked.
inline const std::vector<bool> noLock = {false, false, false};

/// Sensor configuration: pixel aspect ratio and whether the image is already desqueezed.
struct SensorCase
{
    double pixelAspectRatio;
    bool isDesqueezed;
};

/// Sensor configurations used by the *_recover_allModels tests.
inline const std::vector<SensorCase> allSensors = {{1.0, false}, {1.33, false}, {2.0, false}, {1.33, true}, {2.0, true}};

/// Undistortion model of a given type on a 1600x1200 sensor.
inline std::shared_ptr<camera::Undistortion> makeModel(camera::EUNDISTORTION type, const std::vector<double>& params, const SensorCase& sensor)
{
    std::shared_ptr<camera::Undistortion> undistortion = camera::createUndistortion(type, 1600, 1200);
    undistortion->setPixelAspectRatio(sensor.pixelAspectRatio);
    undistortion->setDesqueezed(sensor.isDesqueezed);
    undistortion->setParameters(params);
    return undistortion;
}

}  // namespace test
}  // namespace calibration
}  // namespace aliceVision
