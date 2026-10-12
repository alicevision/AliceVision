// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

// Shared helpers for the camera module unit tests: the list of camera/distortion
// combinations every intrinsic test runs on, sample points inside each camera's
// valid domain, and central finite differences.

#include <aliceVision/camera/camera.hpp>
#include <aliceVision/camera/DistortionBrown.hpp>
#include <aliceVision/camera/DistortionFisheye.hpp>
#include <aliceVision/camera/DistortionFisheye1.hpp>
#include <aliceVision/camera/DistortionRadial.hpp>
#include <aliceVision/camera/Equidistant.hpp>
#include <aliceVision/camera/Equirectangular.hpp>
#include <aliceVision/camera/IntrinsicScaleOffsetDisto.hpp>
#include <aliceVision/camera/Pinhole.hpp>
#include <aliceVision/camera/Undistortion.hpp>
#include <aliceVision/numeric/numeric.hpp>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace aliceVision {
namespace camera {
namespace test {

struct TestCamera
{
    std::string name;
    EINTRINSIC intrinsicType;
    EDISTORTION distortionType;
    EUNDISTORTION undistortionType;
    std::shared_ptr<IntrinsicScaleOffsetDisto> intrinsic;
};

inline std::vector<double> distortionTestParameters(EDISTORTION type)
{
    switch (type)
    {
        case EDISTORTION::DISTORTION_RADIALK1:
            return {0.1};
        case EDISTORTION::DISTORTION_RADIALK3:
            return {-0.18, 0.19, -0.025};
        case EDISTORTION::DISTORTION_RADIALK3PT:
            return {-0.18, 0.19, -0.025};
        case EDISTORTION::DISTORTION_BROWN:
            return {-0.054, 0.014, 0.006, 0.001, -0.001};
        case EDISTORTION::DISTORTION_FISHEYE:
            return {-0.054, 0.014, 0.006, 0.011};
        case EDISTORTION::DISTORTION_FISHEYE1:
            return {0.9};
        case EDISTORTION::DISTORTION_NONE:
            break;
    }
    return {};
}

inline std::vector<double> undistortionTestParameters(EUNDISTORTION type)
{
    switch (type)
    {
        case EUNDISTORTION::UNDISTORTION_RADIALK3:
            return {0.1, -0.05, 0.02};
        case EUNDISTORTION::UNDISTORTION_3DECLASSICLD:
            return {0.1, 1.1, 0.02, -0.03, 0.04};
        case EUNDISTORTION::UNDISTORTION_3DEANAMORPHIC4:
            return {0.1, 0.02, -0.03, 0.04, 0.01, -0.02, 0.03, 0.005, -0.006, 0.007, 0.2, 1.1, 0.9};
        case EUNDISTORTION::UNDISTORTION_3DERADIAL4:
            return {0.1, 0.01, -0.02, 0.03, 0.004, -0.005, 0.3, 0.05};
        case EUNDISTORTION::UNDISTORTION_NONE:
            break;
    }
    return {};
}

inline const std::vector<EDISTORTION>& allDistortionTypes()
{
    static const std::vector<EDISTORTION> types = {EDISTORTION::DISTORTION_NONE,
                                                   EDISTORTION::DISTORTION_RADIALK1,
                                                   EDISTORTION::DISTORTION_RADIALK3,
                                                   EDISTORTION::DISTORTION_RADIALK3PT,
                                                   EDISTORTION::DISTORTION_BROWN,
                                                   EDISTORTION::DISTORTION_FISHEYE,
                                                   EDISTORTION::DISTORTION_FISHEYE1};
    return types;
}

inline const std::vector<EUNDISTORTION>& allUndistortionTypes()
{
    static const std::vector<EUNDISTORTION> types = {EUNDISTORTION::UNDISTORTION_NONE,
                                                     EUNDISTORTION::UNDISTORTION_RADIALK3,
                                                     EUNDISTORTION::UNDISTORTION_3DEANAMORPHIC4,
                                                     EUNDISTORTION::UNDISTORTION_3DECLASSICLD,
                                                     EUNDISTORTION::UNDISTORTION_3DERADIAL4};
    return types;
}

inline const std::vector<EINTRINSIC>& allIntrinsicTypes()
{
    static const std::vector<EINTRINSIC> types = {EINTRINSIC::PINHOLE_CAMERA, EINTRINSIC::EQUIDISTANT_CAMERA, EINTRINSIC::EQUIRECTANGULAR_CAMERA};
    return types;
}

/**
 * @brief Create one camera of the given type/distortion/undistortion with non-trivial
 *        parameters, so that the distortion actually moves points.
 */
inline TestCamera makeTestCamera(EINTRINSIC intrinsicType, EDISTORTION distortionType, EUNDISTORTION undistortionType)
{
    TestCamera tc;
    tc.intrinsicType = intrinsicType;
    tc.distortionType = distortionType;
    tc.undistortionType = undistortionType;
    tc.name =
      EINTRINSIC_enumToString(intrinsicType) + "/" + EDISTORTION_enumToString(distortionType) + "/" + EUNDISTORTION_enumToString(undistortionType);

    std::shared_ptr<IntrinsicBase> intrinsic;
    if (isPinhole(intrinsicType))
    {
        intrinsic = createIntrinsic(intrinsicType, distortionType, undistortionType, 1920, 1080, 1500.0, 1520.0, 12.0, -8.0);
    }
    else if (isEquidistant(intrinsicType))
    {
        // With the default 36x24mm sensor, a 450px focal gives a field of view of about 163 degrees.
        intrinsic = createIntrinsic(intrinsicType, distortionType, undistortionType, 1920, 1080, 450.0, 450.0, 6.0, -4.0);
    }
    else
    {
        // Full sphere: 2*pi radians horizontally, pi vertically.
        intrinsic = createIntrinsic(intrinsicType, distortionType, undistortionType, 2000, 1000, 2000.0 / (2.0 * M_PI), 1000.0 / M_PI, 0.0, 0.0);
    }

    tc.intrinsic = IntrinsicScaleOffsetDisto::cast(intrinsic);

    if (distortionType != EDISTORTION::DISTORTION_NONE && tc.intrinsic->getDistortion())
    {
        tc.intrinsic->setDistortionParams(distortionTestParameters(distortionType));
    }
    if (undistortionType != EUNDISTORTION::UNDISTORTION_NONE && tc.intrinsic->getUndistortion())
    {
        tc.intrinsic->getUndistortion()->setParameters(undistortionTestParameters(undistortionType));
    }

    return tc;
}

/**
 * @brief Every supported camera/distortion combination:
 *        pinhole x each distortion, pinhole x each undistortion, pinhole x each (distortion, undistortion) pair,
 *        equidistant x each distortion, equirectangular.
 */
inline std::vector<TestCamera> makeTestCameras()
{
    std::vector<TestCamera> cameras;

    for (EDISTORTION distortion : allDistortionTypes())
    {
        cameras.push_back(makeTestCamera(EINTRINSIC::PINHOLE_CAMERA, distortion, EUNDISTORTION::UNDISTORTION_NONE));
    }
    for (EUNDISTORTION undistortion : allUndistortionTypes())
    {
        if (undistortion != EUNDISTORTION::UNDISTORTION_NONE)
        {
            cameras.push_back(makeTestCamera(EINTRINSIC::PINHOLE_CAMERA, EDISTORTION::DISTORTION_NONE, undistortion));
        }
    }
    // Pinhole with both: the undistortion corrects the raw image, the distortion the remaining one.
    for (EDISTORTION distortion : allDistortionTypes())
    {
        for (EUNDISTORTION undistortion : allUndistortionTypes())
        {
            if (distortion != EDISTORTION::DISTORTION_NONE && undistortion != EUNDISTORTION::UNDISTORTION_NONE)
            {
                cameras.push_back(makeTestCamera(EINTRINSIC::PINHOLE_CAMERA, distortion, undistortion));
            }
        }
    }
    for (EDISTORTION distortion : allDistortionTypes())
    {
        cameras.push_back(makeTestCamera(EINTRINSIC::EQUIDISTANT_CAMERA, distortion, EUNDISTORTION::UNDISTORTION_NONE));
    }
    cameras.push_back(makeTestCamera(EINTRINSIC::EQUIRECTANGULAR_CAMERA, EDISTORTION::DISTORTION_NONE, EUNDISTORTION::UNDISTORTION_NONE));

    return cameras;
}

/**
 * @brief Deterministic grid of pixels inside the useful domain of the camera:
 *        the central 80% of the image for pinhole and equirectangular cameras,
 *        a disk of 80% of the image circle for equidistant cameras.
 *        The exact image center is avoided as some models are singular there.
 */
inline std::vector<Vec2> samplePixels(const IntrinsicBase& intrinsic, int gridSize = 7)
{
    std::vector<Vec2> pixels;
    const Vec2 center(intrinsic.w() * 0.5, intrinsic.h() * 0.5);
    const Vec2 jitter(1.37, -0.83);

    for (int j = 0; j < gridSize; ++j)
    {
        for (int i = 0; i < gridSize; ++i)
        {
            // [-1, 1] x [-1, 1]
            const Vec2 u(2.0 * i / (gridSize - 1) - 1.0, 2.0 * j / (gridSize - 1) - 1.0);

            if (isEquidistant(intrinsic.getType()))
            {
                if (u.norm() > 1.0)
                {
                    continue;
                }
                const double radius = 0.8 * 0.5 * std::min(intrinsic.w(), intrinsic.h());
                pixels.push_back(center + radius * u + jitter);
            }
            else
            {
                pixels.push_back(center + 0.4 * Vec2(u(0) * intrinsic.w(), u(1) * intrinsic.h()) + jitter);
            }
        }
    }

    return pixels;
}

/**
 * @brief Central finite difference of f: R^n -> R^m around x.
 */
template<typename F>
Eigen::MatrixXd numericalJacobian(F f, const Eigen::VectorXd& x, double h = 1e-6)
{
    const Eigen::VectorXd f0 = f(x);
    Eigen::MatrixXd J(f0.size(), x.size());

    for (int i = 0; i < x.size(); ++i)
    {
        Eigen::VectorXd plus = x;
        Eigen::VectorXd minus = x;
        plus(i) += h;
        minus(i) -= h;

        J.col(i) = (f(plus) - f(minus)) / (2.0 * h);
    }

    return J;
}

inline Eigen::VectorXd toVector(const std::vector<double>& v) { return Eigen::Map<const Eigen::VectorXd>(v.data(), v.size()); }

inline std::vector<double> toStdVector(const Eigen::VectorXd& v) { return std::vector<double>(v.data(), v.data() + v.size()); }

/// Largest absolute difference between two matrices of the same size.
inline double maxAbsDiff(const Eigen::MatrixXd& a, const Eigen::MatrixXd& b)
{
    if (a.rows() != b.rows() || a.cols() != b.cols())
    {
        return std::numeric_limits<double>::infinity();
    }
    if (a.size() == 0)
    {
        return 0.0;
    }
    return (a - b).cwiseAbs().maxCoeff();
}

}  // namespace test
}  // namespace camera
}  // namespace aliceVision
