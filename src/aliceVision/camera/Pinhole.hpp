// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/numeric/numeric.hpp>
#include <aliceVision/numeric/projection.hpp>
#include <aliceVision/geometry/Pose3.hpp>
#include <aliceVision/camera/cameraCommon.hpp>
#include <aliceVision/camera/IntrinsicsScaleOffsetDisto.hpp>

#include <memory>
#include <limits>
#include <algorithm>


namespace aliceVision {
namespace camera {

/// Define a classic Pinhole camera
class Pinhole : public IntrinsicsScaleOffsetDisto
{
public:
    Pinhole() :
    Pinhole(1, 1, 1.0, 1.0, 0.0, 0.0)
    {
    }

    Pinhole(unsigned int w, unsigned int h, const Mat3& K) :
    IntrinsicsScaleOffsetDisto(w, h, K(0, 0), K(1, 1), K(0, 2), K(1, 2))
    {
    }

    Pinhole(unsigned int w, unsigned int h,
            double focalLengthPixX, double focalLengthPixY,
            double offsetX, double offsetY,
            std::shared_ptr<Distortion> distortion = nullptr,
            std::shared_ptr<Undistortion> undistortion = nullptr,
            EInitMode distortionInitializationMode = EInitMode::NONE) :
    IntrinsicsScaleOffsetDisto(w, h, focalLengthPixX, focalLengthPixY, offsetX, offsetY, distortion, undistortion, distortionInitializationMode)
    {
    }

    ~Pinhole() override = default;

    Pinhole* clone() const override
    {
        return new Pinhole(*this);
    }

    void assign(const IntrinsicBase& other) override
    {
        *this = dynamic_cast<const Pinhole&>(other);
    }

    double getFocalLengthPixX() const
    {
        return _scale(0);
    }

    double getFocalLengthPixY() const
    {
        return _scale(1);
    }

    bool isValid() const override
    {
        return getFocalLengthPixX() > 0 && getFocalLengthPixY() > 0 && IntrinsicBase::isValid();
    }

    EINTRINSIC getType() const override
    {
        return EINTRINSIC::PINHOLE_CAMERA;
    }

    Mat3 K() const;

    void setK(double focalLengthPixX, double focalLengthPixY, double ppx, double ppy);

    void setK(const Mat3 & K);

    Vec2 project(const geometry::Pose3& pose, const Vec4& pt, bool applyDistortion = true) const override;

    Eigen::Matrix<double, 2, 9> getDerivativeProjectWrtRotation(const geometry::Pose3& pose, const Vec4 & pt);

    Eigen::Matrix<double, 2, 16> getDerivativeProjectWrtPose(const geometry::Pose3& pose, const Vec4& pt) const override;

    Eigen::Matrix<double, 2, 4> getDerivativeProjectWrtPoint(const geometry::Pose3& pose, const Vec4 & pt) const override;

    Eigen::Matrix<double, 2, Eigen::Dynamic> getDerivativeProjectWrtDisto(const geometry::Pose3& pose, const Vec4 & pt) const;

    Eigen::Matrix<double, 2, 2> getDerivativeProjectWrtPrincipalPoint(const geometry::Pose3& pose, const Vec4 & pt) const;

    Eigen::Matrix<double, 2, 2> getDerivativeProjectWrtScale(const geometry::Pose3& pose, const Vec4 & pt) const;

    Eigen::Matrix<double, 2, Eigen::Dynamic> getDerivativeProjectWrtParams(const geometry::Pose3& pose, const Vec4& pt3D) const override;

    Vec3 toUnitSphere(const Vec2 & pt) const override;

    Eigen::Matrix<double, 3, 2> getDerivativetoUnitSphereWrtPoint(const Vec2 & pt) const;
  
    double imagePlaneToCameraPlaneError(double value) const override;

    Mat34 getProjectiveEquivalent(const geometry::Pose3 & pose) const;

    /**
     * @brief Return true if this ray should be visible in the image
     * @return true if this ray is visible theoretically
     */
    bool isVisibleRay(const Vec3 & ray) const override;
};

} // namespace camera
} // namespace aliceVision
