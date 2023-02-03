// This file is part of the AliceVision project.
// Copyright (c) 2020 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/numeric/numeric.hpp>
#include <aliceVision/numeric/projection.hpp>
#include <aliceVision/camera/cameraCommon.hpp>
#include <aliceVision/camera/IntrinsicsScaleOffsetDisto.hpp>
#include <aliceVision/geometry/Pose3.hpp>

#include "DistortionFisheye1.hpp"

#include <memory>
#include <algorithm>

namespace aliceVision {
namespace camera {

/**
 * @brief EquiDistant is a camera model used for fisheye optics.
 * See https://en.wikipedia.org/wiki/Fisheye_lens
 * 
 */
class EquiDistant : public IntrinsicsScaleOffsetDisto
{
public:
    EquiDistant() = default;

    EquiDistant(unsigned int w, unsigned int h,
                double focalLengthPix,
                double offsetX, double offsetY,
                std::shared_ptr<Distortion> distortion = nullptr) :
        IntrinsicsScaleOffsetDisto(w, h, focalLengthPix, focalLengthPix, offsetX, offsetY, distortion),
        _circleRadius(std::min(w, h) * 0.5), _circleCenter(w / 2.0, h / 2.0)
    {
    }

    EquiDistant(unsigned int w, unsigned int h,
                double focalLengthPix, double offsetX,
                double offsetY, double circleRadiusPix,
                std::shared_ptr<Distortion> distortion = nullptr) :
        IntrinsicsScaleOffsetDisto(w, h, focalLengthPix, focalLengthPix, offsetX, offsetY, distortion),
        _circleRadius(circleRadiusPix), _circleCenter(w / 2.0, h / 2.0)
    {
    }

    ~EquiDistant() override = default;

    EquiDistant* clone() const override
    {
        return new EquiDistant(*this); 
    }

    void assign(const IntrinsicBase& other) override
    {
        *this = dynamic_cast<const EquiDistant&>(other);
    }

    bool isValid() const override
    {
        return _scale(0) > 0 && IntrinsicBase::isValid();
    }

    EINTRINSIC getType() const override
    {
        return EQUIDISTANT_CAMERA;
    }

    Vec2 project(const geometry::Pose3& pose, const Vec4& pt, bool applyDistortion = true) const override;

    Eigen::Matrix<double, 2, 9> getDerivativeProjectWrtRotation(const geometry::Pose3& pose, const Vec4 & pt);

    Eigen::Matrix<double, 2, 16> getDerivativeProjectWrtPose(const geometry::Pose3& pose, const Vec4 & pt) const override;

    Eigen::Matrix<double, 2, 4> getDerivativeProjectWrtPoint(const geometry::Pose3& pose, const Vec4 & pt) const override;

    Eigen::Matrix<double, 2, 3> getDerivativeProjectWrtDisto(const geometry::Pose3& pose, const Vec4 & pt);

    Eigen::Matrix<double, 2, 2> getDerivativeProjectWrtScale(const geometry::Pose3& pose, const Vec4 & pt);

    Eigen::Matrix<double, 2, 2> getDerivativeProjectWrtPrincipalPoint(const geometry::Pose3& pose, const Vec4 & pt);

    Eigen::Matrix<double, 2, Eigen::Dynamic> getDerivativeProjectWrtParams(const geometry::Pose3& pose, const Vec4& pt3D) const override;

    Vec3 toUnitSphere(const Vec2 & pt) const override;

    Eigen::Matrix<double, 3, 2> getDerivativetoUnitSphereWrtPoint(const Vec2 & pt);

    Eigen::Matrix<double, 3, 2> getDerivativetoUnitSphereWrtScale(const Vec2 & pt);

    double imagePlaneToCameraPlaneError(double value) const override;

    // Transform a point from the camera plane to the image plane
    Vec2 cam2ima(const Vec2& p) const override;

    Eigen::Matrix2d getDerivativeCam2ImaWrtPoint() const override;

    // Transform a point from the image plane to the camera plane
    Vec2 ima2cam(const Vec2& p) const override;

    Eigen::Matrix2d getDerivativeIma2CamWrtPoint() const override;

    Eigen::Matrix2d getDerivativeIma2CamWrtPrincipalPoint() const override;

    /**
     * @brief Return true if this ray should be visible in the image
     * @return true if this ray is visible theorically
     */
    bool isVisibleRay(const Vec3 & ray) const override;

    inline double getCircleRadius() const { return _circleRadius; }

    inline void setCircleRadius(double radius) { _circleRadius = radius; }

    inline double getCircleCenterX() const { return _circleCenter(0); }

    inline void setCircleCenterX(double x) { _circleCenter(0) = x; }

    inline double getCircleCenterY() const { return _circleCenter(1); }

    inline void setCircleCenterY(double y) { _circleCenter(1) = y; }

protected:
    double _circleRadius{0.0};
    Vec2 _circleCenter{0.0, 0.0};
};

} // namespace camera
} // namespace aliceVision
