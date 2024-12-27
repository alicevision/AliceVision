// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/numeric/numeric.hpp>
#include <aliceVision/numeric/projection.hpp>
#include <aliceVision/camera/cameraCommon.hpp>
#include <aliceVision/camera/IntrinsicScaleOffsetDisto.hpp>
#include <aliceVision/geometry/Pose3.hpp>

#include "DistortionFisheye1.hpp"

#include <memory>
#include <algorithm>

namespace aliceVision {
namespace camera {

/**
 * @brief Equirectangular is a camera model used for panoramas.
 * See https://en.wikipedia.org/wiki/Equirectangular_projection
 *
 */
class Equirectangular : public IntrinsicScaleOffsetDisto
{
  public:
    Equirectangular()
      : Equirectangular(1, 1, 1.0, 1.0, 0.0, 0.0)
    {}

    Equirectangular(unsigned int w,
                unsigned int h,
                double focalLengthPixX,
                double focalLengthPixY,
                double offsetX,
                double offsetY)
      : IntrinsicScaleOffsetDisto(w, h, focalLengthPixX, focalLengthPixY, offsetX, offsetY)
    {}


    ~Equirectangular() override = default;

    Equirectangular* clone() const override { return new Equirectangular(*this); }

    static std::shared_ptr<Equirectangular> cast(std::shared_ptr<IntrinsicBase> sptr);

    void assign(const IntrinsicBase& other) override { *this = dynamic_cast<const Equirectangular&>(other); }

    bool isValid() const override { return _scale(0) > 0 && IntrinsicBase::isValid(); }

    EINTRINSIC getType() const override;

    Vec2 transformProject(const Eigen::Matrix4d& pose, const Vec4& pt, bool applyDistortion = true) const override;

    Vec2 transformProject(const geometry::Pose3& pose, const Vec4& pt3D, bool applyDistortion = true) const
    {
        return transformProject(pose.getHomogeneous(), pt3D, applyDistortion);
    }

    Vec2 project(const Vec4& pt, bool applyDistortion = true) const override;

    Eigen::Matrix<double, 2, 3> getDerivativeTransformProjectWrtPoint3(const Eigen::Matrix4d& pose, const Vec4& pt) const override;

    Eigen::Matrix<double, 2, 3> getDerivativeTransformProjectWrtDisto(const Eigen::Matrix4d& pose, const Vec4& pt) const;

    Eigen::Matrix<double, 2, 2> getDerivativeTransformProjectWrtScale(const Eigen::Matrix4d& pose, const Vec4& pt) const;

    Eigen::Matrix<double, 2, 2> getDerivativeTransformProjectWrtPrincipalPoint(const Eigen::Matrix4d& pose, const Vec4& pt) const;

    Eigen::Matrix<double, 2, Eigen::Dynamic> getDerivativeTransformProjectWrtParams(const Eigen::Matrix4d& pose, const Vec4& pt3D) const override;

    Vec3 toUnitSphere(const Vec2& pt) const override;

    Eigen::Matrix<double, 3, 2> getDerivativetoUnitSphereWrtPoint(const Vec2& pt) const;

    /**
     * @brief Get the derivative of the unit sphere backprojection
     * @param[in] pt2D The 2D point
     * @return The backproject jacobian with respect to the pose
     */
    Eigen::Matrix<double, 3, Eigen::Dynamic> getDerivativeBackProjectUnitWrtParams(const Vec2& pt2D) const override;

    double imagePlaneToCameraPlaneError(double value) const override;

    /**
     * @brief Return true if this ray should be visible in the image
     * @param[in] ray the ray that may or may not be visible in the image
     * @return True if this ray is visible theoretically, false otherwise
     */
    bool isVisibleRay(const Vec3& ray) const override;

    /**
     * @brief Get the horizontal FOV in radians
     * @return Horizontal FOV in radians
     */
    double getHorizontalFov() const override;

    /**
     * @brief Get the vertical FOV in radians
     * @return Vertical FOV in radians
     */
    double getVerticalFov() const override;


    /**
     * @brief how a one pixel change relates to an angular change
     * @return a value in radians
    */
    double pixelProbability() const override;
};

}  // namespace camera
}  // namespace aliceVision
