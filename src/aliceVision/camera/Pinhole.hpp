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
#include <aliceVision/camera/IntrinsicScaleOffsetDisto.hpp>

#include <memory>
#include <limits>
#include <algorithm>

namespace aliceVision {
namespace camera {

/// Define a classic Pinhole camera
class Pinhole : public IntrinsicScaleOffsetDisto
{
  public:
    Pinhole()
      : Pinhole(1, 1, 1.0, 1.0, 0.0, 0.0)
    {}

    Pinhole(unsigned int w, unsigned int h, const Mat3& K)
      : IntrinsicScaleOffsetDisto(w, h, K(0, 0), K(1, 1), K(0, 2), K(1, 2))
    {}

    Pinhole(unsigned int w,
            unsigned int h,
            double focalLengthPixX,
            double focalLengthPixY,
            double offsetX,
            double offsetY,
            std::shared_ptr<Distortion> distortion = nullptr,
            std::shared_ptr<Undistortion> undistortion = nullptr)
      : IntrinsicScaleOffsetDisto(w, h, focalLengthPixX, focalLengthPixY, offsetX, offsetY, distortion, undistortion)
    {}

    ~Pinhole() override = default;

    Pinhole* clone() const override { return new Pinhole(*this); }

    void assign(const IntrinsicBase& other) override { *this = dynamic_cast<const Pinhole&>(other); }

    static std::shared_ptr<Pinhole> cast(std::shared_ptr<IntrinsicBase> sptr);

    double getFocalLengthPixX() const { return _scale(0); }

    double getFocalLengthPixY() const { return _scale(1); }

    bool isValid() const override { return getFocalLengthPixX() > 0 && getFocalLengthPixY() > 0 && IntrinsicBase::isValid(); }

    EINTRINSIC getType() const override;

    Mat3 K() const;

    void setK(double focalLengthPixX, double focalLengthPixY, double ppx, double ppy);

    void setK(const Mat3& K);

    Vec2 transformProject(const geometry::Pose3& pose, const Vec4& pt3D, bool applyDistortion = true) const
    {
        return transformProject(pose.getHomogeneous(), pt3D, applyDistortion);
    }

    Vec2 transformProject(const Eigen::Matrix4d& pose, const Vec4& pt, bool applyDistortion = true) const override;

    Vec2 project(const Vec4& pt, bool applyDistortion = true) const override;


    Eigen::Matrix<double, 2, 3> getDerivativeTransformProjectWrtPoint3(const Eigen::Matrix4d& pose, const Vec4& pt) const override;

    Eigen::Matrix<double, 2, Eigen::Dynamic> getDerivativeTransformProjectWrtDisto(const Eigen::Matrix4d& pose, const Vec4& pt) const;

    Eigen::Matrix<double, 2, 2> getDerivativeTransformProjectWrtPrincipalPoint(const Eigen::Matrix4d& pose, const Vec4& pt) const;

    Eigen::Matrix<double, 2, 2> getDerivativeTransformProjectWrtScale(const Eigen::Matrix4d& pose, const Vec4& pt) const;

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

    Mat34 getProjectiveEquivalent(const geometry::Pose3& pose) const;

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
