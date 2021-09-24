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

#include <vector>
#include <sstream>
#include <cmath>


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

  EquiDistant(unsigned int w, unsigned int h, double focalLengthPix, double ppx, double ppy,
              std::shared_ptr<Distortion> distortion = nullptr)
      : IntrinsicsScaleOffsetDisto(w, h, focalLengthPix, focalLengthPix, ppx, ppy, distortion)
      , _circleRadius(std::min(w, h) * 0.5)
      , _circleCenter(w / 2.0, h / 2.0)
  {
  }

  EquiDistant(unsigned int w, unsigned int h, double focalLengthPix, double ppx, double ppy, double circleRadiusPix,
              std::shared_ptr<Distortion> distortion = nullptr)
      : IntrinsicsScaleOffsetDisto(w, h, focalLengthPix, focalLengthPix, ppx, ppy, distortion)
      , _circleRadius(circleRadiusPix)
      , _circleCenter(w / 2.0, h / 2.0)
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

  Vec2 project(const geometry::Pose3& pose, const Vec4& pt, bool applyDistortion = true) const override
  {
    const double rsensor = std::min(sensorWidth(), sensorHeight());
    const double rscale = sensorWidth() / std::max(w(), h());
    const double fmm = _scale(0) * rscale;
    const double fov = rsensor / fmm;

    const Vec4 X = pose.getHomogeneous() * pt;

    // Compute angle with optical center
    const double angle_Z = std::atan2(sqrt(X(0) * X(0) + X(1) * X(1)), X(2));
    
    // Ignore depth component and compute radial angle
    const double angle_radial = std::atan2(X(1), X(0));

    const double radius = angle_Z / (0.5 * fov);

    // radius = focal * angle_Z
    const Vec2 P{cos(angle_radial) * radius, sin(angle_radial) * radius};

    const Vec2 pt_disto = applyDistortion ? this->addDistortion(P) : P;
    const Vec2 pt_ima = this->cam2ima(pt_disto);

    return pt_ima;
  }

  Eigen::Matrix<double, 2, 9> getDerivativeProjectWrtRotation(const geometry::Pose3& pose, const Vec4 & pt) 
  {
    Eigen::Matrix4d T = pose.getHomogeneous();
    const Vec4 X = T * pt; // apply pose

    const Eigen::Matrix<double, 3, 9> d_X_d_R = getJacobian_AB_wrt_A<3, 3, 1>(pose.rotation(), pt.head(3));

    /* Compute angle with optical center */
    double len2d = sqrt(X(0) * X(0) + X(1) * X(1));
    Eigen::Matrix<double, 2, 2> d_len2d_d_X;
    d_len2d_d_X(0) = X(0) / len2d;
    d_len2d_d_X(1) = X(1) / len2d;
    
    const double angle_Z = std::atan2(len2d, X(2));
    const double d_angle_Z_d_len2d = X(2) / (len2d*len2d + X(2) * X(2));

    /* Ignore depth component and compute radial angle */
    const double angle_radial = std::atan2(X(1), X(0));

    Eigen::Matrix<double, 2, 3> d_angles_d_X;
    d_angles_d_X(0, 0) = - X(1) / (X(0) * X(0) + X(1) * X(1));
    d_angles_d_X(0, 1) = X(0) / (X(0) * X(0) + X(1) * X(1));
    d_angles_d_X(0, 2) = 0.0;

    d_angles_d_X(1, 0) = d_angle_Z_d_len2d * d_len2d_d_X(0);
    d_angles_d_X(1, 1) = d_angle_Z_d_len2d * d_len2d_d_X(1);
    d_angles_d_X(1, 2) = - len2d / (len2d * len2d + X(2) * X(2));

    const double rsensor = std::min(sensorWidth(), sensorHeight());
    const double rscale = sensorWidth() / std::max(w(), h());
    const double fmm = _scale(0) * rscale;
    const double fov = rsensor / fmm;

    const double radius = angle_Z / (0.5 * fov);

    const double d_radius_d_angle_Z = 1.0 / (0.5 * fov);

    /* radius = focal * angle_Z */
    const Vec2 P{cos(angle_radial) * radius, sin(angle_radial) * radius};

    Eigen::Matrix<double, 2, 2> d_P_d_angles;
    d_P_d_angles(0, 0) = - sin(angle_radial) * radius;
    d_P_d_angles(0, 1) = cos(angle_radial) * d_radius_d_angle_Z;
    d_P_d_angles(1, 0) = cos(angle_radial) * radius;
    d_P_d_angles(1, 1) = sin(angle_radial) * d_radius_d_angle_Z;

    return getDerivativeCam2ImaWrtPoint() * getDerivativeAddDistoWrtPt(P) * d_P_d_angles * d_angles_d_X * d_X_d_R;
  }

  Eigen::Matrix<double, 2, 16> getDerivativeProjectWrtPose(const geometry::Pose3& pose, const Vec4 & pt) const override
  {
    Eigen::Matrix4d T = pose.getHomogeneous();
    const Vec4 X = T * pt; // apply pose

    const Eigen::Matrix<double, 4, 16> d_X_d_T = getJacobian_AB_wrt_A<4, 4, 1>(T, pt);

    /* Compute angle with optical center */
    double len2d = sqrt(X(0) * X(0) + X(1) * X(1));
    Eigen::Matrix<double, 2, 2> d_len2d_d_X;
    d_len2d_d_X(0) = X(0) / len2d;
    d_len2d_d_X(1) = X(1) / len2d;
    
    const double angle_Z = std::atan2(len2d, X(2));
    const double d_angle_Z_d_len2d = X(2) / (len2d*len2d + X(2) * X(2));

    /* Ignore depth component and compute radial angle */
    const double angle_radial = std::atan2(X(1), X(0));

    Eigen::Matrix<double, 2, 3> d_angles_d_X;
    d_angles_d_X(0, 0) = - X(1) / (X(0) * X(0) + X(1) * X(1));
    d_angles_d_X(0, 1) = X(0) / (X(0) * X(0) + X(1) * X(1));
    d_angles_d_X(0, 2) = 0.0;

    d_angles_d_X(1, 0) = d_angle_Z_d_len2d * d_len2d_d_X(0);
    d_angles_d_X(1, 1) = d_angle_Z_d_len2d * d_len2d_d_X(1);
    d_angles_d_X(1, 2) = - len2d / (len2d * len2d + X(2) * X(2));

    const double rsensor = std::min(sensorWidth(), sensorHeight());
    const double rscale = sensorWidth() / std::max(w(), h());
    const double fmm = _scale(0) * rscale;
    const double fov = rsensor / fmm;

    const double radius = angle_Z / (0.5 * fov);

    const double d_radius_d_angle_Z = 1.0 / (0.5 * fov);

    /* radius = focal * angle_Z */
    const Vec2 P{cos(angle_radial) * radius, sin(angle_radial) * radius};

    Eigen::Matrix<double, 2, 2> d_P_d_angles;
    d_P_d_angles(0, 0) = - sin(angle_radial) * radius;
    d_P_d_angles(0, 1) = cos(angle_radial) * d_radius_d_angle_Z;
    d_P_d_angles(1, 0) = cos(angle_radial) * radius;
    d_P_d_angles(1, 1) = sin(angle_radial) * d_radius_d_angle_Z;

    return getDerivativeCam2ImaWrtPoint() * getDerivativeAddDistoWrtPt(P) * d_P_d_angles * d_angles_d_X * d_X_d_T.block<3, 16>(0, 0);
  }

  Eigen::Matrix<double, 2, 4> getDerivativeProjectWrtPoint(const geometry::Pose3& pose, const Vec4 & pt) const override
  {
    Eigen::Matrix4d T = pose.getHomogeneous();
    const Vec4 X = T * pt; // apply pose

    const Eigen::Matrix4d& d_X_d_pt = T;

    /* Compute angle with optical center */
    const double len2d = sqrt(X(0) * X(0) + X(1) * X(1));
    Eigen::Matrix<double, 2, 2> d_len2d_d_X;
    d_len2d_d_X(0) = X(0) / len2d;
    d_len2d_d_X(1) = X(1) / len2d;
    
    const double angle_Z = std::atan2(len2d, X(2));
    const double d_angle_Z_d_len2d = X(2) / (len2d*len2d + X(2) * X(2));

    /* Ignore depth component and compute radial angle */
    const double angle_radial = std::atan2(X(1), X(0));

    Eigen::Matrix<double, 2, 4> d_angles_d_X;
    d_angles_d_X(0, 0) = - X(1) / (X(0) * X(0) + X(1) * X(1));
    d_angles_d_X(0, 1) = X(0) / (X(0) * X(0) + X(1) * X(1));
    d_angles_d_X(0, 2) = 0.0;
    d_angles_d_X(0, 3) = 0.0;

    d_angles_d_X(1, 0) = d_angle_Z_d_len2d * d_len2d_d_X(0);
    d_angles_d_X(1, 1) = d_angle_Z_d_len2d * d_len2d_d_X(1);
    d_angles_d_X(1, 2) = - len2d / (len2d * len2d + X(2) * X(2));
    d_angles_d_X(1, 3) = 0.0;

    const double rsensor = std::min(sensorWidth(), sensorHeight());
    const double rscale = sensorWidth() / std::max(w(), h());
    const double fmm = _scale(0) * rscale;
    const double fov = rsensor / fmm;
    const double radius = angle_Z / (0.5 * fov);

    double d_radius_d_angle_Z = 1.0 / (0.5 * fov);

    /* radius = focal * angle_Z */
    const Vec2 P{cos(angle_radial) * radius, sin(angle_radial) * radius};

    Eigen::Matrix<double, 2, 2> d_P_d_angles;
    d_P_d_angles(0, 0) = - sin(angle_radial) * radius;
    d_P_d_angles(0, 1) = cos(angle_radial) * d_radius_d_angle_Z;
    d_P_d_angles(1, 0) = cos(angle_radial) * radius;
    d_P_d_angles(1, 1) = sin(angle_radial) * d_radius_d_angle_Z;

    return getDerivativeCam2ImaWrtPoint() * getDerivativeAddDistoWrtPt(P) * d_P_d_angles * d_angles_d_X * d_X_d_pt;
  }

  Eigen::Matrix<double, 2, 3> getDerivativeProjectWrtDisto(const geometry::Pose3& pose, const Vec4 & pt)
  {
    Eigen::Matrix4d T = pose.getHomogeneous();
    const Vec4 X = T * pt; // apply pose

    /* Compute angle with optical center */
    const double len2d = sqrt(X(0) * X(0) + X(1) * X(1));
    const double angle_Z = std::atan2(len2d, X(2));
  
    /* Ignore depth component and compute radial angle */
    const double angle_radial = std::atan2(X(1), X(0));

    const double rsensor = std::min(sensorWidth(), sensorHeight());
    const double rscale = sensorWidth() / std::max(w(), h());
    const double fmm = _scale(0) * rscale;
    const double fov = rsensor / fmm;
    const double radius = angle_Z / (0.5 * fov);

    /* radius = focal * angle_Z */
    const Vec2 P{cos(angle_radial) * radius, sin(angle_radial) * radius};

    return getDerivativeCam2ImaWrtPoint() * getDerivativeAddDistoWrtDisto(P);
  }

  Eigen::Matrix<double, 2, 2> getDerivativeProjectWrtScale(const geometry::Pose3& pose, const Vec4 & pt)
  {
    Eigen::Matrix4d T = pose.getHomogeneous();
    const Vec4 X = T * pt; // apply pose

    /* Compute angle with optical center */
    const double len2d = sqrt(X(0) * X(0) + X(1) * X(1));
    const double angle_Z = std::atan2(len2d, X(2));
  
    /* Ignore depth component and compute radial angle */
    const double angle_radial = std::atan2(X(1), X(0));

    const double rsensor = std::min(sensorWidth(), sensorHeight());
    const double rscale = sensorWidth() / std::max(w(), h());
    const double fmm = _scale(0) * rscale;
    const double fov = rsensor / fmm;
    const double radius = angle_Z / (0.5 * fov);

    /* radius = focal * angle_Z */
    const Vec2 P{cos(angle_radial) * radius, sin(angle_radial) * radius};

    Eigen::Matrix<double, 2, 1> d_P_d_radius;
    d_P_d_radius(0, 0) = cos(angle_radial);
    d_P_d_radius(1, 0) = sin(angle_radial);

    Eigen::Matrix<double, 1, 1> d_radius_d_fov;
    d_radius_d_fov(0, 0) = (- 2.0 * angle_Z / (fov * fov));

    Eigen::Matrix<double, 1, 2> d_fov_d_scale;
    d_fov_d_scale(0, 0) = -rsensor / (_scale(0) * _scale(0) * rscale);
    d_fov_d_scale(0, 1) = 0.0;

    return getDerivativeCam2ImaWrtPoint() * getDerivativeAddDistoWrtPt(P) * d_P_d_radius * d_radius_d_fov * d_fov_d_scale;
  }

  Eigen::Matrix<double, 2, 2> getDerivativeProjectWrtPrincipalPoint(const geometry::Pose3& pose, const Vec4 & pt)
  {
    return getDerivativeCam2ImaWrtPrincipalPoint();
  }

  Eigen::Matrix<double, 2, Eigen::Dynamic> getDerivativeProjectWrtParams(const geometry::Pose3& pose, const Vec4& pt3D) const override {
    return Eigen::Matrix<double, 2, Eigen::Dynamic>(2, 6);
  }

  Vec3 toUnitSphere(const Vec2 & pt) const override
  {
    const double rsensor = std::min(sensorWidth(), sensorHeight());
    const double rscale = sensorWidth() / std::max(w(), h());
    const double fmm = _scale(0) * rscale;
    const double fov = rsensor / fmm;

    const double angle_radial = atan2(pt(1), pt(0));
    const double angle_Z = pt.norm() * 0.5 * fov;

    const Vec3 ret{cos(angle_radial) /** / 1.0 / **/ * sin(angle_Z),
                   sin(angle_radial) /** / 1.0 / **/ * sin(angle_Z),
                   cos(angle_Z)};

    return ret;
  }

  Eigen::Matrix<double, 3, 2> getDerivativetoUnitSphereWrtPoint(const Vec2 & pt)
  {
    const double rsensor = std::min(sensorWidth(), sensorHeight());
    const double rscale = sensorWidth() / std::max(w(), h());
    const double fmm = _scale(0) * rscale;
    const double fov = rsensor / fmm;

    const double angle_radial = atan2(pt(1), pt(0));
    const double angle_Z = pt.norm() * 0.5 * fov;

    Eigen::Matrix<double, 3, 2> d_ret_d_angles;
    d_ret_d_angles(0, 0) = -sin(angle_radial) * sin(angle_Z);
    d_ret_d_angles(0, 1) = cos(angle_radial) * cos(angle_Z);
    d_ret_d_angles(1, 0) = cos(angle_radial) * sin(angle_Z);
    d_ret_d_angles(1, 1) = sin(angle_radial) * cos(angle_Z);
    d_ret_d_angles(2, 0) = 0;
    d_ret_d_angles(2, 1) = -sin(angle_Z);

    Eigen::Matrix<double, 2, 2> d_angles_d_pt;
    d_angles_d_pt(0, 0) = - pt(1) / (pt(0) * pt(0) + pt(1) * pt(1));
    d_angles_d_pt(0, 1) = pt(0) / (pt(0) * pt(0) + pt(1) * pt(1));
    d_angles_d_pt(1, 0) = 0.5 * fov * pt(0) / pt.norm();
    d_angles_d_pt(1, 1) = 0.5 * fov * pt(1) / pt.norm();

    return d_ret_d_angles * d_angles_d_pt;
  }

  Eigen::Matrix<double, 3, 2> getDerivativetoUnitSphereWrtScale(const Vec2 & pt)
  {
    const double rsensor = std::min(sensorWidth(), sensorHeight());
    const double rscale = sensorWidth() / std::max(w(), h());
    const double fmm = _scale(0) * rscale;
    const double fov = rsensor / fmm;

    const double angle_radial = atan2(pt(1), pt(0));
    const double angle_Z = pt.norm() * 0.5 * fov;

    Eigen::Matrix<double, 3, 2> d_ret_d_angles;
    d_ret_d_angles(0, 0) = -sin(angle_radial) * sin(angle_Z);
    d_ret_d_angles(0, 1) = cos(angle_radial) * cos(angle_Z);
    d_ret_d_angles(1, 0) = cos(angle_radial) * sin(angle_Z);
    d_ret_d_angles(1, 1) = sin(angle_radial) * cos(angle_Z);
    d_ret_d_angles(2, 0) = 0;
    d_ret_d_angles(2, 1) = -sin(angle_Z);

    Eigen::Matrix<double, 2, 1> d_angles_d_fov;

    d_angles_d_fov(0, 0) = 0;
    d_angles_d_fov(1, 0) = pt.norm() * 0.5;

    Eigen::Matrix<double, 1, 2> d_fov_d_scale;
    d_fov_d_scale(0, 0) = -rsensor / (_scale(0) * _scale(0) * rscale);  
    d_fov_d_scale(0, 1) = 0.0;  

    return d_ret_d_angles * d_angles_d_fov * d_fov_d_scale;
  }

  double imagePlaneToCameraPlaneError(double value) const override
  {
    return value / _scale(0);
  }

  // Transform a point from the camera plane to the image plane
  Vec2 cam2ima(const Vec2& p) const override
  {
    return _circleRadius * p  + getPrincipalPoint();
  }

  Eigen::Matrix2d getDerivativeCam2ImaWrtPoint() const override
  {
    return Eigen::Matrix2d::Identity() * _circleRadius;
  }

  // Transform a point from the image plane to the camera plane
  Vec2 ima2cam(const Vec2& p) const override
  {
    return (p - getPrincipalPoint()) / _circleRadius;
  }

  Eigen::Matrix2d getDerivativeIma2CamWrtPoint() const override
  {
    return Eigen::Matrix2d::Identity() * (1.0 / _circleRadius);
  }

  Eigen::Matrix2d getDerivativeIma2CamWrtPrincipalPoint() const override
  {
    return Eigen::Matrix2d::Identity() * (-1.0 / _circleRadius);
  }

  /**
   * @brief Return true if this ray should be visible in the image
   * @return true if this ray is visible theorically
   */
  bool isVisibleRay(const Vec3 & ray) const override
  {
    const double rsensor = std::min(sensorWidth(), sensorHeight());
    const double rscale = sensorWidth() / std::max(w(), h());
    const double fmm = _scale(0) * rscale;
    const double fov = rsensor / fmm;

    double angle = std::acos(ray.normalized().dot(Eigen::Vector3d::UnitZ()));
    if(std::abs(angle) > 1.2 * (0.5 * fov))
      return false;

    const Vec2 proj = project(geometry::Pose3(), ray.homogeneous(), true);
    const Vec2 centered = proj - Vec2(_circleCenter(0), _circleCenter(1));
    return  centered.norm() <= _circleRadius;
  }

  double getCircleRadius() const
  {
    return _circleRadius;
  }

  void setCircleRadius(double radius)
  {
    _circleRadius = radius;
  }

  double getCircleCenterX() const
  {
    return _circleCenter(0);
  }

  void setCircleCenterX(double x)
  {
    _circleCenter(0) = x;
  }

  double getCircleCenterY() const
  {
    return _circleCenter(1);
  }

  void setCircleCenterY(double y)
  {
    _circleCenter(1) = y;
  }

protected:
  double _circleRadius{0.0};
  Vec2 _circleCenter{0.0, 0.0};
};

} // namespace camera
} // namespace aliceVision
