// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/numeric/numeric.hpp>
#include <aliceVision/camera/cameraCommon.hpp>
#include <aliceVision/camera/IntrinsicsScaleOffsetDisto.hpp>
#include <aliceVision/geometry/Pose3.hpp>
#include <aliceVision/multiview/projection.hpp>

#include "DistortionFisheye1.hpp"

#include <vector>
#include <sstream>
#include <cmath>


namespace aliceVision {
namespace camera {

/// Define a classic Pinhole camera
class EquiDistant : public IntrinsicsScaleOffsetDisto
{
public:

  EquiDistant() = default;

  EquiDistant(unsigned int w, unsigned int h, double fov, double ppx, double ppy, double radiuspixels = 1980.00, std::shared_ptr<Distortion> distortion = nullptr)
  : IntrinsicsScaleOffsetDisto(w, h, fov, fov, ppx, ppy, distortion), _radius(radiuspixels), _center_x(w/2.0), _center_y(h/2.0)
  {
  }

  ~EquiDistant() override = default;

  EquiDistant* clone() const override {
    return new EquiDistant(*this); 
  }

  void assign(const IntrinsicBase& other) override {
    *this = dynamic_cast<const EquiDistant&>(other); 
  }
  
  bool isValid() const override {
    return focal() > 0 && IntrinsicBase::isValid(); 
  }
  
  EINTRINSIC getType() const override {
    return EQUIDISTANT_CAMERA; 
  }

  Vec2 project(const geometry::Pose3& pose, const Vec3& pt, bool applyDistortion = true) const override
  {    
    double rsensor = std::min(sensorWidth(), sensorHeight());
    double rscale = sensorWidth() / std::max(w(), h());
    double fmm = _scale_x * rscale;
    double fov = rsensor / fmm;

    const Vec3 X = pose.rotation() * pt;

    /* Compute angle with optical center */
    double angle_Z = std::atan2(sqrt(X(0) * X(0) + X(1) * X(1)), X(2));
    
    /* Ignore depth component and compute radial angle */
    double angle_radial = std::atan2(X(1), X(0));

    double radius = angle_Z / (0.5 * fov);

    /* radius = focal * angle_Z */
    Vec2 P;
    P(0) = cos(angle_radial) * radius;
    P(1) = sin(angle_radial) * radius;

  
    const Vec2 pt_disto = this->add_disto(P);
    const Vec2 pt_ima = this->cam2ima(pt_disto);

    return pt_ima;
  }

  Eigen::Matrix<double, 2, 9> getDerivativeProjectWrtRotation(const geometry::Pose3& pose, const Vec3 & pt)
  {
    
    const Vec3 X = pose.rotation() * pt;

    const Eigen::Matrix<double, 3, 9> d_X_d_R = getJacobian_AB_wrt_A<3, 3, 1>(pose.rotation(), pt);

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
    const double fmm = _scale_x * rscale;
    const double fov = rsensor / fmm;

    const double radius = angle_Z / (0.5 * fov);

    const double d_radius_d_angle_Z = 1.0 / (0.5 * fov);

    /* radius = focal * angle_Z */
    Vec2 P;
    P(0) = cos(angle_radial) * radius;
    P(1) = sin(angle_radial) * radius;

    Eigen::Matrix<double, 2, 2> d_P_d_angles;
    d_P_d_angles(0, 0) = - sin(angle_radial) * radius;
    d_P_d_angles(0, 1) = cos(angle_radial) * d_radius_d_angle_Z;
    d_P_d_angles(1, 0) = cos(angle_radial) * radius;
    d_P_d_angles(1, 1) = sin(angle_radial) * d_radius_d_angle_Z;

    const Vec2 distorted = this->add_disto(P);
    const Vec2 impt = this->cam2ima(distorted);

    return getDerivativeCam2ImaWrtPoint() * getDerivativeAddDistoWrtPt(P) * d_P_d_angles * d_angles_d_X * d_X_d_R;
  }

  Eigen::Matrix<double, 2, 3> getDerivativeProjectWrtPoint(const geometry::Pose3& pose, const Vec3 & pt)
  {

    Vec3 X = pose.rotation() * pt;

    const Eigen::Matrix3d& d_X_d_pt = pose.rotation();

    /* Compute angle with optical center */
    const double len2d = sqrt(X(0) * X(0) + X(1) * X(1));
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
    const double fmm = _scale_x * rscale;
    const double fov = rsensor / fmm;
    const double radius = angle_Z / (0.5 * fov);

    double d_radius_d_angle_Z = 1.0 / (0.5 * fov);

    /* radius = focal * angle_Z */
    Vec2 P;
    P(0) = cos(angle_radial) * radius;
    P(1) = sin(angle_radial) * radius;

    Eigen::Matrix<double, 2, 2> d_P_d_angles;
    d_P_d_angles(0, 0) = - sin(angle_radial) * radius;
    d_P_d_angles(0, 1) = cos(angle_radial) * d_radius_d_angle_Z;
    d_P_d_angles(1, 0) = cos(angle_radial) * radius;
    d_P_d_angles(1, 1) = sin(angle_radial) * d_radius_d_angle_Z;

    const Vec2 distorted = this->add_disto(P);
    const Vec2 impt = this->cam2ima(distorted);

    return getDerivativeCam2ImaWrtPoint() * getDerivativeAddDistoWrtPt(P) * d_P_d_angles * d_angles_d_X * d_X_d_pt;
  }

  Eigen::Matrix<double, 2, 3> getDerivativeProjectWrtDisto(const geometry::Pose3& pose, const Vec3 & pt)
  {
    
    const Vec3 X = pose.rotation() * pt;

    /* Compute angle with optical center */
    const double len2d = sqrt(X(0) * X(0) + X(1) * X(1));
    const double angle_Z = std::atan2(len2d, X(2));
  
    /* Ignore depth component and compute radial angle */
    const double angle_radial = std::atan2(X(1), X(0));

    const double rsensor = std::min(sensorWidth(), sensorHeight());
    const double rscale = sensorWidth() / std::max(w(), h());
    const double fmm = _scale_x * rscale;
    const double fov = rsensor / fmm;
    const double radius = angle_Z / (0.5 * fov);

    /* radius = focal * angle_Z */
    Vec2 P;
    P(0) = cos(angle_radial) * radius;
    P(1) = sin(angle_radial) * radius;

    const Vec2 distorted = this->add_disto(P);
    const Vec2 impt = this->cam2ima(distorted);

    return getDerivativeCam2ImaWrtPoint() * getDerivativeAddDistoWrtDisto(P);
  }

  Eigen::Matrix<double, 2, 1> getDerivativeProjectWrtScale(const geometry::Pose3& pose, const Vec3 & pt)
  {

    const Vec3 X = pose.rotation() * pt;

    /* Compute angle with optical center */
    const double len2d = sqrt(X(0) * X(0) + X(1) * X(1));
    const double angle_Z = std::atan2(len2d, X(2));
  
    /* Ignore depth component and compute radial angle */
    const double angle_radial = std::atan2(X(1), X(0));

    const double rsensor = std::min(sensorWidth(), sensorHeight());
    const double rscale = sensorWidth() / std::max(w(), h());
    const double fmm = _scale_x * rscale;
    const double fov = rsensor / fmm;
    const double radius = angle_Z / (0.5 * fov);

    /* radius = focal * angle_Z */
    Vec2 P;
    P(0) = cos(angle_radial) * radius;
    P(1) = sin(angle_radial) * radius;

    Eigen::Matrix<double, 2, 1> d_P_d_radius;
    d_P_d_radius(0, 0) = cos(angle_radial);
    d_P_d_radius(1, 0) = sin(angle_radial);

    const Vec2 distorted = this->add_disto(P);
    /*Vec2 impt = this->cam2ima(distorted);*/

    Eigen::Matrix<double, 1, 1> d_radius_d_fov;
    d_radius_d_fov(0, 0) = (- 2.0 * angle_Z / (fov * fov));

    Eigen::Matrix<double, 1, 1> d_fov_d_scale;
    d_fov_d_scale(0, 0) = - rsensor / (_scale_x * _scale_x * rscale);


    return getDerivativeCam2ImaWrtPoint() * getDerivativeAddDistoWrtPt(P) * d_P_d_radius * d_radius_d_fov * d_fov_d_scale;
  }

  Eigen::Matrix<double, 2, 2> getDerivativeProjectWrtPrincipalPoint(const geometry::Pose3& pose, const Vec3 & pt)
  {

    return getDerivativeCam2ImaWrtPrincipalPoint();
  }

  Vec3 toUnitSphere(const Vec2 & pt) const override
  {

    const double rsensor = std::min(sensorWidth(), sensorHeight());
    const double rscale = sensorWidth() / std::max(w(), h());
    const double fmm = _scale_x * rscale;
    const double fov = rsensor / fmm;

    const double angle_radial = atan2(pt(1), pt(0));
    const double angle_Z = pt.norm() * 0.5 * fov;

    Vec3 ret;
    ret(0) = cos(angle_radial) /** / 1.0 / **/ * sin(angle_Z);
    ret(1) = sin(angle_radial) /** / 1.0 / **/ * sin(angle_Z);
    ret(2) = cos(angle_Z);

    return ret;
  }

  Eigen::Matrix<double, 3, 2> getDerivativetoUnitSphereWrtPoint(const Vec2 & pt)
  {

    const double rsensor = std::min(sensorWidth(), sensorHeight());
    const double rscale = sensorWidth() / std::max(w(), h());
    const double fmm = _scale_x * rscale;
    const double fov = rsensor / fmm;

    const double angle_radial = atan2(pt(1), pt(0));
    const double angle_Z = pt.norm() * 0.5 * fov;

    Vec3 ret;
    ret(0) = cos(angle_radial) /** / 1.0 / **/ * sin(angle_Z);
    ret(1) = sin(angle_radial) /** / 1.0 / **/ * sin(angle_Z);
    ret(2) = cos(angle_Z);

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

  Eigen::Matrix<double, 3, 1> getDerivativetoUnitSphereWrtScale(const Vec2 & pt)
  {

    const double rsensor = std::min(sensorWidth(), sensorHeight());
    const double rscale = sensorWidth() / std::max(w(), h());
    const double fmm = _scale_x * rscale;
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

    Eigen::Matrix<double, 1, 1> d_fov_d_scale;
    d_fov_d_scale(0, 0) = - rsensor / (_scale_x * _scale_x * rscale);  

    return d_ret_d_angles * d_angles_d_fov * d_fov_d_scale;
  }
  
  double imagePlane_toCameraPlaneError(double value) const override
  {
    return value / focal();
  }

  // Transform a point from the camera plane to the image plane
  Vec2 cam2ima(const Vec2& p) const override
  {
    return _radius * p  + principal_point();
  }

  Eigen::Matrix2d getDerivativeCam2ImaWrtPoint() const override
  {

    return Eigen::Matrix2d::Identity() * _radius;
  }

  // Transform a point from the image plane to the camera plane
  Vec2 ima2cam(const Vec2& p) const override
  {
    return (p -  principal_point()) / _radius;
  }

  Eigen::Matrix2d getDerivativeIma2CamWrtPoint() const override
  {

    return Eigen::Matrix2d::Identity() * (1.0 / _radius);
  }

  Eigen::Matrix2d getDerivativeIma2CamWrtPrincipalPoint() const override
  {

    return Eigen::Matrix2d::Identity() * (-1.0 / _radius);
  }

  /**
   * @brief Return true if this ray should be visible in the image
   * @return true if this ray is visible theorically
   */
  bool isVisibleRay(const Vec3 & ray) const override
  {
    const double rsensor = std::min(sensorWidth(), sensorHeight());
    const double rscale = sensorWidth() / std::max(w(), h());
    const double fmm = _scale_x * rscale;
    const double fov = rsensor / fmm;

    double angle = std::acos(ray.normalized().dot(Eigen::Vector3d::UnitZ()));
    if (std::abs(angle) > 1.2 * (0.5 * fov)) return false;

    const Vec2 proj = project(geometry::Pose3(), ray, true);

    const Vec2 centered = proj - Vec2(_center_x, _center_y);

    return  centered.norm() <= _radius;
  }

  double getRadius() const
  {
    return _radius;
  }

  void setRadius(double radius)
  {
    _radius = radius;
  }

  double getCenterX() const
  {
    return _center_x;
  }

  void setCenterX(double x)
  {
    _center_x = x;
  }

  double getCenterY() const
  {
    return _center_y;
  }

  void setCenterY(double y)
  {
    _center_y = y;
  }

protected:
  double _radius{};
  double _center_x{};
  double _center_y{};
};

} // namespace camera
} // namespace aliceVision
