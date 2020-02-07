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

#include <vector>
#include <sstream>


namespace aliceVision {
namespace camera {

/// Define a classic Pinhole camera
class Pinhole : public IntrinsicsScaleOffsetDisto
{
  public:

  Pinhole() = default;

  Pinhole(unsigned int w, unsigned int h, const Mat3 K) : 
  IntrinsicsScaleOffsetDisto(w, h, K(0, 0), K(1, 1), K(0, 2), K(1, 2))
  {
  }

  Pinhole(unsigned int w, unsigned int h, double focal_length_pix, double ppx, double ppy, std::shared_ptr<Distortion> distortion = nullptr)
  : IntrinsicsScaleOffsetDisto(w,h, focal_length_pix, focal_length_pix, ppx, ppy, distortion)
  {
  }

  virtual ~Pinhole() {}

  virtual Pinhole* clone() const override { 
    return new Pinhole(*this); 
  }

  virtual void assign(const IntrinsicBase& other) override { 
    *this = dynamic_cast<const Pinhole&>(other); 
  }
  
  virtual bool isValid() const override { 
    return focal() > 0 && IntrinsicBase::isValid(); 
  }
  
  virtual EINTRINSIC getType() const override { 
    return PINHOLE_CAMERA; 
  }

  Mat3 K() const { 
    Mat3 K;
    
    K  << _scale_x, 0., _offset_y, 0., _scale_y, _offset_y, 0., 0., 1.;

    return K; 
  }

  void setK(double focal_length_pix, double ppx, double ppy)
  {
    _scale_x = focal_length_pix;
    _scale_y = focal_length_pix;
    _offset_x = ppx;
    _offset_y = ppy;
  }
  
  void setK(const Mat3 & K) {
    _scale_x = K(0, 0);
    _scale_y = K(1, 1);
    _offset_x = K(0, 2);
    _offset_y = K(1, 2);
  }

  virtual Vec2 project(const geometry::Pose3& pose, const Vec3& pt, bool applyDistortion = true) const override
  {
    const Vec3 X = pose.rotation() * pt; // apply pose
    const Vec2 P = X.head<2>() / X(2);

    Vec2 distorted = this->add_disto(P);
    Vec2 impt = this->cam2ima(distorted);

    return impt;
  }

  Eigen::Matrix<double, 2, 9> getDerivativeProjectWrtRotation(const geometry::Pose3& pose, const Vec3 & pt) {
    
    const Vec3 X = pose.rotation() * pt; // apply pose

    Eigen::Matrix<double, 3, 9> d_X_d_R = getJacobian_AB_wrt_A<3, 3, 1>(pose.rotation(), pt);

    const Vec2 P = X.head<2>() / X(2);

    Eigen::Matrix<double, 2, 3> d_P_d_X;
    d_P_d_X(0, 0) = 1 / X(2);
    d_P_d_X(0, 1) = 0;
    d_P_d_X(0, 2) = - X(0) / (X(2) * X(2));
    d_P_d_X(1, 0) = 0;
    d_P_d_X(1, 1) = 1 / X(2);
    d_P_d_X(1, 2) = - X(1) / (X(2) * X(2));

    Vec2 distorted = this->add_disto(P);
    Vec2 impt = this->cam2ima(distorted);

    return getDerivativeCam2ImaWrtPoint() * getDerivativeAddDistoWrtPt(P) * d_P_d_X * d_X_d_R;
  }

  Eigen::Matrix<double, 2, 3> getDerivativeProjectWrtPoint(const geometry::Pose3& pose, const Vec3 & pt) {

    const Vec3 X = pose.rotation() * pt; // apply pose

    Eigen::Matrix<double, 3, 3> d_X_d_P = pose.rotation();

    const Vec2 P = X.head<2>() / X(2);

    Eigen::Matrix<double, 2, 3> d_P_d_X;
    d_P_d_X(0, 0) = 1 / X(2);
    d_P_d_X(0, 1) = 0;
    d_P_d_X(0, 2) = - X(0) / (X(2) * X(2));
    d_P_d_X(1, 0) = 0;
    d_P_d_X(1, 1) = 1 / X(2);
    d_P_d_X(1, 2) = - X(1) / (X(2) * X(2));

    Vec2 distorted = this->add_disto(P);
    Vec2 impt = this->cam2ima(distorted);

    return getDerivativeCam2ImaWrtPoint() * getDerivativeAddDistoWrtPt(P) * d_P_d_X * d_X_d_P;
  }

  Eigen::Matrix<double, 2, 3> getDerivativeProjectWrtDisto(const geometry::Pose3& pose, const Vec3 & pt) {
    const Vec3 X = pose.rotation() * pt; // apply pose
    const Vec2 P = X.head<2>() / X(2);

    Vec2 distorted = this->add_disto(P);
    Vec2 impt = this->cam2ima(distorted);

    return getDerivativeCam2ImaWrtPoint() * getDerivativeAddDistoWrtDisto(P);
  }
  
  Eigen::Matrix<double, 2, 2> getDerivativeProjectWrtPrincipalPoint(const geometry::Pose3& pose, const Vec3 & pt) {

    return getDerivativeCam2ImaWrtPrincipalPoint();
  }

  Eigen::Matrix<double, 2, 1> getDerivativeProjectWrtScale(const geometry::Pose3& pose, const Vec3 & pt) {

    const Vec3 X = pose.rotation() * pt; // apply pose
    const Vec2 P = X.head<2>() / X(2);

    Vec2 distorted = this->add_disto(P);

    return getDerivativeCam2ImaWrtScale(distorted);
  }

  virtual Vec3 toUnitSphere(const Vec2 & pt) const override {

    Vec3 ptcam = pt.homogeneous();

    return ptcam / ptcam.norm();
  }

  Eigen::Matrix<double, 3, 2> getDerivativetoUnitSphereWrtPoint(const Vec2 & pt) {

    double norm2 = pt(0)*pt(0) + pt(1)*pt(1) + 1.0;
    double norm = sqrt(norm2);

    Vec3 ptcam = pt.homogeneous();

    
    Eigen::Matrix<double, 1, 2> d_norm_d_pt;
    d_norm_d_pt(0, 0) = pt(0) / norm;
    d_norm_d_pt(0, 1) = pt(1) / norm;

    Eigen::Matrix<double, 3, 2> d_ptcam_d_pt;
    d_ptcam_d_pt(0, 0) = 1.0;
    d_ptcam_d_pt(0, 1) = 0.0;
    d_ptcam_d_pt(1, 0) = 0.0;
    d_ptcam_d_pt(1, 1) = 1.0;
    d_ptcam_d_pt(2, 0) = 0.0;
    d_ptcam_d_pt(2, 1) = 0.0;

    return (norm * d_ptcam_d_pt - ptcam * d_norm_d_pt) / norm2;
  }
  
  virtual double imagePlane_toCameraPlaneError(double value) const override
  {
    return value / focal();
  }

  virtual Mat34 get_projective_equivalent(const geometry::Pose3 & pose) const
  {
    Mat34 P;
    Mat3 K = Eigen::Matrix3d::Identity();
    K(0, 0) = _scale_x;
    K(1, 1) = _scale_y;
    K(0, 2) = _offset_x;
    K(1, 2) = _offset_y;

    P_From_KRt(K, pose.rotation(), pose.translation(), &P);
    return P;
  }

  /**
   * @brief Return true if this ray should be visible in the image
   * @return true if this ray is visible theorically
   */
  virtual bool isVisibleRay(const Vec3 & ray) const override {
    
    if (ray(2) < 0) {
      return false;
    }

    Vec2 proj = ray.head(2) / ray(2);

    double xmin;
    double ymin;
    double xmax;
    double ymax;

    Vec2 p1 = remove_disto(ima2cam(Vec2(0,0)));
    Vec2 p2 = remove_disto(ima2cam(Vec2(_w,0)));
    Vec2 p3 = remove_disto(ima2cam(Vec2(_w,_h)));
    Vec2 p4 = remove_disto(ima2cam(Vec2(0,_h)));

    xmin = std::min(p4(0), (std::min(p3(0), std::min(p1(0), p2(0)))));
    ymin = std::min(p4(1), (std::min(p3(1), std::min(p1(1), p2(1)))));
    xmax = std::max(p4(0), (std::max(p3(0), std::max(p1(0), p2(0)))));
    ymax = std::max(p4(1), (std::max(p3(1), std::max(p1(1), p2(1)))));

    if (proj(0) < xmin || proj(0) > xmax || proj(1) < ymin || proj(1) > ymax) {
      return false;
    }

    return true;
  }
};

} // namespace camera
} // namespace aliceVision
