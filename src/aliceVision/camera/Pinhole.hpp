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

  virtual Vec2 project(const geometry::Pose3& pose, const Vec3& pt3D, bool applyDistortion = true) const override
  {
    const Vec3 X = pose(pt3D); // apply pose
    const Vec2 P = X.head<2>() / X(2);

    if (applyDistortion && this->have_disto()) {
      return this->cam2ima( this->add_disto(P));
    }
    else {
      return this->cam2ima(P);
    }
  }

  virtual Vec3 toUnitSphere(const Vec2 & pt) const override {

    Vec3 ptcam = (ima2cam(pt)).homogeneous();

    return ptcam / ptcam.norm();
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
