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

  virtual ~EquiDistant() {}

  virtual EquiDistant* clone() const override { 
    return new EquiDistant(*this); 
  }

  virtual void assign(const IntrinsicBase& other) override { 
    *this = dynamic_cast<const EquiDistant&>(other); 
  }
  
  virtual bool isValid() const override { 
    return focal() > 0 && IntrinsicBase::isValid(); 
  }
  
  virtual EINTRINSIC getType() const override { 
    return EQUIDISTANT_CAMERA; 
  }

  virtual Vec2 project(const geometry::Pose3& pose, const Vec3& pt3D, bool applyDistortion = true) const override
  {
    Vec3 X = pose(pt3D);
    
    /* To unit sphere */
    X.normalize();

    /* Compute angle with optical center */
    double angle_Z = std::atan2(sqrt(X(0) * X(0) + X(1) * X(1)), X(2));
    
    /* Ignore depth component and compute radial angle */
    double angle_radial = std::atan2(X(1), X(0));

    double fov = _scale_x;
    double radius = angle_Z / fov;

    /* radius = focal * angle_Z */
    Vec2 P;
    P(0) = cos(angle_radial) * radius;
    P(1) = sin(angle_radial) * radius;

    if (applyDistortion && this->have_disto()) {
      return this->cam2ima(this->add_disto(P));
    }
    else {
      return this->cam2ima(P);
    }

    return P;
  }

  virtual Vec3 toUnitSphere(const Vec2 & pt) const override {

    const Vec2 camcoords = ima2cam(pt);

    double fov = _scale_x;
    double angle_radial = atan2(camcoords(1), camcoords(0));
    double angle_Z = camcoords.norm() * fov;

    Vec3 ret;
    ret(2) = cos(angle_Z);
    ret(0) = cos(angle_radial) /** / 1.0 / **/ * sin(angle_Z);
    ret(1) = sin(angle_radial) /** / 1.0 / **/ * sin(angle_Z);

    return ret;
  }

  
  virtual double imagePlane_toCameraPlaneError(double value) const override
  {
    return value / focal();
  }

  // Transform a point from the camera plane to the image plane
  virtual Vec2 cam2ima(const Vec2& p) const override
  {
    return _radius * 2.0 * p + principal_point();
  }

  // Transform a point from the image plane to the camera plane
  virtual Vec2 ima2cam(const Vec2& p) const override
  {
    return (p -  principal_point()) / (2.0 * _radius);
  }

  /**
   * @brief Return true if this ray should be visible in the image
   * @return true if this ray is visible theorically
   */
  virtual bool isVisibleRay(const Vec3 & ray) const override {
    
    Vec2 proj = project(geometry::Pose3(), ray, true);

    Vec2 centered = proj - Vec2(_center_x, _center_y);
    if (centered.norm() > _radius) {
      return false;
    }

    return true;
  }

  double getRadius() const {
    return _radius;
  }

  void setRadius(double radius) {
    _radius = radius;
  }

  double getCenterX() const {
    return _center_x;
  }

  void setCenterX(double x) {
    _center_x = x;
  }

  double getCenterY() const {
    return _center_x;
  }

  void setCenterY(double y) {
    _center_y = y;
  }


protected:
  double _radius;
  double _center_x;
  double _center_y;
};

} // namespace camera
} // namespace aliceVision
