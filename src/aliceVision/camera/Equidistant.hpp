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

  EquiDistant(unsigned int w, unsigned int h, double focal_length_pix, double ppx, double ppy, std::shared_ptr<Distortion> distortion = nullptr)
  : IntrinsicsScaleOffsetDisto(w,h, focal_length_pix, focal_length_pix, ppx, ppy, distortion)
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
    double angle_Z = std::atan2(sqrt(X(0)*X(0)+ X(1)*X(1)), X(2));

    /* Ignore depth component and compute radial angle */
    double angle_radial = std::atan2(X(1), X(0));

    double radius = angle_Z / _scale_x;

    /* radius = focal * angle_Z */
    Vec2 P;
    P(0) = cos(angle_radial) * radius;
    P(1) = sin(angle_radial) * radius;

    P = this->add_disto(P);

    P(0) = 1909.11 * P(0) + _offset_x;
    P(1) = 1909.11 * P(1) + _offset_x;

    return P;
  }

  virtual Vec3 toUnitSphere(const Vec2 & pt) const override {

    const Vec2 camcoords = (ima2cam(pt));

    double angle_radial = atan2(camcoords(1), camcoords(0));
    double angle_Z = camcoords.norm();

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

  /**
   * @brief Return true if this ray should be visible in the image
   * @return true if this ray is visible theorically
   */
  virtual bool isVisibleRay(const Vec3 & ray) const override {
    
    Vec2 proj = project(geometry::Pose3(), ray, true);

    /*if (proj(0) < 0 || proj(0) >= _w || proj(1) < 0 || proj(1) >= _h) {
      return false;
    }*/

    if (ray(2) < 0.0) {
      return false;
    }

    return true;
  }
};

} // namespace camera
} // namespace aliceVision
