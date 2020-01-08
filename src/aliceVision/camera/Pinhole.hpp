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

  std::string getTypeStr() const { 
    return EINTRINSIC_enumToString(getType()); 
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

  // Get bearing vector of p point (image coord)
  Vec3 operator () (const Vec2& p) const override
  {
    double x = (p(0) - _offset_x) / _scale_x;
    double y = (p(1) - _offset_y) / _scale_y;

    Vec3 p3(x, y, 1.0);

    return p3.normalized();
  }

  virtual Vec2 project(const geometry::Pose3& pose, const Vec3& pt3D, bool applyDistortion = true) const override
  {
    const Vec3 X = pose(pt3D); // apply pose
    if (applyDistortion && this->have_disto()) {
      return this->cam2ima( this->add_disto(X.head<2>()/X(2)));
    }
    else {
      return this->cam2ima( X.head<2>()/X(2));
    }
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

  // Data wrapper for non linear optimization (get data)
  std::vector<double> getParams() const override
  {
    std::vector<double> params = {_scale_x, _offset_x, _offset_y};

    if (have_disto()) {

      params.insert(params.end(), _pDistortion->getParameters().begin(), _pDistortion->getParameters().end());
    }
    
    return params;
  }

  // Data wrapper for non linear optimization (update from data)
  bool updateFromParams(const std::vector<double>& params) override
  {
    if (_pDistortion == nullptr) {
      if (params.size() != 3) {
        return false;
      }
    }

    if (params.size() != (3 + _pDistortion->getDistortionParametersCount())) {
      return false;
    }

    _scale_x = params[0];
    _scale_y = params[0];
    _offset_x = params[1];
    _offset_y = params[2];


    setDistortionParams({params.begin() + 3, params.end()});

    return true;
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
