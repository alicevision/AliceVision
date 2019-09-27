// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/numeric/numeric.hpp>
#include <aliceVision/camera/cameraCommon.hpp>
#include <aliceVision/camera/IntrinsicBase.hpp>
#include <aliceVision/geometry/Pose3.hpp>
#include <aliceVision/multiview/projection.hpp>

#include <vector>
#include <sstream>


namespace aliceVision {
namespace camera {

/// Define an equidistant camera (store a K 3x3 matrix)
///  with intrinsic parameters defining the K calibration matrix
class Equidistant : public IntrinsicBase
{
  public:

  Equidistant() = default;

  Equidistant(
    unsigned int w, unsigned int h,
    const Mat3 K)
    :IntrinsicBase(w,h)
  {
    _K = K;
    _Kinv = _K.inverse();
  }

  Equidistant(
    unsigned int w, unsigned int h,
    double focal_length_pix,
    double ppx, double ppy, const std::vector<double>& distortionParams = {})
    : IntrinsicBase(w,h)
    , _distortionParams(distortionParams)
  {
    setK(focal_length_pix, ppx, ppy);
  }

  virtual ~Equidistant() {}

  virtual Equidistant* clone() const override { return new Equidistant(*this); }
  virtual void assign(const IntrinsicBase& other) override { *this = dynamic_cast<const Equidistant&>(other); }
  
  virtual bool isValid() const override { return focal() > 0 && IntrinsicBase::isValid(); }
  
  virtual EINTRINSIC getType() const override { return EQUIDISTANT_CAMERA; }
  std::string getTypeStr() const { return EINTRINSIC_enumToString(getType()); }

  double getFocalLengthPix() const { return _K(0,0); }

  Vec2 getPrincipalPoint() const { return Vec2(_K(0,2), _K(1,2)); }

  const Mat3& K() const { return _K; }
  const Mat3& Kinv() const { return _Kinv; }
  void setK(double focal_length_pix, double ppx, double ppy)
  {
    _K << focal_length_pix, 0., ppx, 0., focal_length_pix, ppy, 0., 0., 1.;
    _Kinv = _K.inverse();
  }
  void setK(const Mat3 &K) { _K = K;}
  /// Return the value of the focal in pixels
  inline double focal() const {return _K(0,0);}
  inline Vec2 principal_point() const {return Vec2(_K(0,2), _K(1,2));}

  // Get bearing vector of p point (image coord)
  Vec3 operator () (const Vec2& p) const override
  {
    Vec3 p3(p(0),p(1),1.0);
    return (_Kinv * p3).normalized();
  }

  // Transform a point from the camera plane to the image plane
  Vec2 cam2ima(const Vec2& p) const override
  {
    return focal() * p + principal_point();
  }

  // Transform a point from the image plane to the camera plane
  Vec2 ima2cam(const Vec2& p) const override
  {
    return ( p -  principal_point() ) / focal();
  }

  virtual bool have_disto() const override {  return false; }

  virtual Vec2 add_disto(const Vec2& p) const override { return p; }

  virtual Vec2 remove_disto(const Vec2& p) const override { return p; }

  virtual double imagePlane_toCameraPlaneError(double value) const override
  {
    return value / focal();
  }

  virtual Mat34 get_projective_equivalent(const geometry::Pose3 & pose) const override
  {
    Mat34 P;
    P_From_KRt(K(), pose.rotation(), pose.translation(), &P);
    return P;
  }

  // Data wrapper for non linear optimization (get data)
  std::vector<double> getParams() const override
  {
    std::vector<double> params = {_K(0,0), _K(0,2), _K(1,2)};
    params.insert(params.end(), _distortionParams.begin(), _distortionParams.end());
    return params;
  }

  bool hasDistortion() const override
  {
    for(double d: _distortionParams)
      if(d != 0.0)
        return true;
    return false;
  }

  const std::vector<double>& getDistortionParams() const
  {
    return _distortionParams;
  }

  void setDistortionParams(const std::vector<double>& distortionParams)
  {
    if(distortionParams.size() != _distortionParams.size())
    {
        std::stringstream s;
        s << "Pinhole::setDistortionParams: wrong number of distortion parameters (expected: " << _distortionParams.size() << ", given:" << distortionParams.size() << ").";
        throw std::runtime_error(s.str());
    }
    _distortionParams = distortionParams;
  }

  // Data wrapper for non linear optimization (update from data)
  bool updateFromParams(const std::vector<double>& params) override
  {
    if (params.size() != (3 + _distortionParams.size()))
      return false;

    this->setK(params[0], params[1], params[2]);
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

    return true;
  }

  /// Return the un-distorted pixel (with removed distortion)
  virtual Vec2 get_ud_pixel(const Vec2& p) const override {return p;}

  /// Return the distorted pixel (with added distortion)
  virtual Vec2 get_d_pixel(const Vec2& p) const override {return p;}

private:
  // Focal & principal point are embed into the calibration matrix K
  Mat3 _K, _Kinv;
protected:
  std::vector<double> _distortionParams;
};

} // namespace camera
} // namespace aliceVision
