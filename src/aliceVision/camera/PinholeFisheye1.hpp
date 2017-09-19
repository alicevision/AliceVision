// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#pragma once

#include "aliceVision/numeric/numeric.hpp"
#include "aliceVision/camera/cameraCommon.hpp"

#include <vector>

namespace aliceVision {
namespace camera {

/**
 * Implement a simple Fish-eye camera model with only one parameter
 * 
 * Fredreric Devernay and Olivier Faugeras. 2001. Straight lines have to be 
 * straight: automatic calibration and removal of distortion from scenes of 
 * structured environments. Mach. Vision Appl. 13, 1 (August 2001), 14-24. 
 * DOI: 10.1007/PL00013269 https://hal.inria.fr/inria-00267247/document
 */
class PinholeFisheye1 : public Pinhole
{
protected:
  // center of distortion is applied by the Intrinsics class
  std::vector<double> _distortionParams; // K1

public:

  PinholeFisheye1(
    int w = 0, int h = 0,
    double focal = 0.0, double ppx = 0, double ppy = 0,
    double k1 = 0.0)
        :Pinhole(w, h, focal, ppx, ppy)
  {
    _distortionParams = {k1};
  }

  PinholeFisheye1* clone() const { return new PinholeFisheye1(*this); }
  void assign(const IntrinsicBase& other) { *this = dynamic_cast<const PinholeFisheye1&>(other); }

  EINTRINSIC getType() const { return PINHOLE_CAMERA_FISHEYE1; }

  virtual bool have_disto() const { return true;}

  virtual Vec2 add_disto(const Vec2 & p) const
  {
    const double k1 = _distortionParams[0];
    const double r = std::hypot(p(0), p(1));
    const double coef = (std::atan(2.0 * r * std::tan(0.5 * k1)) / k1) / r;
    return  p * coef;
  }

  virtual Vec2 remove_disto(const Vec2 & p) const
  {
    const double k1 = _distortionParams[0];
    const double r = std::hypot(p(0), p(1));
    const double coef = 0.5 * std::tan(r * k1) / (std::tan(0.5 * k1) * r);
    return  p * coef;
  }

  // Data wrapper for non linear optimization (get data)
  virtual std::vector<double> getParams() const
  {
    std::vector<double> params = Pinhole::getParams();
    params.push_back(_distortionParams[0]);
    return params;
  }

  virtual std::vector<double> getDistortionParams() const
  {
    return _distortionParams;
  }

  // Data wrapper for non linear optimization (update from data)
  virtual bool updateFromParams(const std::vector<double> & params)
  {
    if (params.size() == 7)
    {
      this->setK(params[0], params[1], params[2]);
      _distortionParams = {params[3]};
      return true;
    }
    return false;
  }

  /// Return the un-distorted pixel (with removed distortion)
  virtual Vec2 get_ud_pixel(const Vec2& p) const
  {
    return cam2ima( remove_disto(ima2cam(p)) );
  }

  /// Return the distorted pixel (with added distortion)
  virtual Vec2 get_d_pixel(const Vec2& p) const
  {
    return cam2ima( add_disto(ima2cam(p)) );
  }

  // Serialization
  template <class Archive>
  void save( Archive & ar) const
  {
    Pinhole::save(ar);
    ar(cereal::make_nvp("fisheye1", _distortionParams));
  }

  // Serialization
  template <class Archive>
  void load( Archive & ar)
  {
    Pinhole::load(ar);
    ar(cereal::make_nvp("fisheye1", _distortionParams));
  }
};

} // namespace camera
} // namespace aliceVision

#include <cereal/types/polymorphic.hpp>
#include <cereal/types/vector.hpp>

CEREAL_REGISTER_TYPE_WITH_NAME(aliceVision::camera::PinholeFisheye1, "fisheye1");
