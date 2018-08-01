// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/numeric/numeric.hpp>
#include <aliceVision/camera/cameraCommon.hpp>
#include <aliceVision/camera/Pinhole.hpp>

#include <vector>

namespace aliceVision {
namespace camera {

namespace radial_distortion{

  /// Solve by bisection the p' radius such that Square(disto(radius(p'))) = r^2
  template <class Disto_Functor>
  double bisection_Radius_Solve(
    const std::vector<double> & params, // radial distortion parameters
    double r2, // targeted radius
    Disto_Functor & functor,
    double epsilon = 1e-8 // criteria to stop the bisection
  )
  {
    // Guess plausible upper and lower bound
    double lowerbound = r2, upbound = r2;
    while (functor(params, lowerbound) > r2) lowerbound /= 1.05;
    while (functor(params, upbound) < r2) upbound *= 1.05;

    // Perform a bisection until epsilon accuracy is not reached
    while (epsilon < upbound - lowerbound)
    {
      const double mid = .5*(lowerbound + upbound);
      if (functor(params, mid) > r2)
        upbound = mid;
      else
        lowerbound = mid;
    }
    return .5*(lowerbound+upbound);
  }

} // namespace radial_distortion

/// Implement a Pinhole camera with a 1 radial distortion coefficient.
/// x_d = x_u (1 + K_1 r^2)
class PinholeRadialK1 : public Pinhole
{
  public:

  PinholeRadialK1(
    int w = 0, int h = 0,
    double focal = 0.0, double ppx = 0, double ppy = 0,
    double k1 = 0.0)
      :Pinhole(w, h, focal, ppx, ppy, {k1})
  {
  }

  PinholeRadialK1* clone() const { return new PinholeRadialK1(*this); }
  void assign(const IntrinsicBase& other) { *this = dynamic_cast<const PinholeRadialK1&>(other); }

  EINTRINSIC getType() const { return PINHOLE_CAMERA_RADIAL1; }

  virtual bool have_disto() const {  return true; }

  /// Add distortion to the point p (assume p is in the camera frame [normalized coordinates])
  virtual Vec2 add_disto(const Vec2 & p) const {

    const double k1 = _distortionParams.at(0);

    const double r2 = p(0)*p(0) + p(1)*p(1);
    const double r_coeff = (1. + k1*r2);

    return (p * r_coeff);
  }

  /// Remove distortion (return p' such that disto(p') = p)
  virtual Vec2 remove_disto(const Vec2& p) const {
    // Compute the radius from which the point p comes from thanks to a bisection
    // Minimize disto(radius(p')^2) == actual Squared(radius(p))

    const double r2 = p(0)*p(0) + p(1)*p(1);
    const double radius = (r2 == 0) ?
      1. :
      ::sqrt(radial_distortion::bisection_Radius_Solve(_distortionParams, r2, distoFunctor) / r2);
    return radius * p;
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

  private:

  /// Functor to solve Square(disto(radius(p'))) = r^2
  static double distoFunctor(const std::vector<double> & params, double r2)
  {
    const double k1 = params[0];
    return r2 * Square(1.+r2*k1);
  }
};

/// Implement a Pinhole camera with a 3 radial distortion coefficients.
/// x_d = x_u (1 + K_1 r^2 + K_2 r^4 + K_3 r^6)
class PinholeRadialK3 : public Pinhole
{
  public:

  PinholeRadialK3(
    int w = 0, int h = 0,
    double focal = 0.0, double ppx = 0, double ppy = 0,
    double k1 = 0.0, double k2 = 0.0, double k3 = 0.0)
      :Pinhole(w, h, focal, ppx, ppy, {k1, k2, k3})
  {
  }

  PinholeRadialK3* clone() const { return new PinholeRadialK3(*this); }
  void assign(const IntrinsicBase& other) { *this = dynamic_cast<const PinholeRadialK3&>(other); }

  EINTRINSIC getType() const { return PINHOLE_CAMERA_RADIAL3; }

  virtual bool have_disto() const {  return true; }

  /// Add distortion to the point p (assume p is in the camera frame [normalized coordinates])
  virtual Vec2 add_disto(const Vec2 & p) const
  {
    const double k1 = _distortionParams[0], k2 = _distortionParams[1], k3 = _distortionParams[2];

    const double r2 = p(0)*p(0) + p(1)*p(1);
    const double r4 = r2 * r2;
    const double r6 = r4 * r2;
    const double r_coeff = (1. + k1*r2 + k2*r4 + k3*r6);

    return (p * r_coeff);
  }

  /// Remove distortion (return p' such that disto(p') = p)
  virtual Vec2 remove_disto(const Vec2& p) const {
    // Compute the radius from which the point p comes from thanks to a bisection
    // Minimize disto(radius(p')^2) == actual Squared(radius(p))

    const double r2 = p(0)*p(0) + p(1)*p(1);
    const double radius = (r2 == 0) ? //1. : ::sqrt(bisectionSolve(_distortionParams, r2) / r2);
      1. :
      ::sqrt(radial_distortion::bisection_Radius_Solve(_distortionParams, r2, distoFunctor) / r2);
    return radius * p;
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

  private:

  /// Functor to solve Square(disto(radius(p'))) = r^2
  static double distoFunctor(const std::vector<double> & params, double r2)
  {
    const double k1 = params[0], k2 = params[1], k3 = params[2];
    return r2 * Square(1.+r2*(k1+r2*(k2+r2*k3)));
  }
};

} // namespace camera
} // namespace aliceVision
