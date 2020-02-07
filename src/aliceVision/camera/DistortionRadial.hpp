// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/camera/Distortion.hpp>

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
    while (epsilon < (upbound - lowerbound))
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

class DistortionRadialK1 : public Distortion {
public:
  DistortionRadialK1() {
    _distortionParams = {0.0};
  }

  DistortionRadialK1(double p1) {
    _distortionParams = {p1};
  }

  /// Add distortion to the point p (assume p is in the camera frame [normalized coordinates])
  virtual Vec2 add_disto(const Vec2 & p) const override
  {
    const double k1 = _distortionParams.at(0);

    const double r2 = p(0)*p(0) + p(1)*p(1);
    const double r_coeff = (1. + k1*r2);

    return (p * r_coeff);
  }

  virtual Eigen::Matrix2d getDerivativeAddDistoWrtPt(const Vec2 & p) const override {

    const double k1 = _distortionParams[0];

    const double r = sqrt(p(0)*p(0) + p(1)*p(1));
    Eigen::Matrix<double, 1, 2> d_r_d_p;
    d_r_d_p(0) = p(0) / r;
    d_r_d_p(1) = p(1) / r;
    
    const double r2 = r * r;
    const double r_coeff = 1.0 + k1 * r2;

    double d_r_coeff_d_r = 2.0 * k1 * r;
    Eigen::Matrix<double, 1, 2> d_r_coeff_d_p = d_r_coeff_d_r * d_r_d_p;

    return Eigen::Matrix2d::Identity() * r_coeff + p * d_r_coeff_d_p;
  }

  virtual Eigen::MatrixXd getDerivativeAddDistoWrtDisto(const Vec2 & p) const  override {
    
    const double k1 = _distortionParams[0];

    const double r = sqrt(p(0)*p(0) + p(1)*p(1));    
    const double r2 = r * r;
    
    const double r_coeff = 1.0 + k1 * r2;

    Eigen::MatrixXd ret = p * 2.0 * k1 * r;

    return ret;
  }

  /// Remove distortion (return p' such that disto(p') = p)
  virtual Vec2 remove_disto(const Vec2& p) const override {
    // Compute the radius from which the point p comes from thanks to a bisection
    // Minimize disto(radius(p')^2) == actual Squared(radius(p))

    const double r2 = p(0)*p(0) + p(1)*p(1);
    const double radius = (r2 == 0) ? 1. : ::sqrt(radial_distortion::bisection_Radius_Solve(_distortionParams, r2, distoFunctor) / r2);
    return radius * p;
  }

  virtual Eigen::Matrix2d getDerivativeRemoveDistoWrtPt(const Vec2 & p) const override {

    Vec2 undist = remove_disto(p);

    Eigen::Matrix2d Jinv = getDerivativeAddDistoWrtPt(undist);

    return Jinv.inverse();
  }

  virtual Eigen::MatrixXd getDerivativeRemoveDistoWrtDisto(const Vec2 & p) const override {
    
    Vec2 p_undist = remove_disto(p);

    const double k1 = _distortionParams[0];

    const double r = sqrt(p(0)*p(0) + p(1)*p(1));
    const double r2 = r * r;
    
    const double r_coeff = 1.0 + k1 * r2;

    Eigen::Matrix<double, 1, 1> d_rcoeff_d_params;
    d_rcoeff_d_params(0, 0) = r2;
    
    Eigen::Matrix<double, 2, 1> ret;
    ret(0, 0) = - (p(0) * d_rcoeff_d_params(0, 0)) / (r_coeff * r_coeff);
    ret(1, 0) = - (p(1) * d_rcoeff_d_params(0, 0)) / (r_coeff * r_coeff);

    return ret;
  }

  virtual double getUndistortedRadius(double r) const override {
    return std::sqrt(radial_distortion::bisection_Radius_Solve(_distortionParams, r * r, distoFunctor));
  }

  /// Functor to solve Square(disto(radius(p'))) = r^2
  static double distoFunctor(const std::vector<double> & params, double r2)
  {
    const double k1 = params[0];
    return r2 * Square(1.+r2*k1);
  }
};

class DistortionRadialK3 : public Distortion {
public:
  DistortionRadialK3() {
    _distortionParams = {0.0, 0.0, 0.0};
  }

  DistortionRadialK3(double p1, double p2, double p3) {
    _distortionParams = {p1, p2, p3};
  }

  /// Add distortion to the point p (assume p is in the camera frame [normalized coordinates])
  virtual Vec2 add_disto(const Vec2 & p) const override
  {
    const double k1 = _distortionParams[0];
    const double k2 = _distortionParams[1];
    const double k3 = _distortionParams[2];

    const double r = sqrt(p(0)*p(0) + p(1)*p(1));
    
    const double r2 = r * r;
    const double r4 = r2 * r2;
    const double r6 = r4 * r2;
    const double r_coeff = (1. + k1 * r2 + k2 * r4 + k3 * r6);

    return (p * r_coeff);
  }

  virtual Eigen::Matrix2d getDerivativeAddDistoWrtPt(const Vec2 & p) const override {

    const double k1 = _distortionParams[0];
    const double k2 = _distortionParams[1];
    const double k3 = _distortionParams[2];

    const double r = sqrt(p(0)*p(0) + p(1)*p(1));
    Eigen::Matrix<double, 1, 2> d_r_d_p;
    d_r_d_p(0) = p(0) / r;
    d_r_d_p(1) = p(1) / r;
    
    const double r2 = r * r;
    const double r3 = r2 * r;
    const double r4 = r2 * r2;
    const double r5 = r4 * r;
    const double r6 = r4 * r2;
    
    const double r_coeff = 1.0 + k1 * r2 + k2 * r4 + k3 * r6;

    double d_r_coeff_d_r = 2.0 * k1 * r + 4.0 * k2 * r3 + 6.0 * k3 * r5;
    Eigen::Matrix<double, 1, 2> d_r_coeff_d_p = d_r_coeff_d_r * d_r_d_p;

    return Eigen::Matrix2d::Identity() * r_coeff + p * d_r_coeff_d_p;
  }

  virtual Eigen::MatrixXd getDerivativeAddDistoWrtDisto(const Vec2 & p) const  override {
    
    const double k1 = _distortionParams[0];
    const double k2 = _distortionParams[1];
    const double k3 = _distortionParams[2];

    const double r = sqrt(p(0)*p(0) + p(1)*p(1));
    const double r2 = r * r;
    const double r4 = r2 * r2;
    const double r6 = r4 * r2;
    

    const double r_coeff = 1.0 + k1 * r2 + k2 * r4 + k3 * r6;

    Eigen::Matrix<double, 1, 3> d_r_coeff_d_params;
    d_r_coeff_d_params(0, 0) = r2;
    d_r_coeff_d_params(0, 1) = r4;
    d_r_coeff_d_params(0, 2) = r6;

    Eigen::MatrixXd ret = p * d_r_coeff_d_params;

    return ret;
  }


  /// Remove distortion (return p' such that disto(p') = p)
  virtual Vec2 remove_disto(const Vec2& p) const override {
    // Compute the radius from which the point p comes from thanks to a bisection
    // Minimize disto(radius(p')^2) == actual Squared(radius(p))

    const double r2 = p(0)*p(0) + p(1)*p(1);
    const double radius = (r2 == 0) ? //1. : ::sqrt(bisectionSolve(_distortionParams, r2) / r2);
      1. :
      ::sqrt(radial_distortion::bisection_Radius_Solve(_distortionParams, r2, distoFunctor) / r2);
    return radius * p;
  }

  virtual Eigen::Matrix2d getDerivativeRemoveDistoWrtPt(const Vec2 & p) const override {

    Vec2 undist = remove_disto(p);

    Eigen::Matrix2d Jinv = getDerivativeAddDistoWrtPt(undist);

    return Jinv.inverse();
  }

  virtual Eigen::MatrixXd getDerivativeRemoveDistoWrtDisto(const Vec2 & p) const override {
    
    Vec2 p_undist = remove_disto(p);

    const double k1 = _distortionParams[0];
    const double k2 = _distortionParams[1];
    const double k3 = _distortionParams[2];

    const double r = sqrt(p(0)*p(0) + p(1)*p(1));
    const double r2 = r * r;
    const double r4 = r2 * r2;
    const double r6 = r4 * r2;
    
    const double r_coeff = 1.0 + k1 * r2 + k2 * r4 + k3 * r6;

    Eigen::Matrix<double, 1, 3> d_rcoeff_d_params;
    d_rcoeff_d_params(0, 0) = r2;
    d_rcoeff_d_params(0, 1) = r4;
    d_rcoeff_d_params(0, 2) = r6;
    
    Eigen::Matrix<double, 2, 3> ret;
    ret(0, 0) = - (p(0) * d_rcoeff_d_params(0, 0)) / (r_coeff * r_coeff);
    ret(0, 1) = - (p(0) * d_rcoeff_d_params(0, 1)) / (r_coeff * r_coeff);
    ret(0, 2) = - (p(0) * d_rcoeff_d_params(0, 2)) / (r_coeff * r_coeff);
    ret(1, 0) = - (p(1) * d_rcoeff_d_params(0, 0)) / (r_coeff * r_coeff);
    ret(1, 1) = - (p(1) * d_rcoeff_d_params(0, 1)) / (r_coeff * r_coeff);
    ret(1, 2) = - (p(1) * d_rcoeff_d_params(0, 2)) / (r_coeff * r_coeff);

    return ret;
  }

  virtual double getUndistortedRadius(double r) const override {
    return std::sqrt(radial_distortion::bisection_Radius_Solve(_distortionParams, r * r, distoFunctor));
  }

  /// Functor to solve Square(disto(radius(p'))) = r^2
  static double distoFunctor(const std::vector<double> & params, double r2)
  {
    const double k1 = params[0], k2 = params[1], k3 = params[2];
    return r2 * Square(1.+r2*(k1+r2*(k2+r2*k3)));
  }
};

class DistortionRadialK3PT : public Distortion {
public:
  DistortionRadialK3PT() {
    _distortionParams = {0.0, 0.0, 0.0};
  }

  DistortionRadialK3PT(double p1, double p2, double p3) {
    _distortionParams = {p1, p2, p3};
  }

  /// Add distortion to the point p (assume p is in the camera frame [normalized coordinates])
  virtual Vec2 add_disto(const Vec2 & p) const override
  {
    const double k1 = _distortionParams[0];
    const double k2 = _distortionParams[1];
    const double k3 = _distortionParams[2];

    const double r = sqrt(p(0)*p(0) + p(1)*p(1));
    
    const double r2 = r * r;
    const double r4 = r2 * r2;
    const double r6 = r4 * r2;
    
    const double r_coeff = (1.0 + k1*r2 + k2*r4 + k3*r6) / (1.0 + k1 + k2 + k3);

    return (p * r_coeff);
  }

  virtual Eigen::Matrix2d getDerivativeAddDistoWrtPt(const Vec2 & p) const override {

    const double k1 = _distortionParams[0];
    const double k2 = _distortionParams[1];
    const double k3 = _distortionParams[2];

    const double r = sqrt(p(0)*p(0) + p(1)*p(1));
    Eigen::Matrix<double, 1, 2> d_r_d_p;
    d_r_d_p(0) = p(0) / r;
    d_r_d_p(1) = p(1) / r;
    
    const double r2 = r * r;
    const double r3 = r2 * r;
    const double r4 = r2 * r2;
    const double r5 = r4 * r;
    const double r6 = r4 * r2;
    
    const double r_coeff = (1.0 + k1*r2 + k2*r4 + k3*r6) / (1.0 + k1 + k2 + k3);

    double d_r_coeff_d_r = (2.0 * k1 * r + 4.0 * k2 * r3 + 6.0 * k3 * r5) / (1.0 + k1 + k2 + k3);
    Eigen::Matrix<double, 1, 2> d_r_coeff_d_p = d_r_coeff_d_r * d_r_d_p;

    return Eigen::Matrix2d::Identity() * r_coeff + p * d_r_coeff_d_p;
  }

  virtual Eigen::MatrixXd getDerivativeAddDistoWrtDisto(const Vec2 & p) const  override {
    
    const double k1 = _distortionParams[0];
    const double k2 = _distortionParams[1];
    const double k3 = _distortionParams[2];

    const double r = sqrt(p(0)*p(0) + p(1)*p(1));
    const double r2 = r * r;
    const double r4 = r2 * r2;
    const double r6 = r4 * r2;
    
    const double denum = (1.0 + k1 + k2 + k3);
    const double num = (1.0 + k1*r2 + k2*r4 + k3*r6);
    const double r_coeff = num / denum;

    double denum2 = denum * denum;
    Eigen::Matrix<double, 1, 3> d_num_d_params;
    d_num_d_params(0, 0) = r2;
    d_num_d_params(0, 1) = r4;
    d_num_d_params(0, 2) = r6;

    Eigen::Matrix<double, 1, 3> d_denum_d_params;
    d_denum_d_params(0, 0) = 1;
    d_denum_d_params(0, 1) = 1;
    d_denum_d_params(0, 2) = 1;

    Eigen::Matrix<double, 1, 3> d_rcoeff_d_params = (denum * d_num_d_params - num * d_denum_d_params) / denum2;
    Eigen::MatrixXd ret = p * d_rcoeff_d_params;

    return ret;
  }

  virtual Eigen::Matrix2d getDerivativeRemoveDistoWrtPt(const Vec2 & p) const override {

    Vec2 undist = remove_disto(p);

    Eigen::Matrix2d Jinv = getDerivativeAddDistoWrtPt(undist);

    return Jinv.inverse();
  }

  virtual Eigen::MatrixXd getDerivativeRemoveDistoWrtDisto(const Vec2 & p) const override {
    
    Vec2 p_undist = remove_disto(p);

    const double k1 = _distortionParams[0];
    const double k2 = _distortionParams[1];
    const double k3 = _distortionParams[2];

    const double r = sqrt(p(0)*p(0) + p(1)*p(1));
    const double r2 = r * r;
    const double r4 = r2 * r2;
    const double r6 = r4 * r2;
    
    const double denum = (1.0 + k1 + k2 + k3);
    const double num = (1.0 + k1*r2 + k2*r4 + k3*r6);
    const double r_coeff = num / denum;

    double denum2 = denum * denum;
    Eigen::Matrix<double, 1, 3> d_num_d_params;
    d_num_d_params(0, 0) = r2;
    d_num_d_params(0, 1) = r4;
    d_num_d_params(0, 2) = r6;

    Eigen::Matrix<double, 1, 3> d_denum_d_params;
    d_denum_d_params(0, 0) = 1;
    d_denum_d_params(0, 1) = 1;
    d_denum_d_params(0, 2) = 1;

    Eigen::Matrix<double, 1, 3> d_rcoeff_d_params = (denum * d_num_d_params - num * d_denum_d_params) / denum2;
    
    Eigen::Matrix<double, 2, 3> ret;
    ret(0, 0) = - (p(0) * d_rcoeff_d_params(0, 0)) / (r_coeff * r_coeff);
    ret(0, 1) = - (p(0) * d_rcoeff_d_params(0, 1)) / (r_coeff * r_coeff);
    ret(0, 2) = - (p(0) * d_rcoeff_d_params(0, 2)) / (r_coeff * r_coeff);
    ret(1, 0) = - (p(1) * d_rcoeff_d_params(0, 0)) / (r_coeff * r_coeff);
    ret(1, 1) = - (p(1) * d_rcoeff_d_params(0, 1)) / (r_coeff * r_coeff);
    ret(1, 2) = - (p(1) * d_rcoeff_d_params(0, 2)) / (r_coeff * r_coeff);

    

    return ret;
  }

  /// Remove distortion (return p' such that disto(p') = p)
  virtual Vec2 remove_disto(const Vec2& p) const override {
    // Compute the radius from which the point p comes from thanks to a bisection
    // Minimize disto(radius(p')^2) == actual Squared(radius(p))

    const double r2 = p(0)*p(0) + p(1)*p(1);
    const double radius = (r2 == 0) ? //1. : ::sqrt(bisectionSolve(_distortionParams, r2) / r2);
      1. :
      ::sqrt(radial_distortion::bisection_Radius_Solve(_distortionParams, r2, distoFunctor, 1e-12) / r2);

    Vec2 p_undist = radius * p;
    return p_undist;
  }

  virtual double getUndistortedRadius(double r) const override {
    return std::sqrt(radial_distortion::bisection_Radius_Solve(_distortionParams, r * r, distoFunctor));
  }

  /// Functor to solve Square(disto(radius(p'))) = r^2
  static double distoFunctor(const std::vector<double> & params, double r2)
  {
    const double k1 = params[0];
    const double k2 = params[1];
    const double k3 = params[2];

    const double r4 = r2 * r2;
    const double r6 = r4 * r2;
    
    const double r_coeff = (1.0 + k1*r2 + k2*r4 + k3*r6) / (1.0 + k1 + k2 + k3);

    return r2 * Square(r_coeff);
  }
};

} // namespace camera
} // namespace aliceVision
