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

    /**
    * @brief Solve by bisection the p' radius such that Square(disto(radius(p'))) = r^2
    * @param[in] params the distortion parameters.
    * @param[in] r2 targeted radius
    * @param[in] functor functor to solve Square(disto(radius(p'))) = r^2
    * @param epsilon minimum error to stop the bisection
    * @return optimal radius
    */
  template <class Disto_Functor>
  double bisection_Radius_Solve(const std::vector<double> & params, // radial distortion parameters
                                double r2,
                                Disto_Functor & functor,
                                double epsilon = 1e-8)
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

/**
 * @class DistortionRadialK1
 * @brief Radial distortion with a single distortion coefficient.
 * \f$ x_d = x_u (1 + K_1 r^2 ) \f$
 */
class DistortionRadialK1 : public Distortion {
public:

  /**
   * @brief Default contructor, no distortion
   */
  DistortionRadialK1() {
    _distortionParams = {0.0};
  }

  /**
   * @brief Constructor with the single distortion coefficient.
   * @param[in] k1 the distortion coefficient
   */
  explicit DistortionRadialK1(double k1) {
    _distortionParams = {k1};
  }

  DistortionRadialK1* clone() const override { return new DistortionRadialK1(*this); }

  /// Add distortion to the point p (assume p is in the camera frame [normalized coordinates])
  Vec2 addDistortion(const Vec2 & p) const override
  {
    const double k1 = _distortionParams.at(0);

    const double r2 = p(0)*p(0) + p(1)*p(1);
    const double r_coeff = (1. + k1*r2);

    return (p * r_coeff);
  }

  Eigen::Matrix2d getDerivativeAddDistoWrtPt(const Vec2 & p) const override
  {

    const double k1 = _distortionParams[0];

    const double r = sqrt(p(0)*p(0) + p(1)*p(1));
    const double eps = 1e-8;
    if (r < eps) {
      return Eigen::Matrix2d::Identity();
    }

    Eigen::Matrix<double, 1, 2> d_r_d_p;
    d_r_d_p(0) = p(0) / r;
    d_r_d_p(1) = p(1) / r;

    
    
    const double r2 = r * r;
    const double r_coeff = 1.0 + k1 * r2;

    const double d_r_coeff_d_r = 2.0 * k1 * r;
    const Eigen::Matrix<double, 1, 2> d_r_coeff_d_p = d_r_coeff_d_r * d_r_d_p;

    return Eigen::Matrix2d::Identity() * r_coeff + p * d_r_coeff_d_p;
  }

  Eigen::MatrixXd getDerivativeAddDistoWrtDisto(const Vec2 & p) const  override
  {
    
    const double k1 = _distortionParams[0];

    const double r = sqrt(p(0)*p(0) + p(1)*p(1));   
    const double eps = 1e-8;
    if (r < eps)
    {
      return Eigen::Matrix<double, 2, 1>::Zero();
    }

    const Eigen::MatrixXd ret = p * r * r;

    return ret;
  }

  /// Remove distortion (return p' such that disto(p') = p)
  Vec2 removeDistortion(const Vec2& p) const override
  {
    // Compute the radius from which the point p comes from thanks to a bisection
    // Minimize disto(radius(p')^2) == actual Squared(radius(p))

    const double r2 = p(0)*p(0) + p(1)*p(1);
    const double radius = (r2 == 0) ? 1. : ::sqrt(radial_distortion::bisection_Radius_Solve(_distortionParams, r2, distoFunctor) / r2);
    return radius * p;
  }

  Eigen::Matrix2d getDerivativeRemoveDistoWrtPt(const Vec2 & p) const override
  {

    const double r = sqrt(p(0)*p(0) + p(1)*p(1));
    const double eps = 1e-8;
    if (r < eps)
    {
      return Eigen::Matrix2d::Identity();
    }

    const Vec2 undist = removeDistortion(p);

    const Eigen::Matrix2d Jinv = getDerivativeAddDistoWrtPt(undist);

    return Jinv.inverse();
  }

  Eigen::MatrixXd getDerivativeRemoveDistoWrtDisto(const Vec2 & p) const override {
    
    double r_dist = sqrt(p(0)*p(0) + p(1)*p(1));
    const double eps = 1e-8;
    if (r_dist < eps)
    {
      return Eigen::Matrix<double, 2, 1>::Zero();
    }

    const Vec2 p_undist = removeDistortion(p);

    const double k1 = _distortionParams[0];

    const double r = sqrt(p_undist(0)*p_undist(0) + p_undist(1)*p_undist(1));
    const double r2 = r * r;
    
    const double r_coeff = 1.0 + k1 * r2;

    Eigen::Matrix<double, 1, 1> d_rcoeff_d_params;
    d_rcoeff_d_params(0, 0) = r2;
    
    Eigen::Matrix<double, 2, 1> ret;
    ret(0, 0) = - (p(0) * d_rcoeff_d_params(0, 0)) / (r_coeff * r_coeff);
    ret(1, 0) = - (p(1) * d_rcoeff_d_params(0, 0)) / (r_coeff * r_coeff);

    return ret;
  }

  double getUndistortedRadius(double r) const override
  {
    return std::sqrt(radial_distortion::bisection_Radius_Solve(_distortionParams, r * r, distoFunctor));
  }

  /// Functor to solve Square(disto(radius(p'))) = r^2
  static double distoFunctor(const std::vector<double> & params, double r2)
  {
    const double k1 = params[0];
    return r2 * Square(1.+r2*k1);
  }

  ~DistortionRadialK1() override = default;
};


/**
 * @class DistortionRadialK3
 * @brief Radial distortion modeled with a 6th degree polynomial with three distortion coefficients.
 * \f$ x_d = x_u (1 + K_1 r^2 + K_2 r^4 + K_3 r^6) \f$
 */
class DistortionRadialK3 : public Distortion {
public:
  /**
   * @brief Default constructor, no distortion.
   */
  DistortionRadialK3()
  {
    _distortionParams = {0.0, 0.0, 0.0};
  }

  /**
   * @brief Constructor with the three coefficients
   * @param[in] k1 first coefficient
   * @param[in] k2 second coefficient
   * @param[in] k3 third coefficient
   */
  explicit DistortionRadialK3(double k1, double k2, double k3)
  {
    _distortionParams = {k1, k2, k3};
  }

  DistortionRadialK3* clone() const override { return new DistortionRadialK3(*this); }

  /// Add distortion to the point p (assume p is in the camera frame [normalized coordinates])
  Vec2 addDistortion(const Vec2 & p) const override
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

  Eigen::Matrix2d getDerivativeAddDistoWrtPt(const Vec2 & p) const override
  {

    const double k1 = _distortionParams[0];
    const double k2 = _distortionParams[1];
    const double k3 = _distortionParams[2];

    const double r = sqrt(p(0)*p(0) + p(1)*p(1));
    const double eps = 1e-8;
    if (r < eps)
    {
      return Eigen::Matrix2d::Identity();
    }

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

  Eigen::MatrixXd getDerivativeAddDistoWrtDisto(const Vec2 & p) const  override
  {
    const double r = sqrt(p(0)*p(0) + p(1)*p(1));
    const double eps = 1e-8;
    if (r < eps)
    {
      return Eigen::Matrix<double, 2, 3>::Zero();
    }

    const double r2 = r * r;
    const double r4 = r2 * r2;
    const double r6 = r4 * r2;
    
    /*const double r_coeff = 1.0 + k1 * r2 + k2 * r4 + k3 * r6;*/

    Eigen::Matrix<double, 1, 3> d_r_coeff_d_params;
    d_r_coeff_d_params(0, 0) = r2;
    d_r_coeff_d_params(0, 1) = r4;
    d_r_coeff_d_params(0, 2) = r6;

    Eigen::MatrixXd ret = p * d_r_coeff_d_params;

    return ret;
  }


  /// Remove distortion (return p' such that disto(p') = p)
  Vec2 removeDistortion(const Vec2& p) const override
  {
    // Compute the radius from which the point p comes from thanks to a bisection
    // Minimize disto(radius(p')^2) == actual Squared(radius(p))

    const double r2 = p(0)*p(0) + p(1)*p(1);
    const double radius = (r2 == 0) ? //1. : ::sqrt(bisectionSolve(_distortionParams, r2) / r2);
      1. :
      ::sqrt(radial_distortion::bisection_Radius_Solve(_distortionParams, r2, distoFunctor) / r2);
    return radius * p;
  }

  Eigen::Matrix2d getDerivativeRemoveDistoWrtPt(const Vec2 & p) const override
  {

    const double r = sqrt(p(0)*p(0) + p(1)*p(1));
    const double eps = 1e-8;
    if (r < eps) {
      return Eigen::Matrix2d::Identity();
    }

    const Vec2 undist = removeDistortion(p);

    const Eigen::Matrix2d Jinv = getDerivativeAddDistoWrtPt(undist);

    return Jinv.inverse();
  }

  Eigen::MatrixXd getDerivativeRemoveDistoWrtDisto(const Vec2 & p) const override
  {
    
    double r_dist = sqrt(p(0)*p(0) + p(1)*p(1));
    const double eps = 1e-8;
    if (r_dist < eps) {
      return Eigen::Matrix<double, 2, 3>::Zero();
    }

    Vec2 p_undist = removeDistortion(p);

    const double k1 = _distortionParams[0];
    const double k2 = _distortionParams[1];
    const double k3 = _distortionParams[2];

    const double r = sqrt(p_undist(0) * p_undist(0) + p_undist(1) * p_undist(1));
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

  double getUndistortedRadius(double r) const override
  {
    return std::sqrt(radial_distortion::bisection_Radius_Solve(_distortionParams, r * r, distoFunctor));
  }

  /// Functor to solve Square(disto(radius(p'))) = r^2
  static double distoFunctor(const std::vector<double> & params, double r2)
  {
    const double k1 = params[0], k2 = params[1], k3 = params[2];
    return r2 * Square(1.+r2*(k1+r2*(k2+r2*k3)));
  }

  ~DistortionRadialK3() override = default;
};

class DistortionRadialK3PT : public Distortion {
public:
  DistortionRadialK3PT()
  {
    _distortionParams = {0.0, 0.0, 0.0};
  }

  explicit DistortionRadialK3PT(double k1, double k2, double k3)
  {
    _distortionParams = {k1, k2, k3};
  }

  DistortionRadialK3PT* clone() const override { return new DistortionRadialK3PT(*this); }

  /// Add distortion to the point p (assume p is in the camera frame [normalized coordinates])
  Vec2 addDistortion(const Vec2 & p) const override
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

  Eigen::Matrix2d getDerivativeAddDistoWrtPt(const Vec2 & p) const override
  {

    const double k1 = _distortionParams[0];
    const double k2 = _distortionParams[1];
    const double k3 = _distortionParams[2];

    const double r = sqrt(p(0)*p(0) + p(1)*p(1));
    if (r < 1e-12) {
      return Eigen::Matrix2d::Identity();
    }

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

  Eigen::MatrixXd getDerivativeAddDistoWrtDisto(const Vec2 & p) const  override
  {
    
    const double k1 = _distortionParams[0];
    const double k2 = _distortionParams[1];
    const double k3 = _distortionParams[2];

    const double r = sqrt(p(0)*p(0) + p(1)*p(1));
    const double eps = 1e-8;
    if (r < eps)
    {
      return Eigen::Matrix<double, 2, 3>::Zero();
    }

    const double r2 = r * r;
    const double r4 = r2 * r2;
    const double r6 = r4 * r2;
    
    const double denum = (1.0 + k1 + k2 + k3);
    const double num = (1.0 + k1*r2 + k2*r4 + k3*r6);
    /*const double r_coeff = num / denum;*/

    const double denum2 = denum * denum;
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

  Eigen::Matrix2d getDerivativeRemoveDistoWrtPt(const Vec2 & p) const override
  {

    double r_dist = sqrt(p(0)*p(0) + p(1)*p(1));
    const double eps = 1e-8;
    if (r_dist < eps)
    {
      return Eigen::Matrix2d::Identity();
    }

    Vec2 undist = removeDistortion(p);

    Eigen::Matrix2d Jinv = getDerivativeAddDistoWrtPt(undist);

    return Jinv.inverse();
  }

  Eigen::MatrixXd getDerivativeRemoveDistoWrtDisto(const Vec2 & p) const override
  {
    
    const double k1 = _distortionParams[0];
    const double k2 = _distortionParams[1];
    const double k3 = _distortionParams[2];

    double r_dist = sqrt(p(0)*p(0) + p(1)*p(1));
    const double eps = 1e-8;
    if (r_dist < eps) {
      return Eigen::Matrix<double, 2, 3>::Zero();
    }

    Vec2 p_undist = removeDistortion(p);
    double r = sqrt(p_undist(0) * p_undist(0) + p_undist(1) * p_undist(1));

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
  Vec2 removeDistortion(const Vec2& p) const override
  {
    // Compute the radius from which the point p comes from thanks to a bisection
    // Minimize disto(radius(p')^2) == actual Squared(radius(p))

    const double r2 = p(0)*p(0) + p(1)*p(1);
    const double radius = (r2 == 0) ? //1. : ::sqrt(bisectionSolve(_distortionParams, r2) / r2);
      1. :
      ::sqrt(radial_distortion::bisection_Radius_Solve(_distortionParams, r2, distoFunctor, 1e-12) / r2);

    const Vec2 p_undist = radius * p;
    return p_undist;
  }

  double getUndistortedRadius(double r) const override
  {
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

  ~DistortionRadialK3PT() override = default;
};

} // namespace camera
} // namespace aliceVision
