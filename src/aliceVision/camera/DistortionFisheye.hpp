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

class DistortionFisheye : public Distortion
{
public:
  DistortionFisheye()
  {
    _distortionParams = {0.0, 0.0, 0.0, 0.0};
  }

  DistortionFisheye(double p1, double p2, double p3, double p4)
  {
    _distortionParams = {p1, p2, p3, p4};
  }

  DistortionFisheye* clone() const override
  {
      return new DistortionFisheye(*this);
  }

  /// Add distortion to the point p (assume p is in the camera frame [normalized coordinates])
  Vec2 addDistortion(const Vec2 & p) const override
  {
    const double eps = 1e-8;
    const double r = std::hypot(p(0), p(1));
    if (r < eps)
    {
      return p;
    }

    const double k1 = _distortionParams.at(0);
    const double k2 = _distortionParams.at(1);
    const double k3 = _distortionParams.at(2);
    const double k4 = _distortionParams.at(3);    

    const double theta = std::atan(r);
    const double theta2 = theta*theta;
    const double theta3 = theta2*theta;
    const double theta4 = theta2*theta2;
    const double theta5 = theta4*theta;
    const double theta6 = theta3*theta3;
    const double theta7 = theta6*theta;
    const double theta8 = theta4*theta4;
    const double theta9 = theta8*theta;
    const double theta_dist = theta + k1*theta3 + k2*theta5 + k3*theta7 + k4*theta9;
    const double cdist = theta_dist / r;

    return p * cdist;
  }

   Eigen::Matrix2d getDerivativeAddDistoWrtPt(const Vec2 & p) const override
  {
      const double eps = 1e-8;
      const double r = sqrt(p(0) * p(0) + p(1) * p(1));
      if(r < eps)
      {
          return Eigen::Matrix2d::Identity();
      }

      const double k1 = _distortionParams.at(0);
      const double k2 = _distortionParams.at(1);
      const double k3 = _distortionParams.at(2);
      const double k4 = _distortionParams.at(3);

      const double theta = std::atan(r);
      const double theta2 = theta * theta;
      const double theta3 = theta2 * theta;
      const double theta4 = theta2 * theta2;
      const double theta5 = theta4 * theta;
      const double theta6 = theta3 * theta3;
      const double theta7 = theta6 * theta;
      const double theta8 = theta4 * theta4;
      const double theta9 = theta8 * theta;
      const double theta_dist = theta + k1 * theta3 + k2 * theta5 + k3 * theta7 + k4 * theta9;
      const double cdist = theta_dist / r;

      const double d_theta_dist_d_theta = 1.0 + 3.0 * k1 * theta2 + 5.0 * k2 * theta4 + 7.0 * k3 * theta6 + 9.0 * k4 * theta8;
      Eigen::Matrix<double, 1, 2> d_r_d_p;
      d_r_d_p(0) = p(0) / r;
      d_r_d_p(1) = p(1) / r;

      const double d_cdist_d_r = -theta_dist / (r * r);
      const double d_cdist_d_theta_dist = 1.0 / r;
      const double d_theta_d_r = 1.0 / (r * r + 1.0);

      const Eigen::Matrix<double, 1, 2> d_cdist_d_p =
          d_cdist_d_r * d_r_d_p + d_cdist_d_theta_dist * d_theta_dist_d_theta * d_theta_d_r * d_r_d_p;

      return Eigen::Matrix2d::Identity() * cdist + p * d_cdist_d_p;
  }

  Eigen::MatrixXd getDerivativeAddDistoWrtDisto(const Vec2 & p) const  override
  {
    
    const double eps = 1e-8;

    const double r = sqrt(p(0)*p(0) + p(1)*p(1));
    if (r < eps)
    {
      return Eigen::Matrix<double, 2, 4>::Zero();
    }

    const double theta = std::atan(r);
    const double theta2 = theta*theta;
    const double theta3 = theta2*theta;
    const double theta4 = theta2*theta2;
    const double theta5 = theta4*theta;
    const double theta6 = theta3*theta3;
    const double theta7 = theta6*theta;
    const double theta8 = theta4*theta4;
    const double theta9 = theta8*theta;

    const double d_cdist_d_theta_dist = 1.0 / r;

    Eigen::Matrix<double, 1, 4> d_r_theta_dist_d_params;
    d_r_theta_dist_d_params(0, 0) = theta3;
    d_r_theta_dist_d_params(0, 1) = theta5;
    d_r_theta_dist_d_params(0, 2) = theta7;
    d_r_theta_dist_d_params(0, 3) = theta9;

    const Eigen::MatrixXd ret = p * d_cdist_d_theta_dist * d_r_theta_dist_d_params;

    return ret;
  }

  /// Remove distortion (return p' such that disto(p') = p)
  Vec2 removeDistortion(const Vec2& p) const override
  {
    const double eps = 1e-8;
    double scale = 1.0;
    const double theta_dist = std::hypot(p[0], p[1]);
    if (theta_dist > eps)
    {
      double theta = theta_dist;
      for (int j = 0; j < 10; ++j)
      {
        const double theta2 = theta*theta;
        const double theta4 = theta2*theta2;
        const double theta6 = theta4*theta2;
        const double theta8 = theta6*theta2;
        theta = theta_dist / (1 + _distortionParams.at(0) * theta2 + _distortionParams.at(1) * theta4 + _distortionParams.at(2) * theta6 + _distortionParams.at(3) * theta8);
      }
      scale = std::tan(theta) / theta_dist;
    }
    return p * scale;
  }

  Eigen::Matrix2d getDerivativeRemoveDistoWrtPt(const Vec2 & p) const override
  {
    const double eps = 1e-8;
    const double r = sqrt(p(0)*p(0) + p(1)*p(1));
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
    if (r_dist < eps) {
      return Eigen::Matrix<double, 2, 4>::Zero();
    }

    const Vec2 p_undist = removeDistortion(p);
    const double r = sqrt(p_undist(0) * p_undist(0) + p_undist(1) * p_undist(1));

    const double k1 = _distortionParams.at(0);
    const double k2 = _distortionParams.at(1);
    const double k3 = _distortionParams.at(2);
    const double k4 = _distortionParams.at(3);

    const double theta = std::atan(r);
    const double theta2 = theta*theta;
    const double theta3 = theta2*theta;
    const double theta4 = theta2*theta2;
    const double theta5 = theta4*theta;
    const double theta6 = theta3*theta3;
    const double theta7 = theta6*theta;
    const double theta8 = theta4*theta4;
    const double theta9 = theta8*theta;
    const double theta_dist = theta + k1 * theta3 + k2 * theta5 + k3 * theta7 + k4 * theta9;
    const double r_coeff = theta_dist / r;
    double d_r_coeff_d_theta_dist = 1.0 / r;

    Eigen::Matrix<double, 1, 4> d_r_theta_dist_d_params;
    d_r_theta_dist_d_params(0, 0) = theta3;
    d_r_theta_dist_d_params(0, 1) = theta5;
    d_r_theta_dist_d_params(0, 2) = theta7;
    d_r_theta_dist_d_params(0, 3) = theta9;

    Eigen::Matrix<double, 1, 4> d_rcoeff_d_params = d_r_coeff_d_theta_dist * d_r_theta_dist_d_params;
    
    Eigen::Matrix<double, 2, 4> ret;
    ret(0, 0) = - (p(0) * d_rcoeff_d_params(0, 0)) / (r_coeff * r_coeff);
    ret(0, 1) = - (p(0) * d_rcoeff_d_params(0, 1)) / (r_coeff * r_coeff);
    ret(0, 2) = - (p(0) * d_rcoeff_d_params(0, 2)) / (r_coeff * r_coeff);
    ret(0, 3) = - (p(0) * d_rcoeff_d_params(0, 3)) / (r_coeff * r_coeff);
    ret(1, 0) = - (p(1) * d_rcoeff_d_params(0, 0)) / (r_coeff * r_coeff);
    ret(1, 1) = - (p(1) * d_rcoeff_d_params(0, 1)) / (r_coeff * r_coeff);
    ret(1, 2) = - (p(1) * d_rcoeff_d_params(0, 2)) / (r_coeff * r_coeff);
    ret(1, 3) = - (p(1) * d_rcoeff_d_params(0, 3)) / (r_coeff * r_coeff);

    return ret;
  }

  ~DistortionFisheye() override = default;
};

} // namespace camera
} // namespace aliceVision
