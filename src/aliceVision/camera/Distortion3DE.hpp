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

/**
 * @class DistortionRadial3DE
 * @brief 3DE radial distortion
 */
class Distortion3DERadial4 : public Distortion {
public:
  /**
   * @brief Default constructor, no distortion.
   */
  Distortion3DERadial4()
  {
    _distortionParams = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  }

  /**
   * @brief Constructor with the three coefficients
   */
  explicit Distortion3DERadial4(double c2, double c4, double u1, double v1, double u3, double v3)
  {
    _distortionParams = {c2, c4, u1, v1, u3, v3};
  }

  Distortion3DERadial4* clone() const override { return new Distortion3DERadial4(*this); }

  /// Add distortion to the point p (assume p is in the camera frame [normalized coordinates])
  Vec2 addDistortion(const Vec2 & p) const override
  {
    const double c2 = _distortionParams[0];
    const double c4 = _distortionParams[1];
    const double u1 = _distortionParams[2];
    const double v1 = _distortionParams[3];
    const double u3 = _distortionParams[4];
    const double v3 = _distortionParams[5];

    Vec2 np;

    double x = p.x();
    double y = p.y();
    double xx = x * x;
    double yy = y * y;
    double xy = x * y;
    double r2 = xx + yy;
    double r4 = r2 * r2;

    double p1 = 1.0 + c2 * r2 + c4 * r4;
    double p2 = r2 + 2.0 * xx;
    double p3 = r2 + 2.0 * yy;

    double p4 = u1 + u3 * r2;
    double p5 = v1 + v3 * r2;
    double p6 = 2.0 * xy;
    

    np.x() = x * p1 + p2 * p4 + p6 * p5;
    np.y() = y * p1 + p3 * p5 + p6 * p4;

    return np;
  }

  Eigen::Matrix2d getDerivativeAddDistoWrtPt(const Vec2 & p) const override
  {
    const double c2 = _distortionParams[0];
    const double c4 = _distortionParams[1];
    const double u1 = _distortionParams[2];
    const double v1 = _distortionParams[3];
    const double u3 = _distortionParams[4];
    const double v3 = _distortionParams[5];

    Vec2 np;

    double x = p.x();
    double y = p.y();
    double xx = x * x;
    double yy = y * y;
    double xy = x * y;
    double r2 = xx + yy;
    double r4 = r2 * r2;

    const double eps = 1e-16;
    if (r2 < eps) {
      return Eigen::Matrix<double, 2, 2>::Zero();
    }

    double p1 = 1.0 + c2 * r2 + c4 * r4;
    double p2 = r2 + 2.0 * xx;
    double p3 = r2 + 2.0 * yy;

    double p4 = u1 + u3 * r2;
    double p5 = v1 + v3 * r2;
    double p6 = 2.0 * xy;
    

    //np.x() = x * p1 + p2 * p4 + p6 * p5;
    //np.y() = y * p1 + p3 * p5 + p6 * p4;

    Eigen::Matrix<double, 1, 2> d_r2_d_p;
    d_r2_d_p(0, 0) = 2.0 * x;
    d_r2_d_p(0, 1) = 2.0 * y;

    double d_p1_d_r2 = c2 + 2.0 * c4 * r2;
    double d_p4_d_r2 = u3;
    double d_p5_d_r2 = v3;

    Eigen::Matrix<double, 1, 2> d_p1_d_p = d_p1_d_r2 * d_r2_d_p;
    
    Eigen::Matrix<double, 1, 2> d_p2_d_p;
    d_p2_d_p(0, 0) = d_r2_d_p(0, 0) + 4.0 * x;
    d_p2_d_p(0, 1) = d_r2_d_p(0, 1);

    Eigen::Matrix<double, 1, 2> d_p3_d_p;
    d_p3_d_p(0, 0) = d_r2_d_p(0, 0);
    d_p3_d_p(0, 1) = d_r2_d_p(0, 1) + 4.0 * y;

    Eigen::Matrix<double, 1, 2> d_p4_d_p = d_p4_d_r2 * d_r2_d_p;
    Eigen::Matrix<double, 1, 2> d_p5_d_p = d_p5_d_r2 * d_r2_d_p;

    Eigen::Matrix<double, 1, 2> d_p6_d_p;
    d_p6_d_p(0, 0) = 2.0 * y;
    d_p6_d_p(0, 1) = 2.0 * x;

    Eigen::Matrix<double, 1, 2> d_x_d_p;
    d_x_d_p(0, 0) = 1.0;
    d_x_d_p(0, 1) = 0.0;

    Eigen::Matrix<double, 1, 2> d_y_d_p;
    d_y_d_p(0, 0) = 0.0;
    d_y_d_p(0, 1) = 1.0;

    Eigen::Matrix<double, 2, 2> ret;
    ret.block<1, 2>(0, 0) = (x * d_p1_d_p + d_x_d_p * p1) + (p2 * d_p4_d_p + d_p2_d_p * p4) + (p6 * d_p5_d_p + d_p6_d_p * p5);
    ret.block<1, 2>(1, 0) = (y * d_p1_d_p + d_y_d_p * p1) + (p3 * d_p5_d_p + d_p3_d_p * p5) + (p6 * d_p4_d_p + d_p6_d_p * p4);
 //np.x() = x * p1 + p2 * p4 + p6 * p5;
    //np.y() = y * p1 + p3 * p5 + p6 * p4;
    return ret;
  }

  Eigen::MatrixXd getDerivativeAddDistoWrtDisto(const Vec2 & p) const  override
  {
    const double c2 = _distortionParams[0];
    const double c4 = _distortionParams[1];
    const double u1 = _distortionParams[2];
    const double v1 = _distortionParams[3];
    const double u3 = _distortionParams[4];
    const double v3 = _distortionParams[5];
    
    Vec2 np;

    double x = p.x();
    double y = p.y();
    double xx = x * x;
    double yy = y * y;
    double xy = x * y;
    double r2 = xx + yy;
    double r4 = r2 * r2;

    const double eps = 1e-8;
    if (r2 < eps) {
      return Eigen::Matrix<double, 2, 6>::Zero();
    }


    double p1 = 1.0 + c2 * r2 + c4 * r4;
    double p2 = r2 + 2.0 * xx;
    double p3 = r2 + 2.0 * yy;

    double p4 = u1 + u3 * r2;
    double p5 = v1 + v3 * r2;
    double p6 = 2.0 * xy;
    

    //np.x() = x * p1 + p2 * p4 + p6 * p5;
    //np.y() = y * p1 + p3 * p5 + p6 * p4;
    Eigen::Matrix<double, 1, 6> d_p1_d_disto;
    d_p1_d_disto(0, 0) = r2;
    d_p1_d_disto(0, 1) = r4;
    d_p1_d_disto(0, 2) = 0;
    d_p1_d_disto(0, 3) = 0;
    d_p1_d_disto(0, 4) = 0;
    d_p1_d_disto(0, 5) = 0;

    Eigen::Matrix<double, 1, 6> d_p4_d_disto;
    d_p4_d_disto(0, 0) = 0;
    d_p4_d_disto(0, 1) = 0;
    d_p4_d_disto(0, 2) = 1.0;
    d_p4_d_disto(0, 3) = 0;
    d_p4_d_disto(0, 4) = r2;
    d_p4_d_disto(0, 5) = 0;

    Eigen::Matrix<double, 1, 6> d_p5_d_disto;
    d_p5_d_disto(0, 0) = 0;
    d_p5_d_disto(0, 1) = 0;
    d_p5_d_disto(0, 2) = 0;
    d_p5_d_disto(0, 3) = 1.0;
    d_p5_d_disto(0, 4) = 0;
    d_p5_d_disto(0, 5) = r2;

    Eigen::Matrix<double, 2, 6> ret;

    ret.block<1, 6>(0, 0) = x * d_p1_d_disto + p2 * d_p4_d_disto + p6 * d_p5_d_disto;
    ret.block<1, 6>(1, 0) = y * d_p1_d_disto + p3 * d_p5_d_disto + p6 * d_p4_d_disto;


    return ret;
  }


  /// Remove distortion (return p' such that disto(p') = p)
  Vec2 removeDistortion(const Vec2& p) const override
  {
    double epsilon = 1e-8;
    Vec2 undistorted_value = p;

    Vec2 diff = addDistortion(undistorted_value) - p;

    int iter = 0;
    while (diff.norm() > epsilon)
    {
      undistorted_value = undistorted_value - getDerivativeAddDistoWrtPt(undistorted_value).inverse() * diff;
      diff = addDistortion(undistorted_value) - p;
      iter++;
      if (iter > 100) break;
    }

    return undistorted_value;
  }

  Eigen::Matrix2d getDerivativeRemoveDistoWrtPt(const Vec2 & p) const override
  {
    std::cout << "invalid class for getDerivativeRemoveDistoWrtPt" << std::endl;
    return Eigen::MatrixXd();
  }

  Eigen::MatrixXd getDerivativeRemoveDistoWrtDisto(const Vec2 & p) const override
  {
    
    std::cout << "invalid class for getDerivativeRemoveDistoWrtDisto" << std::endl;
    return Eigen::MatrixXd();
  }

  double getUndistortedRadius(double r) const override
  {
    std::cout << "invalid class for getUndistortedRadius" << std::endl;
    return 0.0;
  }

  ~Distortion3DERadial4() override = default;
};

/**
 * @class DistortionRadialAnamorphic4
 * @brief Anamorphic radial distortion
 */
class Distortion3DEAnamorphic4 : public Distortion {
public:
  /**
   * @brief Default constructor, no distortion.
   */
  Distortion3DEAnamorphic4()
  {
    _distortionParams = {0.0, 0.0, 0.0, 0.0};
  }

  /**
   * @brief Constructor with the three coefficients
   */
  explicit Distortion3DEAnamorphic4(double cxx, double cxy, double cyx, double cyy)
  {
    _distortionParams = {cxx, cxy, cyx, cyy};
  }

  Distortion3DEAnamorphic4* clone() const override { return new Distortion3DEAnamorphic4(*this); }

  /// Add distortion to the point p (assume p is in the camera frame [normalized coordinates])
  Vec2 addDistortion(const Vec2 & p) const override
  {
    const double cxx = _distortionParams[0];
    const double cxy = _distortionParams[1];
    const double cyx = _distortionParams[2];
    const double cyy = _distortionParams[3];

    Vec2 np;

    double x = p.x();
    double y = p.y();
    double xx = x * x;
    double yy = y * y;

    np.x() = x * (1.0 + cxx * xx + cxy * yy);
    np.y() = y * (1.0 + cyx * xx + cyy * yy);

    return np;
  }

  Eigen::Matrix2d getDerivativeAddDistoWrtPt(const Vec2 & p) const override
  {
    const double cxx = _distortionParams[0];
    const double cxy = _distortionParams[1];
    const double cyx = _distortionParams[2];
    const double cyy = _distortionParams[3];

    double x = p.x();
    double y = p.y();
    double xx = x * x;
    double yy = y * y;

    Eigen::Matrix2d ret;

    //np.x() = x * (1.0 + cxx * xx + cxy * yy);
    //np.y() = y * (1.0 + cyx * xx + cyy * yy);

    //np.x() = x + cxx * xxx + cxy * xyy);
    //np.x() = y + cxx * yxx + cxy * yyy);

    ret(0, 0) = 1.0 + 3.0 * cxx * xx + cxy * yy;
    ret(0, 1) = 2.0 * cxy * y;
    ret(1, 0) = 2.0 * cyx * x;
    ret(1, 1) = 1.0 + cxx * xx + 3.0 * cxy * yy;

    return ret;
  }

  Eigen::MatrixXd getDerivativeAddDistoWrtDisto(const Vec2 & p) const  override
  {
    const double cxx = _distortionParams[0];
    const double cxy = _distortionParams[1];
    const double cyx = _distortionParams[2];
    const double cyy = _distortionParams[3];

    Vec2 np;

    double x = p.x();
    double y = p.y();
    double xx = x * x;
    double yy = y * y;

    Eigen::Matrix<double, 2, 4> ret;

    ret(0, 0) = x * xx;
    ret(0, 1) = x * yy;
    ret(0, 2) = 0;
    ret(0, 3) = 0;

    ret(1, 0) = 0;
    ret(1, 1) = 0;
    ret(1, 2) = y * xx;
    ret(1, 3) = y * yy;

    return ret;
  }


  /// Remove distortion (return p' such that disto(p') = p)
  Vec2 removeDistortion(const Vec2& p) const override
  {
    double epsilon = 1e-8;
    Vec2 undistorted_value = p;

    Vec2 diff = addDistortion(undistorted_value) - p;

    int iter = 0;
    while (diff.norm() > epsilon)
    {
      undistorted_value = undistorted_value - getDerivativeAddDistoWrtPt(undistorted_value).inverse() * diff;
      diff = addDistortion(undistorted_value) - p;
      iter++;
      if (iter > 10) break;
    }

    return undistorted_value;
  }

  Eigen::Matrix2d getDerivativeRemoveDistoWrtPt(const Vec2 & p) const override
  {
    std::cout << "invalid class for getDerivativeRemoveDistoWrtPt" << std::endl;
    return Eigen::MatrixXd();
  }

  Eigen::MatrixXd getDerivativeRemoveDistoWrtDisto(const Vec2 & p) const override
  {
    
    std::cout << "invalid class for getDerivativeRemoveDistoWrtDisto" << std::endl;
    return Eigen::MatrixXd();
  }

  double getUndistortedRadius(double r) const override
  {
    std::cout << "invalid class for getUndistortedRadius" << std::endl;
    return 0.0;
  }

  ~Distortion3DEAnamorphic4() override = default;
};

/**
 * @class Distortion3DEClassicLD
 * @brief Anamorphic radial distortion
 */
class Distortion3DEClassicLD : public Distortion {
public:
  /**
   * @brief Default constructor, no distortion.
   */
  Distortion3DEClassicLD()
  {
    _distortionParams = {0.0, 1.0, 0.0, 0.0, 0.0};
  }

  /**
   * @brief Constructor with the three coefficients
   */
  explicit Distortion3DEClassicLD(double delta = 0.0, double epsilon = 1.0, double mux = 0.0, double muy = 0.0, double q = 0.0)
  {
    _distortionParams = {delta, 1.0 / epsilon, mux, muy, q};
  }

  Distortion3DEClassicLD* clone() const override { return new Distortion3DEClassicLD(*this); }

  /// Add distortion to the point p (assume p is in the camera frame [normalized coordinates])
  Vec2 addDistortion(const Vec2 & p) const override
  {
    const double delta = _distortionParams[0];
    const double invepsilon = _distortionParams[1];
    const double mux = _distortionParams[2];
    const double muy = _distortionParams[3];
    const double q = _distortionParams[4];

    const double eps = 1.0 + cos(invepsilon);

    const double cxx = delta * eps;
    const double cxy = (delta + mux) * eps;
    const double cxxx = q * eps;
    const double cxxy = 2.0 * q * eps;
    const double cxyy = q * eps;
    const double cyx = delta + muy;
    const double cyy = delta;
    const double cyxx = q;
    const double cyxy = 2.0 * q;
    const double cyyy = q;
    

    Vec2 np;

    double x = p.x();
    double y = p.y();
    double xx = x * x;
    double yy = y * y;
    double xxxx = xx * xx;
    double yyyy = yy * yy;
    double xxyy = xx * yy;
    

    np.x() = x * (1.0 + cxx * xx + cxy * yy + cxxx * xxxx + cxxy * xxyy + cxyy * yyyy);
    np.y() = y * (1.0 + cyx * xx + cyy * yy + cyxx * xxxx + cyxy * xxyy + cyyy * yyyy);

    return np;
  }

  Eigen::Matrix2d getDerivativeAddDistoWrtPt(const Vec2 & p) const override
  {
    const double delta = _distortionParams[0];
    const double invepsilon = _distortionParams[1];
    const double mux = _distortionParams[2];
    const double muy = _distortionParams[3];
    const double q = _distortionParams[4];
    
    const double eps = 1.0 + cos(invepsilon);

    const double cxx = delta * eps;
    const double cxy = (delta + mux) * eps;
    const double cxxx = q * eps;
    const double cxxy = 2.0 * q * eps;
    const double cxyy = q * eps;
    const double cyx = delta + muy;
    const double cyy = delta;
    const double cyxx = q;
    const double cyxy = 2.0 * q;
    const double cyyy = q;
    

    double x = p.x();
    double y = p.y();
    double xx = x * x;
    double yy = y * y;
    double xy = x * y;

    double xxx = xx * x;
    double yyy = yy * y;
    double xxy = xx * y;
    double xyy = x * yy;
    
    double xxxx = xx * xx;
    double yyyy = yy * yy;
    double xxyy = xx * yy;

    double xyyy = x * yyy;
    double xxxy = xxx * y;

    Eigen::Matrix2d ret;

    //np.x() = x * (1.0 + cxx * xx + cxy * yy + cxxx * xxxx + cxxy * xxyy + cxyy * yyyy);
    //np.y() = y * (1.0 + cyx * xx + cyy * yy + cyxx * xxxx + cyxy * xxyy + cyyy * yyyy);

    //np.x() = x + cxx * xxx + cxy * xyy + cxxx * xxxxx + cxxy * xxxyy + cxyy * xyyyy;
    //np.y() = y + cyx * xxy + cyy * yyy + cyxx * xxxxy + cyxy * xxyyy + cyyy * yyyyy;

    ret(0, 0) = 1.0 + 3.0 * cxx * xx + cxy * yy + 5.0 * cxxx * xxxx + 3.0 * cxxy * xxyy + cxyy * yyyy;
    ret(0, 1) = 2.0 * cxy * xy + 2.0 * cxxy * xxxy + 4.0 * cxyy * xyyy;
    ret(1, 0) = 2.0 * cyx * xy + 4.0 * cyxx * xxxy + 2.0 * cyxy * xyyy;
    ret(1, 1) = 1.0 + cyx * xx + 3.0 * cyy * yy + cyxx * xxxx + 3.0 * cyxy * xxyy + 5.0 * cyyy * yyyy;

    return ret;
  }

  Eigen::MatrixXd getDerivativeAddDistoWrtDisto(const Vec2 & p) const  override
  {
    const double delta = _distortionParams[0];
    const double invepsilon = _distortionParams[1];
    const double mux = _distortionParams[2];
    const double muy = _distortionParams[3];
    const double q = _distortionParams[4];
    
    const double eps = 1.0 + cos(invepsilon);

    const double cxx = delta * eps;
    const double cxy = (delta + mux) * eps;
    const double cxxx = q * eps;
    const double cxxy = 2.0 * q * eps;
    const double cxyy = q * eps;
    const double cyx = delta + muy;
    const double cyy = delta;
    const double cyxx = q;
    const double cyxy = 2.0 * q;
    const double cyyy = q;

    

    Vec2 np;

    double x = p.x();
    double y = p.y();
    double xx = x * x;
    double yy = y * y;

    double xxx = xx * x;
    double xyy = x * yy;
    double xxy = xx * y;
    double yyy = yy * y;
    double xxxx = xx * xx;
    double xxxxx = xxxx * x;
    double yyyy = yy * yy;
    double yyyyy = yyyy * y;
    double xxxxy = xxxx * y;
    double xxxyy = xxx * yy;
    double xxyyy = xx * yyy;
    double xyyyy = x * yyyy;

    Eigen::Matrix<double, 2, 10> ret;

    //np.x() = x + cxx * xxx + cxy * xyy + cxxx * xxxxx + cxxy * xxxyy + cxyy * xyyyy;
    //np.y() = y + cyx * xxy + cyy * yyy + cyxx * xxxxy + cyxy * xxyyy + cyyy * yyyyy;

    ret(0, 0) = xxx;
    ret(0, 1) = xyy;
    ret(0, 2) = xxxxx;
    ret(0, 3) = xxxyy;
    ret(0, 4) = xyyyy;
    ret(0, 5) = 0;
    ret(0, 6) = 0;
    ret(0, 7) = 0;
    ret(0, 8) = 0;
    ret(0, 9) = 0;


    ret(1, 0) = 0;
    ret(1, 1) = 0;
    ret(1, 2) = 0;
    ret(1, 3) = 0;
    ret(1, 4) = 0;
    ret(1, 5) = xxy;
    ret(1, 6) = yyy;
    ret(1, 7) = xxxxy;
    ret(1, 8) = xxyyy;
    ret(1, 9) = yyyyy;


    Eigen::Matrix<double, 10, 5> localParams = Eigen::Matrix<double, 10, 5>::Zero(); 
    
    const double d_eps_d_invepsilon = -sin(invepsilon);

    localParams(0, 0) = eps;
    localParams(0, 1) = delta * d_eps_d_invepsilon;
    localParams(1, 0) = eps;
    localParams(1, 1) = (delta + mux) * d_eps_d_invepsilon;
    localParams(1, 2) = eps;
    localParams(2, 1) = q * d_eps_d_invepsilon;
    localParams(2, 4) = eps;
    localParams(3, 1) = 2.0 * q * d_eps_d_invepsilon;
    localParams(3, 4) = 2.0 * eps;
    localParams(4, 1) = q * d_eps_d_invepsilon;
    localParams(4, 4) = eps;
    localParams(5, 0) = 1;
    localParams(5, 3) = 1;
    localParams(6, 0) = 1;
    localParams(7, 4) = 1;
    localParams(8, 4) = 2;
    localParams(9, 4) = 1;


    return ret * localParams;
  }


  /// Remove distortion (return p' such that disto(p') = p)
  Vec2 removeDistortion(const Vec2& p) const override
  {
    double epsilon = 1e-8;
    Vec2 undistorted_value = p;

    Vec2 diff = addDistortion(undistorted_value) - p;

    int iter = 0;
    while (diff.norm() > epsilon)
    {
      undistorted_value = undistorted_value - getDerivativeAddDistoWrtPt(undistorted_value).inverse() * diff;
      diff = addDistortion(undistorted_value) - p;
      iter++;
      if (iter > 1000) break;
    }


    return undistorted_value;
  }

  Eigen::Matrix2d getDerivativeRemoveDistoWrtPt(const Vec2 & p) const override
  {
    std::cout << "invalid class for getDerivativeRemoveDistoWrtPt" << std::endl;
    return Eigen::MatrixXd();
  }

  Eigen::MatrixXd getDerivativeRemoveDistoWrtDisto(const Vec2 & p) const override
  {
    
    std::cout << "invalid class for getDerivativeRemoveDistoWrtDisto" << std::endl;
    return Eigen::MatrixXd();
  }

  double getUndistortedRadius(double r) const override
  {
    std::cout << "invalid class for getUndistortedRadius" << std::endl;
    return 0.0;
  }

  ~Distortion3DEClassicLD() override = default;
};

} // namespace camera
} // namespace aliceVision
