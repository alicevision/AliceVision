// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/geometry/lie.hpp>
#include <Eigen/Dense>
#include <unsupported/Eigen/KroneckerProduct>
#include <aliceVision/utils/CeresUtils.hpp>
#include <ceres/ceres.h>

namespace aliceVision {
namespace SO3 {

/**
Compute the jacobian of the logarithm wrt changes in the rotation matrix values
@param R the input rotation matrix
@return the jacobian matrix (3*9 matrix)
*/
inline Eigen::Matrix<double, 3, 9, Eigen::RowMajor> dlogmdr(const Eigen::Matrix3d & R) {
    double p1 = R(2, 1) - R(1, 2);
    double p2 = R(0, 2) - R(2, 0);
    double p3 = R(1, 0) - R(0, 1);

    double costheta = (R.trace() - 1.0) / 2.0;
    if (costheta > 1.0) costheta = 1.0;
    else if (costheta < -1.0) costheta = -1.0;

    double theta = acos(costheta);

    if (fabs(theta) < std::numeric_limits<float>::epsilon()) {
        Eigen::Matrix<double, 3, 9> J;
        J.fill(0);
        J(0, 5) = 1;
        J(0, 7) = -1;
        J(1, 2) = -1;
        J(1, 6) = 1;
        J(2, 1) = 1;
        J(2, 3) = -1;
        return J;
    }

    double scale = theta / (2.0 * sin(theta));

    Eigen::Vector3d resnoscale;
    resnoscale(0) = p1;
    resnoscale(1) = p2;
    resnoscale(2) = p3;

    Eigen::Matrix<double, 3, 3> dresdp = Eigen::Matrix3d::Identity() * scale;
    Eigen::Matrix<double, 3, 9> dpdmat;
    dpdmat.fill(0);
    dpdmat(0, 5) = 1;
    dpdmat(0, 7) = -1;
    dpdmat(1, 2) = -1;
    dpdmat(1, 6) = 1;
    dpdmat(2, 1) = 1;
    dpdmat(2, 3) = -1;


    double dscaledtheta = -0.5 * theta * cos(theta) / (sin(theta)*sin(theta)) + 0.5 / sin(theta);
    double dthetadcostheta = -1.0 / sqrt(-costheta*costheta + 1.0);

    Eigen::Matrix<double, 1, 9> dcosthetadmat;
    dcosthetadmat << 0.5, 0, 0, 0, 0.5, 0, 0, 0, 0.5;
    Eigen::Matrix<double, 1, 9> dscaledmat = dscaledtheta * dthetadcostheta * dcosthetadmat;

    return dpdmat * scale + resnoscale * dscaledmat;
}

class Manifold : public utils::CeresManifold {
 public:
  ~Manifold() override = default;

  bool Plus(const double* x, const double* delta, double* x_plus_delta) const override {
 
    double* ptrBase = (double*)x;
    double* ptrResult = (double*)x_plus_delta;
    Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor> > rotation(ptrBase);
    Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor> > rotationResult(ptrResult);

    Eigen::Vector3d axis;
    axis(0) = delta[0];
    axis(1) = delta[1];
    axis(2) = delta[2];
    double angle = axis.norm();

    axis.normalize();

    Eigen::AngleAxisd aa(angle, axis);
    Eigen::Matrix3d Rupdate;
    Rupdate = aa.toRotationMatrix();

    rotationResult = Rupdate * rotation;

    return true;
  }

  bool PlusJacobian(const double* /*x*/, double* jacobian) const override {
    
    Eigen::Map<Eigen::Matrix<double, 9, 3, Eigen::RowMajor>> J(jacobian);
    //Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> R(x);

    J.fill(0);

    J(1, 2) = 1;
    J(2, 1) = -1;
    J(3, 2) = -1;
    J(5, 0) = 1;
    J(6, 1) = 1;
    J(7, 0) = -1;

    return true;
  }

  bool Minus(const double* y, const double* x, double* delta) const override {
    throw std::invalid_argument("SO3::Manifold::Minus() should never be called");
  }

  bool MinusJacobian(const double* x, double* jacobian) const override {
    throw std::invalid_argument("SO3::Manifold::MinusJacobian() should never be called");
  }

  int AmbientSize() const override { return 9; }

  int TangentSize() const override { return 3; }
};

}//namespace SO3 

namespace SE3 {


class ManifoldLeft : public utils::CeresManifold {
public:
  ManifoldLeft(bool refineRotation, bool refineTranslation) :
  _refineRotation(refineRotation), 
  _refineTranslation(refineTranslation)
  {
  }

  bool Plus(const double* x, const double* delta, double* x_plus_delta) const override {

    Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> T(x);
    Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> T_result(x_plus_delta);
    Eigen::Map<const Eigen::Matrix<double, 6, 1>> vec_update(delta);
    Eigen::Matrix4d T_update = Eigen::Matrix4d::Identity();

    T_update = expm(vec_update);
    T_result = T_update * T;

    return true;
  }

  bool PlusJacobian(const double * x, double* jacobian) const override {

    Eigen::Map<Eigen::Matrix<double, 16, 6, Eigen::RowMajor>> J(jacobian);

    J.fill(0);

    if (_refineRotation) 
    {
      J(1, 2) = 1;
      J(2, 1) = -1;

      J(4, 2) = -1;
      J(6, 0) = 1;

      J(8, 1) = 1;
      J(9, 0) = -1;
    }

    if (_refineTranslation) 
    {
      J(12, 3) = 1;
      J(13, 4) = 1;
      J(14, 5) = 1;
    }

    return true;
  }

  bool Minus(const double* y, const double* x, double* delta) const override {
    throw std::invalid_argument("SE3::Manifold::Minus() should never be called");
  }

  bool MinusJacobian(const double* x, double* jacobian) const override {
    throw std::invalid_argument("SE3::Manifold::MinusJacobian() should never be called");
  }

  int AmbientSize() const override {
    return 16;
  }

  int TangentSize() const override {
    return 6;
  }

private:
  bool _refineRotation;
  bool _refineTranslation;
};


class ManifoldRight : public utils::CeresManifold {
public:
  ManifoldRight(bool refineRotation, bool refineTranslation) :
  _refineRotation(refineRotation), 
  _refineTranslation(refineTranslation)
  {
  }

  bool Plus(const double* x, const double* delta, double* x_plus_delta) const override {

    Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> T(x);
    Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> T_result(x_plus_delta);
    Eigen::Map<const Eigen::Matrix<double, 6, 1>> vec_update(delta);
    Eigen::Matrix4d T_update = Eigen::Matrix4d::Identity();

    T_update = expm(vec_update);
    T_result = T * T_update;

    return true;
  }

  bool PlusJacobian(const double * x, double* jacobian) const override {

    Eigen::Map<Eigen::Matrix<double, 16, 6, Eigen::RowMajor>> J(jacobian);
    Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> T(x);

    J.fill(0);

    if (_refineRotation) 
    {
      J(1, 2) = 1;
      J(2, 1) = -1;

      J(4, 2) = -1;
      J(6, 0) = 1;

      J(8, 1) = 1;
      J(9, 0) = -1;
    }

    if (_refineTranslation) 
    {
      J(12, 3) = 1;
      J(13, 4) = 1;
      J(14, 5) = 1;
    }

    return true;
  }

  bool Minus(const double* y, const double* x, double* delta) const override {
    throw std::invalid_argument("SE3::Manifold::Minus() should never be called");
  }

  bool MinusJacobian(const double* x, double* jacobian) const override {
    throw std::invalid_argument("SE3::Manifold::MinusJacobian() should never be called");
  }

  int AmbientSize() const override {
    return 16;
  }

  int TangentSize() const override {
    return 6;
  }

private:
  bool _refineRotation;
  bool _refineTranslation;
};

} //namespace SE3 


namespace SO2 {

class Manifold : public utils::CeresManifold {
public:
  bool Plus(const double* x, const double* delta, double* x_plus_delta) const override {

    Eigen::Map<const Eigen::Matrix<double, 2, 2, Eigen::RowMajor>> T(x);
    Eigen::Map<Eigen::Matrix<double, 2, 2, Eigen::RowMajor>> T_result(x_plus_delta);
    double update = delta[0];

    Eigen::Matrix2d T_update = expm(update);
    T_result = T_update * T;

    return true;
  }

  bool PlusJacobian(const double * x, double* jacobian) const override {

    Eigen::Map<Eigen::Matrix<double, 4, 1>> J(jacobian);

    J.fill(0);

    J(1, 0) = 1;
    J(2, 0) = -1;
    
    return true;
  }

  bool Minus(const double* y, const double* x, double* delta) const override {
    throw std::invalid_argument("SO3::Manifold::Minus() should never be called");
  }

  bool MinusJacobian(const double* x, double* jacobian) const override {
    throw std::invalid_argument("SO3::Manifold::MinusJacobian() should never be called");
  }

  int AmbientSize() const override {
    return 4;
  }

  int TangentSize() const override {
    return 1;
  }
};

} //namespace SO2 

} //namespace aliceVision 
