// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/geometry/lie.hpp>
#include <Eigen/Dense>
#include <unsupported/Eigen/KroneckerProduct>
#include <ceres/ceres.h>

namespace aliceVision {
namespace sfm {

class SO2Vec : public ceres::Manifold {
 public:
  ~SO2Vec() override = default;

  bool Plus(const double* x, const double* delta, double* x_plus_delta) const override {
 
    double* ptrBase = (double*)x;
    double* ptrResult = (double*)x_plus_delta;

    double theta = delta[0];
    double old = atan2(x[1], x[0]);
    x_plus_delta[0] = cos(old + theta);
    x_plus_delta[1] = sin(old + theta);
    
    return true;
  }

  bool PlusJacobian(const double* x, double* jacobian) const override {
    
    Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> J(jacobian, AmbientSize(), TangentSize());
    double old = atan2(x[1], x[0]);

    J(0, 0) = -sin(old);
    J(1, 0) = cos(old);

    return true;
  }

  bool Minus(const double* y, const double* x, double* delta) const override {
    throw std::invalid_argument("SO3::Manifold::Minus() should never be called");
  }

  bool MinusJacobian(const double* x, double* jacobian) const override {
    throw std::invalid_argument("SO3::Manifold::MinusJacobian() should never be called");
  }

  int AmbientSize() const override { return 2; }

  int TangentSize() const override { return 1; }
};

}//namespace sfm 

} //namespace aliceVision 
