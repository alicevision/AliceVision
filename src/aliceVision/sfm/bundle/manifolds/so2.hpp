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

class SO2Manifold : public ceres::Manifold
{
  public:
    bool Plus(const double* x, const double* delta, double* x_plus_delta) const override
    {
        Eigen::Map<const Eigen::Matrix<double, 2, 2, Eigen::RowMajor>> T(x);
        Eigen::Map<Eigen::Matrix<double, 2, 2, Eigen::RowMajor>> T_result(x_plus_delta);
        double update = delta[0];

        Eigen::Matrix2d T_update = expm(update);
        T_result = T_update * T;

        return true;
    }

    bool PlusJacobian(const double* x, double* jacobian) const override
    {
        Eigen::Map<Eigen::Matrix<double, 4, 1>> J(jacobian);

        J.fill(0);

        J(1, 0) = 1;
        J(2, 0) = -1;

        return true;
    }

    bool Minus(const double* y, const double* x, double* delta) const override
    {
        throw std::invalid_argument("SO3::Manifold::Minus() should never be called");
    }

    bool MinusJacobian(const double* x, double* jacobian) const override
    {
        throw std::invalid_argument("SO3::Manifold::MinusJacobian() should never be called");
    }

    int AmbientSize() const override { return 4; }

    int TangentSize() const override { return 1; }
};

}  // namespace sfm

}  // namespace aliceVision
