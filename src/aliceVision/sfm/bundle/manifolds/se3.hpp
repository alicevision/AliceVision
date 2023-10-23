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

#include <aliceVision/geometry/lie.hpp>

namespace aliceVision {
namespace sfm {

class SE3ManifoldLeft : public ceres::Manifold
{
  public:
    SE3ManifoldLeft(bool refineRotation, bool refineTranslation)
      : _refineRotation(refineRotation),
        _refineTranslation(refineTranslation)
    {}

    bool Plus(const double* x, const double* delta, double* x_plus_delta) const override
    {
        Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> T(x);
        Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> T_result(x_plus_delta);
        Eigen::Map<const Eigen::Matrix<double, 6, 1>> vec_update(delta);
        Eigen::Matrix4d T_update = Eigen::Matrix4d::Identity();

        T_update = SE3::expm(vec_update);
        T_result = T_update * T;

        return true;
    }

    bool PlusJacobian(const double* x, double* jacobian) const override
    {
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

    bool Minus(const double* y, const double* x, double* delta) const override
    {
        throw std::invalid_argument("SE3::Manifold::Minus() should never be called");
    }

    bool MinusJacobian(const double* x, double* jacobian) const override
    {
        throw std::invalid_argument("SE3::Manifold::MinusJacobian() should never be called");
    }

    int AmbientSize() const override { return 16; }

    int TangentSize() const override { return 6; }

  private:
    bool _refineRotation;
    bool _refineTranslation;
};

class SE3ManifoldRight : public ceres::Manifold
{
  public:
    SE3ManifoldRight(bool refineRotation, bool refineTranslation)
      : _refineRotation(refineRotation),
        _refineTranslation(refineTranslation)
    {}

    bool Plus(const double* x, const double* delta, double* x_plus_delta) const override
    {
        Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> T(x);
        Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> T_result(x_plus_delta);
        Eigen::Map<const Eigen::Matrix<double, 6, 1>> vec_update(delta);
        Eigen::Matrix4d T_update = Eigen::Matrix4d::Identity();

        T_update = SE3::expm(vec_update);
        T_result = T * T_update;

        return true;
    }

    bool PlusJacobian(const double* x, double* jacobian) const override
    {
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

    bool Minus(const double* y, const double* x, double* delta) const override
    {
        throw std::invalid_argument("SE3::Manifold::Minus() should never be called");
    }

    bool MinusJacobian(const double* x, double* jacobian) const override
    {
        throw std::invalid_argument("SE3::Manifold::MinusJacobian() should never be called");
    }

    int AmbientSize() const override { return 16; }

    int TangentSize() const override { return 6; }

  private:
    bool _refineRotation;
    bool _refineTranslation;
};

}  // namespace sfm

}  // namespace aliceVision
