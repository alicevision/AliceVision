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

class SO3Vec : public ceres::Manifold
{
  public:
    ~SO3Vec() override = default;

    bool Plus(const double* x, const double* delta, double* x_plus_delta) const override
    {
        double* ptrBase = (double*)x;
        double* ptrResult = (double*)x_plus_delta;

        Eigen::Vector3d n;
        n(0) = x[0];
        n(1) = x[1];
        n(2) = x[2];

        Eigen::Vector2d update;
        update(0) = delta[0];
        update(1) = delta[1];

        // Find a point on the plane
        int maxidx = 0;
        double maxval = std::abs(n(0));

        if (std::abs(n(1)) > maxval)
        {
            maxidx = 1;
            maxval = std::abs(n(1));
        }

        if (std::abs(n(2)) > maxval)
        {
            maxidx = 2;
            maxval = std::abs(n(2));
        }

        double sum = 0.0;
        for (int i = 0; i < 3; i++)
        {
            if (i != maxidx)
            {
                sum += n(i);
            }
        }

        Vec3 a1;
        a1(0) = 1;
        a1(1) = 1;
        a1(2) = 1;
        a1(maxidx) = -sum / n(maxidx);
        a1.normalize();

        Vec3 a2 = n.cross(a1);

        Eigen::Matrix<double, 3, 2> A;
        A.col(0) = a1;
        A.col(1) = a2;

        Vec3 new_n = SO3::expm(A * update) * n;

        x_plus_delta[0] = new_n(0);
        x_plus_delta[1] = new_n(1);
        x_plus_delta[2] = new_n(2);

        return true;
    }

    bool PlusJacobian(const double* x, double* jacobian) const override
    {
        Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> J(jacobian, AmbientSize(), TangentSize());
        J.fill(0);

        Eigen::Vector3d n;
        n(0) = x[0];
        n(1) = x[1];
        n(2) = x[2];

        // Find a point on the plane
        int maxidx = 0;
        double maxval = std::abs(n(0));

        if (std::abs(n(1)) > maxval)
        {
            maxidx = 1;
            maxval = std::abs(n(1));
        }

        if (std::abs(n(2)) > maxval)
        {
            maxidx = 2;
            maxval = std::abs(n(2));
        }

        double sum = 0.0;
        for (int i = 0; i < 3; i++)
        {
            if (i != maxidx)
            {
                sum += n(i);
            }
        }

        Vec3 a1;
        a1(0) = 1;
        a1(1) = 1;
        a1(2) = 1;
        a1(maxidx) = -sum / n(maxidx);
        a1.normalize();

        Vec3 a2 = n.cross(a1);

        Eigen::Matrix<double, 3, 2> A;
        A.col(0) = a1;
        A.col(1) = a2;

        // d(I + [A*update]_x)*n/dupdate
        //= d([A*update]_x * n)/dupdate
        //= - d([n]_x * A*update)/dupdate

        J = -SO3::skew(n) * A;

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

    int AmbientSize() const override { return 3; }

    int TangentSize() const override { return 2; }
};

}  // namespace sfm

}  // namespace aliceVision
