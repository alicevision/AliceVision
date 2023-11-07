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

class IntrinsicsManifoldSymbolic : public ceres::Manifold
{
  public:
    explicit IntrinsicsManifoldSymbolic(size_t parametersSize,
                                        double focalRatio,
                                        bool lockFocal,
                                        bool lockFocalRatio,
                                        bool lockCenter,
                                        bool lockDistortion)
      : _ambientSize(parametersSize),
        _focalRatio(focalRatio),
        _lockFocal(lockFocal),
        _lockFocalRatio(lockFocalRatio),
        _lockCenter(lockCenter),
        _lockDistortion(lockDistortion)
    {
        _distortionSize = _ambientSize - 4;
        _tangentSize = 0;

        if (!_lockFocal)
        {
            if (_lockFocalRatio)
            {
                _tangentSize += 1;
            }
            else
            {
                _tangentSize += 2;
            }
        }

        if (!_lockCenter)
        {
            _tangentSize += 2;
        }

        if (!_lockDistortion)
        {
            _tangentSize += _distortionSize;
        }
    }

    virtual ~IntrinsicsManifoldSymbolic() = default;

    bool Plus(const double* x, const double* delta, double* x_plus_delta) const override
    {
        for (int i = 0; i < _ambientSize; i++)
        {
            x_plus_delta[i] = x[i];
        }

        size_t posDelta = 0;
        if (!_lockFocal)
        {
            if (_lockFocalRatio)
            {
                x_plus_delta[0] = x[0] + delta[posDelta];
                x_plus_delta[1] = x[1] + _focalRatio * delta[posDelta];
                ++posDelta;
            }
            else
            {
                x_plus_delta[0] = x[0] + delta[posDelta];
                ++posDelta;
                x_plus_delta[1] = x[1] + delta[posDelta];
                ++posDelta;
            }
        }

        if (!_lockCenter)
        {
            x_plus_delta[2] = x[2] + delta[posDelta];
            ++posDelta;

            x_plus_delta[3] = x[3] + delta[posDelta];
            ++posDelta;
        }

        if (!_lockDistortion)
        {
            for (int i = 0; i < _distortionSize; i++)
            {
                x_plus_delta[4 + i] = x[4 + i] + delta[posDelta];
                ++posDelta;
            }
        }

        return true;
    }

    bool PlusJacobian(const double* x, double* jacobian) const override
    {
        Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> J(jacobian, AmbientSize(), TangentSize());

        J.fill(0);

        size_t posDelta = 0;
        if (!_lockFocal)
        {
            if (_lockFocalRatio)
            {
                J(0, posDelta) = 1.0;
                J(1, posDelta) = _focalRatio;
                ++posDelta;
            }
            else
            {
                J(0, posDelta) = 1.0;
                ++posDelta;
                J(1, posDelta) = 1.0;
                ++posDelta;
            }
        }

        if (!_lockCenter)
        {
            J(2, posDelta) = 1.0;
            ++posDelta;

            J(3, posDelta) = 1.0;
            ++posDelta;
        }

        if (!_lockDistortion)
        {
            for (int i = 0; i < _distortionSize; i++)
            {
                J(4 + i, posDelta) = 1.0;
                ++posDelta;
            }
        }

        return true;
    }

    bool Minus(const double* y, const double* x, double* delta) const override
    {
        throw std::invalid_argument("IntrinsicsManifold::Minus() should never be called");
    }

    bool MinusJacobian(const double* x, double* jacobian) const override
    {
        throw std::invalid_argument("IntrinsicsManifold::MinusJacobian() should never be called");
    }

    int AmbientSize() const override { return _ambientSize; }

    int TangentSize() const override { return _tangentSize; }

  private:
    size_t _distortionSize;
    size_t _ambientSize;
    size_t _tangentSize;
    double _focalRatio;
    bool _lockFocal;
    bool _lockFocalRatio;
    bool _lockCenter;
    bool _lockDistortion;
};

}  // namespace sfm
}  // namespace aliceVision
