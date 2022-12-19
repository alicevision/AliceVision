// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <ceres/ceres.h>
#include <memory>

namespace aliceVision {
namespace utils {

// We can remove this wrapper once we can use ceres-solver 2.1 and newer. We can't do so right now
// because ceres-solver 2.1 breaks with GCC 6.x and older due to
// https://gcc.gnu.org/bugzilla/show_bug.cgi?id=56480
#define ALICEVISION_CERES_HAS_MANIFOLD ((CERES_VERSION_MAJOR * 100 + CERES_VERSION_MINOR) >= 201)

// Ceres has removed support for CXSPARSE after 2.1
// See https://github.com/ceres-solver/ceres-solver/commit/2335b5b4b7a4703ca9458aa275ca878945763a34
#define ALICEVISION_CERES_HAS_CXSPARSE ((CERES_VERSION_MAJOR * 100 + CERES_VERSION_MINOR) <= 201)

#if ALICEVISION_CERES_HAS_MANIFOLD
using CeresManifold = ceres::Manifold;
#else
class CeresManifold {
public:
    virtual ~CeresManifold() = default;
    virtual int AmbientSize() const = 0;
    virtual int TangentSize() const = 0;
    virtual bool Plus(const double* x,
                      const double* delta,
                      double* x_plus_delta) const = 0;
    virtual bool PlusJacobian(const double* x, double* jacobian) const = 0;
    virtual bool Minus(const double* y,
                       const double* x,
                       double* y_minus_x) const = 0;
    virtual bool MinusJacobian(const double* x, double* jacobian) const = 0;
};

class ManifoldToParameterizationWrapper : public ceres::LocalParameterization
{
public:
    // takes ownership of manifold
    ManifoldToParameterizationWrapper(CeresManifold* manifold) : _manifold{manifold} {}

    ~ManifoldToParameterizationWrapper() override = default;

    bool Plus(const double* x, const double* delta, double* x_plus_delta) const override
    {
        return _manifold->Plus(x, delta, x_plus_delta);
    }

    bool ComputeJacobian(const double* x, double* jacobian) const override
    {
        return _manifold->PlusJacobian(x, jacobian);
    }

    int GlobalSize() const override
    {
        return _manifold->AmbientSize();
    }

    int LocalSize() const override
    {
        return _manifold->TangentSize();
    }

private:
    std::unique_ptr<CeresManifold> _manifold;
};
#endif

} // namespace utils
} // namespace aliceVision
