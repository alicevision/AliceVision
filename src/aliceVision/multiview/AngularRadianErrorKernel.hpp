// This file is part of the AliceVision project.
// Copyright (c) 2019 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/config.hpp>
#include <aliceVision/numeric/numeric.hpp>
#include <aliceVision/robustEstimation/conditioning.hpp>
#include <aliceVision/robustEstimation/ISolver.hpp>
#include <aliceVision/robustEstimation/PointFittingRansacKernel.hpp>

/**
 * @brief Collection of kernel.
 * @ref [1] "Robust and accurate calibration of camera networks". PhD. Pierre MOULON
 *
 * - AngularRadianErrorKernel: essential matrix estimation between spherical camera
 *
 *  Mainly it add correct data normalization and define the function required
 *  by the generic ACRANSAC / LORANSAC routine.
 */

namespace aliceVision {
namespace multiview {

/**
 * @brief Two view Kernel adapter for the A contrario (AC) model estimator.
 *        Specialization to handle radian angular residual error.
 */
template<typename SolverT_, typename ErrorT_, typename ModelT_ = robustEstimation::Mat3Model>
class AngularRadianErrorKernel
    : public robustEstimation::PointFittingRansacKernel<SolverT_, ErrorT_, ModelT_, robustEstimation::UndefinedSolver<ModelT_>>
{
public:

  using KernelBase = robustEstimation::PointFittingRansacKernel<SolverT_, ErrorT_, ModelT_, robustEstimation::UndefinedSolver<ModelT_>>;

  AngularRadianErrorKernel(const Mat& x1, const Mat& x2)
    : KernelBase(x1, x2)
    , _logalpha0(log10(1.0 / 2.0))
  {
    assert(3 == x1.rows());
    assert(x1.rows() == x2.rows());
    assert(x1.cols() == x2.cols());
  }

  inline double error(std::size_t sample, const ModelT_& model) const override
  {
    return Square(KernelBase::error(sample, model));
  }

  void unnormalize(ModelT_& model) const override
  {
    // do nothing, no normalization in the angular case
  }

  double logalpha0() const override {return _logalpha0; }
  double multError() const override {return 1./4.;}
  Mat3 normalizer1() const override {return Mat3::Identity();}
  Mat3 normalizer2() const override {return Mat3::Identity();}
  double unormalizeError(double val) const override {return sqrt(val);}

protected:
  /// Alpha0 is used to make the error adaptive to the image size
  double _logalpha0;
};

} // namespace multiview
} // namespace aliceVision
