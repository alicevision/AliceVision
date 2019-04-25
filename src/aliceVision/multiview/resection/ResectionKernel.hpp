// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/robustEstimation/PointFittingKernel.hpp>
#include <aliceVision/multiview/resection/Resection6PSolver.hpp>
#include <aliceVision/multiview/resection/ProjectionDistanceError.hpp>

namespace aliceVision {
namespace multiview {
namespace resection {

/**
 * @brief Generic solver for resection algorithm using linear least squares.
 */
template<typename SolverT, typename ErrorT, typename ModelT = robustEstimation::Mat34Model>
class ResectionKernel : public robustEstimation::PointFittingKernel<SolverT,ErrorT, ModelT>
{
public:

  using KernelBase = robustEstimation::PointFittingKernel<SolverT,ErrorT, ModelT>;

  ResectionKernel(const Mat& x2d, const Mat& x3d)
    : robustEstimation::PointFittingKernel<SolverT, ErrorT, ModelT>(x2d, x3d)
  {}

  void fit(const std::vector<std::size_t>& samples, std::vector<ModelT>& models) const override
  {
    const Mat x2d = ExtractColumns(KernelBase::_x1, samples);
    const Mat x3d = ExtractColumns(KernelBase::_x2, samples);

    assert(2 == x2d.rows());
    assert(3 == x3d.rows());
    assert(KernelBase::_kernelSolver.getMinimumNbRequiredSamples() <= x2d.cols());
    assert(x2d.cols() == x3d.cols());

    KernelBase::_kernelSolver.solve(x2d, x3d, models);
  }
};

/**
 * @brief Usable solver for the 6pt Resection estimation
 */
typedef robustEstimation::PointFittingKernel<Resection6PSolver, ProjectionDistanceError, robustEstimation::Mat34Model>  Resection6PKernel;

}  // namespace resection
}  // namespace multiview
}  // namespace aliceVision
