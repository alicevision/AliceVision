// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// Copyright (c) 2010 libmv contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/robustEstimation/conditioning.hpp>
#include <aliceVision/robustEstimation/PointFittingKernel.hpp>
#include <aliceVision/multiview/relativePose/Essential5PSolver.hpp>
#include <aliceVision/multiview/relativePose/Essential8PSolver.hpp>
#include <aliceVision/multiview/relativePose/FundamentalError.hpp>
#include <aliceVision/multiview/essential.hpp>
#include <aliceVision/multiview/Unnormalizer.hpp>

namespace aliceVision {
namespace multiview {
namespace relativePose {

/**
 * @brief Generic Solver for the 5pt Essential Matrix Estimation.
 *
 * @todo  Error must be overwrite in order to compute F from E and K's.
 *        Fitting must normalize image values to camera values.
 */
template<typename SolverT, typename ErrorT, typename ModelT = robustEstimation::Mat3Model>
class EssentialKernel : public robustEstimation::PointFittingKernel<SolverT, ErrorT, ModelT>
{
public:

  using KernelBase = robustEstimation::PointFittingKernel<SolverT, ErrorT, ModelT>;

  EssentialKernel(const Mat& x1, const Mat& x2, const Mat3& K1, const Mat3& K2)
    : robustEstimation::PointFittingKernel<SolverT, ErrorT, ModelT>(x1,x2)
    , _K1(K1)
    , _K2(K2)
  {}

  void fit(const std::vector<std::size_t>& samples, std::vector<ModelT>& models) const override
  {
    const Mat x1 = ExtractColumns(KernelBase::_x1, samples);
    const Mat x2 = ExtractColumns(KernelBase::_x2, samples);

    assert(2 == x1.rows());
    assert(KernelBase::_kernelSolver.getMinimumNbRequiredSamples() <= x1.cols());
    assert(x1.rows() == x2.rows());
    assert(x1.cols() == x2.cols());

    // normalize the data (image coords to camera coords).
    const Mat3 K1Inverse = _K1.inverse();
    const Mat3 K2Inverse = _K2.inverse();

    Mat x1_normalized;
    Mat x2_normalized;

    robustEstimation::applyTransformationToPoints(x1, K1Inverse, &x1_normalized);
    robustEstimation::applyTransformationToPoints(x2, K2Inverse, &x2_normalized);

    KernelBase::_kernelSolver.solve(x1_normalized, x2_normalized, models);
  }

  double error(std::size_t sample, const ModelT& model) const
  {
    Mat3 F;
    fundamentalFromEssential(model.getMatrix(), _K1, _K2, &F);
    robustEstimation::Mat3Model modelF(F);
    return KernelBase::_errorEstimator.error(modelF, KernelBase::_x1.col(sample), KernelBase::_x2.col(sample));
  }

protected:

  // The two camera calibrated camera matrix
  Mat3 _K1, _K2;
};

/**
 * @brief Solver kernel for the 8pt Essential Matrix Estimation
 */
typedef EssentialKernel<Essential8PSolver, FundamentalSampsonError, robustEstimation::Mat3Model>  Essential8PKernel;

/**
 * @brief Solver kernel for the 5pt Essential Matrix Estimation
 */
typedef EssentialKernel<Essential5PSolver, FundamentalSampsonError, robustEstimation::Mat3Model>  Essential5PKernel;


}  // namespace relativePose
}  // namespace multiview
}  // namespace aliceVision
