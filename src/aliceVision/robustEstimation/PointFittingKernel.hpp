// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// Copyright (c) 2010 libmv contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/numeric/numeric.hpp>
#include <aliceVision/robustEstimation/conditioning.hpp>
#include <aliceVision/robustEstimation/ISolver.hpp>

#include <vector>
#include <cassert>

namespace aliceVision {
namespace robustEstimation {

/**
 * @brief This is one example (targeted at solvers that operate on correspondences
 * between two views) that shows the "kernel" part of a robust fitting
 * problem:
 *
 *   1. The model; Mat3Model in the case of the F or H matrix.
 *   2. The minimum number of samples needed to fit; 7 or 8 (or 4).
 *   3. A way to convert samples to a model.
 *   4. A way to convert a sample and a model to an error.
 *
 * Of particular note is that the kernel does not expose what the samples are.
 * All the robust fitting algorithm sees is that there is some number of
 * samples; it is able to fit subsets of them (via the kernel) and check their
 * error, but can never access the samples themselves.
 *
 * The Kernel objects must follow the following concept so that the robust
 * fitting algorithm can fit this type of relation:
 *ModelT
 *   1. kernel.getMaximumNbModels()
 *   2. kernel.getMinimumNbRequiredSamples()
 *   3. kernel.fit(std::vector<std::size_t>, std::vector<ModelT>&)
 *   4. kernel.error(std::size_t, ModelT) -> error
 *
 * The fit routine must not clear existing entries in the vector of models; it
 * should append new solutions to the end.
 */
template<typename SolverT_, typename ErrorT_, typename ModelT_ = Mat3Model>
class PointFittingKernel
{
public:

  using SolverT = SolverT_;
  using ErrorT = ErrorT_;
  using ModelT = ModelT_;

  PointFittingKernel(const Mat& x1, const Mat& x2)
    : _x1(x1)
    , _x2(x2)
  {}

  /**
   * @brief Return the minimum number of required samples
   * @return minimum number of required samples
   */
  inline std::size_t getMinimumNbRequiredSamples() const
  {
    return _kernelSolver.getMinimumNbRequiredSamples();
  }

  /**
   * @brief Return the maximum number of models
   * @return maximum number of models
   */
  inline std::size_t getMaximumNbModels() const
  {
    return _kernelSolver.getMaximumNbModels();
  }

  /**
   * @brief Extract required sample and fit model(s) to the sample
   * @param[in] samples
   * @param[out] models
   */
  inline virtual void fit(const std::vector<std::size_t>& samples, std::vector<ModelT>& models) const
  {
    const Mat x1 = ExtractColumns(_x1, samples);
    const Mat x2 = ExtractColumns(_x2, samples);
    _kernelSolver.solve(x1, x2, models);
  }

  /**
   * @brief Return the error associated to the model and a sample point
   * @param[in] sample
   * @param[in] model
   * @return error value
   */
  inline virtual double error(std::size_t sample, const ModelT& model) const
  {
    return _errorEstimator.error(model, _x1.col(sample), _x2.col(sample));
  }

  /**
   * @brief Return the errors associated to the model and each sample point
   * @param[in] model
   * @param[out] errors
   */
  inline virtual void errors(const ModelT& model, std::vector<double>& errors) const
  {
    errors.resize(_x1.cols());
    for(std::size_t sample = 0; sample < _x1.cols(); ++sample)
      errors.at(sample) = error(sample, model);
  }

  /**
   * @brief get the number of putative points
   * @return number of putative points
   */
  inline std::size_t nbSamples() const
  {
    return _x1.cols();
  }

protected:

  /// left corresponding data
  const Mat& _x1;
  /// right corresponding data
  const Mat& _x2;
  /// two view solver
  const SolverT _kernelSolver{};
  /// solver error estimation
  const ErrorT _errorEstimator{};
};

template<typename SolverT_, typename ErrorT_, typename UnnormalizerT_, typename ModelT_ = Mat3Model>
class NormalizedPointFittingKernel : public PointFittingKernel<SolverT_, ErrorT_, ModelT_>
{
public:

  using KernelBase = PointFittingKernel<SolverT_, ErrorT_, ModelT_>;

  NormalizedPointFittingKernel(const Mat& x1, const Mat& x2)
    : KernelBase(x1, x2)
  {}

  /**
   * @brief Extract required sample and fit model(s) to the sample
   * @param[in] samples
   * @param[out] models
   */
  inline void fit(const std::vector<std::size_t>& samples, std::vector<ModelT_>& models) const override
  {
    const Mat x1 = ExtractColumns(KernelBase::_x1, samples);
    const Mat x2 = ExtractColumns(KernelBase::_x2, samples);

    assert(2 == x1.rows());
    assert(KernelBase::getMinimumNbRequiredSamples() <= x1.cols());
    assert(x1.rows() == x2.rows());
    assert(x1.cols() == x2.cols());

    // normalize the data.
    Mat3 T1;
    Mat3 T2;
    Mat x1_normalized;
    Mat x2_normalized;

    normalizePoints(x1, &x1_normalized, &T1);
    normalizePoints(x2, &x2_normalized, &T2);

    KernelBase::_kernelSolver.solve(x1_normalized, x2_normalized, models);

    // unnormalize model from the computed conditioning.
    for(int i = 0; i < models.size(); ++i)
      UnnormalizerT_::unnormalize(T1, T2, &(models.at(i).getMatrix()));
  }
};

}  // namespace robustEstimation
}  // namespace aliceVision
