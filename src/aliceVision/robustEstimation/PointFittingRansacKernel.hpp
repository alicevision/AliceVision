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
#include <aliceVision/robustEstimation/IRansacKernel.hpp>
#include <aliceVision/robustEstimation/PointFittingKernel.hpp>

namespace aliceVision {
namespace robustEstimation {

/**
 * @brief A virtual kernel used for the ACRANSAC / LORANSAC framework.
 * @tparam SolverT_ The minimal solver able to find a solution from a minimum set of points.
 * @tparam ErrorT_ The functor computing the error for each data sample with respect to the estimated model.
 * @tparam UnnormalizerT_ The functor used to normalize the data before the estimation of the model.
 * @tparam ModelT_ = Mat34Model The type of the model to estimate.
 * @tparam SolverLsT_ = SolverT The least square solver that is used to find a solution from any set of
 *         data larger than the minimum required.
 *
 * @note Handle data normalization and compute the corresponding logalpha 0
 *       that depends of the error model (point to line, or point to point)
 */
template <typename SolverT_, typename ErrorT_, typename ModelT_, typename SolverLsT_ = robustEstimation::UndefinedSolver<ModelT_>>
class PointFittingRansacKernel
    : public robustEstimation::IRansacKernel<ModelT_>
    , public robustEstimation::PointFittingKernel<SolverT_, ErrorT_, ModelT_>
{
public:

  using PFKernel = robustEstimation::PointFittingKernel<SolverT_, ErrorT_, ModelT_>;

  PointFittingRansacKernel(const Mat& x1, const Mat& x2)
    : PFKernel(x1, x2)
  {}

  /**
   * @brief Return the minimum number of required samples for the solver
   * @return minimum number of required samples
   */
  std::size_t getMinimumNbRequiredSamples() const override
  {
    return PFKernel::getMinimumNbRequiredSamples();
  }

  std::size_t getMinimumNbRequiredSamplesLS() const override
  {
    return _solverLs.getMinimumNbRequiredSamples();
  }

  /**
   * @brief Return the maximum number of models for the solver
   * @return maximum number of models
   */
  std::size_t getMaximumNbModels() const override
  {
    return PFKernel::getMaximumNbModels();
  }

  /**
   * @brief This function is called to estimate the model from the minimum number
   * of sample \p minSample (i.e. minimal problem solver).
   * @param[in] samples A vector containing the indices of the data to be used for
   * the minimal estimation.
   * @param[out] models The model(s) estimated by the minimal solver.
   */
  void fit(const std::vector<std::size_t>& samples, std::vector<ModelT_>& models) const override
  {
    PFKernel::fit(samples, models);
  }


  void fitLS(const std::vector<std::size_t>& inliers, std::vector<ModelT_>& models, const std::vector<double>* weights = nullptr) const override
  {
    const Mat x1 = ExtractColumns(PFKernel::_x1, inliers);
    const Mat x2 = ExtractColumns(PFKernel::_x2, inliers);

    if(weights == nullptr)
      _solverLs.solve(x1, x2, models);
    else
      _solverLs.solve(x1, x2, models, *weights);
  }

  void computeWeights(const ModelT_& model, const std::vector<std::size_t>& inliers,  std::vector<double>& weights, const double eps = 0.001) const override
  {
    const auto numInliers = inliers.size();
    weights.resize(numInliers);
    for(std::size_t sample = 0; sample < numInliers; ++sample)
    {
      const auto idx = inliers[sample];
      weights[sample] = PFKernel::_errorEstimator.error(model, PFKernel::_x1.col(idx), PFKernel::_x2.col(idx));
      // avoid division by zero
      weights[sample] = 1.0 / std::pow(std::max(eps, weights[sample]), 2);
    }
  }

  /**
   * @brief Function that computes the estimation error for a given model and a given element.
   * @param[in] sample The index of the element for which the error is computed.
   * @param[in] model The model to consider.
   * @return The estimation error for the given element and the given model.
   */
  double error(std::size_t sample, const ModelT_& model) const override
  {
    return PFKernel::error(sample, model);
  }

  /**
   * @brief Function that computes the estimation error for a given model and all the elements.
   * @param[in] model The model to consider.
   * @param[out] vec_errors The vector containing all the estimation errors for every element.
   */
  void errors(const ModelT_& model, std::vector<double>& errors) const override
  {
    PFKernel::errors(model, errors);
  }

  /**
   * @brief Function used to unnormalize the model.
   * @param[in,out] model The model to unnormalize.
   */
  virtual void unnormalize(ModelT_& model) const = 0;

  /**
   * @brief The number of elements in the data.
   * @return the number of elements in the data.
   */
  std::size_t nbSamples() const override
  {
    return PFKernel::nbSamples();
  }

  /**
   * @brief Get logalpha0, Alpha0 is used to make the error adaptive to the image size
   * @return logalpha0
   */
  virtual double logalpha0() const = 0;

  virtual double multError() const = 0;
  virtual double unormalizeError(double val) const = 0;
  virtual Mat3 normalizer1() const = 0;
  virtual Mat3 normalizer2() const = 0;

private:
  const SolverLsT_ _solverLs{};
};

} // namespace robustEstimation
} // namespace aliceVision
