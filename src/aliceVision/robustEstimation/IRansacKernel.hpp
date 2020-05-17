// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/numeric/numeric.hpp>

#include <vector>

namespace aliceVision {
namespace robustEstimation{

/**
 * @brief A generic kernel used for the ACRANSAC / LORANSAC framework.
 * @tparam ModelT The class representing the model to estimate.
 *
 * @note Handle data normalization and compute the corresponding logalpha 0
 *       that depends of the error model (point to line, or point to point)
 */
template <typename ModelT>
class IRansacKernel
{
  public:

  /**
   * @brief Return the minimum number of required samples for the solver
   * @return minimum number of required samples
   */
  virtual std::size_t getMinimumNbRequiredSamples() const = 0;

  /**
   * @brief Return the minimum number of required samples for the solver Ls
   * @return minimum number of required samples
   */
  virtual std::size_t getMinimumNbRequiredSamplesLS() const = 0;

  /**
   * @brief Return the maximum number of models for the solver
   * @return maximum number of models
   */
  virtual std::size_t getMaximumNbModels() const = 0;

  /**
   * @brief This function is called to estimate the model from the minimum number
   * of sample \p minSample (i.e. minimal problem solver).
   * @param[in] samples A vector containing the indices of the data to be used for
   * the minimal estimation.
   * @param[out] models The model(s) estimated by the minimal solver.
   */
  virtual void fit(const std::vector<std::size_t>& samples, std::vector<ModelT>& models) const = 0;

  /**
   * @brief This function is called to estimate the model using a least squared
   * algorithm from a minumum of \p minSampleLS.
   * @param[in] inliers An array containing the indices of the data to use.
   * @param[out] models The model(s) estimated using the least squared algorithm.
   * @param[in] weights An optional array of weights, one for each sample
   */
  virtual void fitLS(const std::vector<std::size_t>& inliers, std::vector<ModelT>& models, const std::vector<double>* weights = nullptr) const = 0;

  /**
   * @brief Function used to estimate the weights, typically used by the least square algorithm.
   * @param[in] model The model against which the weights are computed.
   * @param[in] inliers The array of the indices of the data to be used.
   * @param[out] vec_weights The array of weight of the same size as \p inliers.
   * @param[in] eps An optional threshold to max out the value of the threshold (typically
   * to avoid division by zero or too small numbers).
   */
  virtual void computeWeights(const ModelT& model, const std::vector<std::size_t>& inliers, std::vector<double>& weights, const double eps = 0.001) const = 0;

  /**
   * @brief Function that computes the estimation error for a given model and a given element.
   * @param[in] sample The index of the element for which the error is computed.
   * @param[in] model The model to consider.
   * @return The estimation error for the given element and the given model.
   */
  virtual double error(std::size_t sample, const ModelT& model) const = 0;

  /**
   * @brief Function that computes the estimation error for a given model and all the elements.
   * @param[in] model The model to consider.
   * @param[out] vec_errors The vector containing all the estimation errors for every element.
   */
  virtual void errors(const ModelT& model, std::vector<double>& errors) const = 0;

  /**
   * @brief Function used to unnormalize the model.
   * @param[in,out] model The model to unnormalize.
   */
  virtual void unnormalize(ModelT& model) const = 0;

  /**
   * @brief The number of elements in the data.
   * @return the number of elements in the data.
   */
  virtual std::size_t nbSamples() const = 0;

  /**
   * @brief Get logalpha0, Alpha0 is used to make the error adaptive to the image size
   * @return logalpha0
   */
  virtual double logalpha0() const = 0;

  virtual double multError() const = 0;
  virtual double unormalizeError(double val) const = 0;
  virtual Mat3 normalizer1() const = 0;
  virtual Mat3 normalizer2() const = 0;
};

} // namespace robustEstimation
} // namespace aliceVision
