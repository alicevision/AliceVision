// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// Copyright (c) 2007 libmv contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/numeric/numeric.hpp>
#include <aliceVision/robustEstimation/ISolver.hpp>
#include <aliceVision/robustEstimation/IRansacKernel.hpp>

namespace aliceVision {
namespace robustEstimation {

inline double pointToLineError(const Vec2& lineEq, const Vec2& xs)
{
  const double b = lineEq[0];
  const double a = lineEq[1];
  const double x = xs[0];
  const double y = xs[1];
  const double e = y - (a * x + b);
  return e*e;
}

/**
 * @brief Embed the basic solver to fit from sampled point set
 */
class LineKernel : public IRansacKernel<robustEstimation::MatrixModel<Vec2>>
{
public:

  using ModelT = robustEstimation::MatrixModel<Vec2>;

  explicit LineKernel(const Mat2X& xs)
    : _xs(xs)
    , _logalpha0(0)
  {}

  LineKernel(const Mat2X& xs, int w, int h)
    : _xs(xs)
  {
    // model error as point to line error
    // ratio of containing diagonal image rectangle over image area
    const double D = sqrt(w * w * 1.0 + h * h); // diameter
    const double A = w * h; // area
    _logalpha0 = log10(2.0 * D / A / 1.0);
  }

  /**
   * @brief Return the minimum number of required samples
   * @return minimum number of required samples
   */
  inline std::size_t getMinimumNbRequiredSamples() const override
  {
    return 2;
  }

  /**
   * @brief Return the maximum number of models
   * @return maximum number of models
   */
  inline std::size_t getMaximumNbModels() const override
  {
    return 1;
  }

  /**
   * @brief Return the minimum number of required samples for the solver Ls
   * @return minimum number of required samples
   */
  inline std::size_t getMinimumNbRequiredSamplesLS() const override
  {
    return 2;
  }

  /**
   * @brief The number of elements in the data.
   * @return the number of elements in the data.
   */
  inline std::size_t nbSamples() const override
  {
    return static_cast<std::size_t>(_xs.cols());
  }

  /**
   * @brief This function is called to estimate the model from the minimum number
   * of sample \p minSample (i.e. minimal problem solver).
   * @param[in] samples A vector containing the indices of the data to be used for
   * the minimal estimation.
   * @param[out] models The model(s) estimated by the minimal solver.
   */
  void fit(const std::vector<std::size_t>& samples, std::vector<ModelT>& models) const override
  {
    assert(samples.size() >= getMinimumNbRequiredSamples());

    // standard least squares solution.
    const Mat2X sampled_xs = ExtractColumns(_xs, samples);

    Mat X(sampled_xs.cols(), 2);
    X.col(0).setOnes();
    X.col(1) = sampled_xs.row(0).transpose();

    const Mat A(X.transpose() * X);
    const Vec b(X.transpose() * sampled_xs.row(1).transpose());

    Eigen::JacobiSVD<Mat> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);

    const Vec2 ba = svd.solve(b);

    models.emplace_back(ba);
  }

  /**
   * @brief This function is called to estimate the model using a least squared
   * algorithm from a minumum of \p minSampleLS.
   * @param[in] inliers An array containing the indices of the data to use.
   * @param[out] models The model(s) estimated using the least squared algorithm.
   * @param[in] weights An optional array of weights, one for each sample
   */
  void fitLS(const std::vector<std::size_t>& samples, std::vector<ModelT>& models, const std::vector<double>* weights = nullptr) const override
  {
    if(weights == nullptr)
    {
      fit(samples, models);
      return;
    }

    assert(samples.size() >= getMinimumNbRequiredSamples());

    // standard least squares solution.
    const Mat2X sampled_xs = ExtractColumns(_xs, samples);


    const std::size_t numPts = sampled_xs.cols();

    assert(numPts == weights->size());

    // create the weight matrix
    Mat W = Mat::Zero(numPts, numPts);
    for(std::size_t i = 0; i < numPts; ++i)
      W(i,i) = (*weights)[i];

    Mat X(numPts, 2);
    X.col(0).setOnes();
    X.col(1) = sampled_xs.row(0).transpose();
    const Mat A(X.transpose() * W * X);
    const Vec b(X.transpose() * W * sampled_xs.row(1).transpose());
    Eigen::JacobiSVD<Mat> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
    const Vec2 ba = svd.solve(b);
    models.emplace_back(ba);
  }

  /**
   * @brief Function used to estimate the weights, typically used by the least square algorithm.
   * @param[in] model The model against which the weights are computed.
   * @param[in] inliers The array of the indices of the data to be used.
   * @param[out] vec_weights The array of weight of the same size as \p inliers.
   * @param[in] eps An optional threshold to max out the value of the threshold (typically
   * to avoid division by zero or too small numbers).
   */
  void computeWeights(const ModelT& model, const std::vector<std::size_t>& inliers, std::vector<double>& weights, const double eps = 0.001)  const override
  {
    const auto nbInliers = inliers.size();
    weights.resize(nbInliers);
    for(std::size_t sample = 0; sample < nbInliers; ++sample)
    {
      const auto idx = inliers[sample];
      weights[sample] = error(idx, model);

      // avoid division by zero
      weights[sample] = 1 / std::pow(std::max(eps, weights[sample]), 2);
    }
  }

  /**
   * @brief Function that computes the estimation error for a given model and a given element.
   * @param[in] sample The index of the element for which the error is computed.
   * @param[in] model The model to consider.
   * @return The estimation error for the given element and the given model.
   */
  inline double error(std::size_t sample, const ModelT& model) const override
  {
    // point to line error
    const Vec2& xs = _xs.col(sample);
    return pointToLineError(model.getMatrix(), xs);
  }

  /**
   * @brief Function that computes the estimation error for a given model and all the elements.
   * @param[in] model The model to consider.
   * @param[out] vec_errors The vector containing all the estimation errors for every element.
   */
  inline void errors(const ModelT& model, std::vector<double>& errors) const override
  {
    errors.resize(_xs.cols());
    for(std::size_t sample = 0; sample < _xs.cols(); ++sample)
      errors.at(sample) = error(sample, model);
  }
  
  /**
   * @brief Function used to unnormalize the model.
   * @param[in,out] model The model to unnormalize.
   */
  inline void unnormalize(ModelT& model) const override
  {
    // nothing to do, model is left unchanged
  }

  /**
   * @brief Get logalpha0, Alpha0 is used to make the error adaptive to the image size
   * @return logalpha0
   */
  inline double logalpha0() const override
  {
    return _logalpha0;
  }

  inline double multError() const override
  {
    return 0.5;
  }

  inline Mat3 normalizer1() const override
  {
    return Mat3::Identity();
  }

  inline Mat3 normalizer2() const override
  {
    return Mat3::Identity();
  }

  inline double unormalizeError(double val) const override
  {
    return std::sqrt(val);
  }

private:
  const Mat2X& _xs;
  double _logalpha0;
};

} // namespace robustEstimation
} // namespace aliceVision
