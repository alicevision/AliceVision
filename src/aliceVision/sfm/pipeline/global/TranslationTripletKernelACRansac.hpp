// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/numeric/numeric.hpp>
#include <aliceVision/robustEstimation/conditioning.hpp>
#include <aliceVision/linearProgramming/linearProgramming.hpp>
#include <aliceVision/robustEstimation/IRansacKernel.hpp>
#include <aliceVision/robustEstimation/ACRansac.hpp>

namespace aliceVision {
namespace sfm {

using namespace aliceVision::trifocal::kernel;

/// AContrario Kernel to solve a translation triplet & structure problem
template <typename SolverT_, typename ErrorT_, typename ModelT_>
class TranslationTripletKernelACRansac
    : robustEstimation::IRansacKernel<ModelT_>
{
public:
  using SolverT = SolverT_;
  using ErrorT = SolverT_;
  using ModelT =  ModelT_;

  TranslationTripletKernelACRansac(const Mat& x1,
                                   const Mat& x2,
                                   const Mat& x3,
                                   const std::vector<Mat3>& vec_KRi,
                                   const Mat3& K,
                                   const double ThresholdUpperBound)
    : _x1(x1)
    , _x2(x2)
    , _x3(x3)
    , _vecKR(vec_KRi)
    , _K(K)
    , _thresholdUpperBound(ThresholdUpperBound)
    , _logalpha0(log10(M_PI))
    , _Kinv(K.inverse())
  {
    // Normalize points by inverse(K)
    robustEstimation::applyTransformationToPoints(_x1, _Kinv, &_x1n);
    robustEstimation::applyTransformationToPoints(_x2, _Kinv, &_x2n);
    robustEstimation::applyTransformationToPoints(_x3, _Kinv, &_x3n);

    _vecKR[0] = _Kinv * _vecKR[0];
    _vecKR[1] = _Kinv * _vecKR[1];
    _vecKR[2] = _Kinv * _vecKR[2];
  }
  /**
   * @brief Return the minimum number of required samples for the solver
   * @return minimum number of required samples
   */
  inline std::size_t getMinimumNbRequiredSamples() const override
  {
    return _kernelSolver.getMinimumNbRequiredSamples();
  }

  /**
   * @brief Return the maximum number of models for the solver
   * @return maximum number of models
   */
  inline std::size_t getMaximumNbModels() const override
  {
    return _kernelSolver.getMaximumNbModels();
  }

  void fit(const std::vector<std::size_t>& samples, std::vector<ModelT_>& models) const override
  {

    // create a model from the points
    _kernelSolver.solve(ExtractColumns(_x1n, samples),
                        ExtractColumns(_x2n, samples),
                        ExtractColumns(_x3n, samples),
                        _vecKR, models, _thresholdUpperBound);
  }

  /**
   * @brief error
   * @param sample
   * @param model
   * @return
   */
  double error(std::size_t sample, const ModelT_& model) const override
  {
    return _errorEstimator.error(model, _x1n.col(sample), _x2n.col(sample), _x3n.col(sample));
  }

  void errors(const ModelT_& model, std::vector<double>& errors) const override
  {
    for(std::size_t sample = 0; sample < _x1n.cols(); ++sample)
      errors[sample] = error(sample, model);
  }

  std::size_t nbSamples() const override
  {
    return _x1n.cols();
  }

  void unnormalize(ModelT_& model) const override
  {
    // unnormalize model from the computed conditioning.
    model.P1 = _K * model.P1;
    model.P2 = _K * model.P2;
    model.P3 = _K * model.P3;
  }

  double logalpha0() const override
  {
    return _logalpha0;
  }

  double multError() const override
  {
    return 1.0;
  }

  Mat3 normalizer1() const override
  {
    return _Kinv;
  }

  Mat3 normalizer2() const override
  {
    return Mat3::Identity();
  }

  double unormalizeError(double val) const override
  {
    return std::sqrt(val) / _Kinv(0,0);
  }

  std::size_t getMinimumNbRequiredSamplesLS() const
  {
    throw std::logic_error("TranslationTripletKernelACRansac cannot be used in LO_RANSAC.");
    return 0;
  }

  void fitLS(const std::vector<std::size_t>& inliers, std::vector<ModelT>& models, const std::vector<double>* weights = nullptr) const
  {
    throw std::logic_error("TranslationTripletKernelACRansac cannot be used in LO_RANSAC.");
  }

  void computeWeights(const ModelT& model, const std::vector<std::size_t>& inliers, std::vector<double> & weights, const double eps = 0.001) const
  {
    throw std::logic_error("TranslationTripletKernelACRansac cannot be used in LO_RANSAC.");
  }

private:
  const Mat& _x1;
  const Mat& _x2;
  const Mat& _x3;
  Mat _x1n;
  Mat _x2n;
  Mat _x3n;
  const Mat3 _Kinv, _K;
  const double _logalpha0;
  const double _thresholdUpperBound;
  std::vector<Mat3> _vecKR;

  /// two view solver
  const SolverT _kernelSolver = SolverT();
  /// solver error estimation
  const ErrorT _errorEstimator = ErrorT();
};

} // namespace sfm
} // namespace aliceVision
