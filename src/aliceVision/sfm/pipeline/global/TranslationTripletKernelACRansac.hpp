// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/numeric/numeric.hpp>
#include <aliceVision/multiview/conditioning.hpp>
#include <aliceVision/linearProgramming/linearProgramming.hpp>
#include <aliceVision/robustEstimation/ACRansac.hpp>

namespace aliceVision {
namespace sfm {

using namespace aliceVision::trifocal::kernel;

/// AContrario Kernel to solve a translation triplet & structure problem
template <typename SolverArg, typename ErrorArg, typename ModelArg>
class TranslationTripletKernelACRansac
{
public:
  typedef SolverArg Solver;
  typedef ModelArg  Model;

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
    ApplyTransformationToPoints(_x1, _Kinv, &_x1n);
    ApplyTransformationToPoints(_x2, _Kinv, &_x2n);
    ApplyTransformationToPoints(_x3, _Kinv, &_x3n);

    _vecKR[0] = _Kinv * _vecKR[0];
    _vecKR[1] = _Kinv * _vecKR[1];
    _vecKR[2] = _Kinv * _vecKR[2];
  }

  enum { MINIMUM_SAMPLES = Solver::MINIMUM_SAMPLES };
  enum { MAX_MODELS = Solver::MAX_MODELS };

  void Fit(const std::vector<std::size_t>& samples, std::vector<Model>* models) const
  {

    // Create a model from the points
    Solver::Solve(ExtractColumns(_x1n, samples),
                  ExtractColumns(_x2n, samples),
                  ExtractColumns(_x3n, samples),
                  _vecKR, models, _thresholdUpperBound);
  }

  double Error(std::size_t sample, const Model& model) const
  {
    return ErrorArg::Error(model, _x1n.col(sample), _x2n.col(sample), _x3n.col(sample));
  }

  void Errors(const Model& model, std::vector<double>& vec_errors) const
  {
    for(std::size_t sample = 0; sample < _x1n.cols(); ++sample)
      vec_errors[sample] = ErrorArg::Error(model, _x1n.col(sample), _x2n.col(sample), _x3n.col(sample));
  }

  std::size_t NumSamples() const
  {
    return _x1n.cols();
  }

  void Unnormalize(Model* model) const
  {
    // Unnormalize model from the computed conditioning.
    model->P1 = _K * model->P1;
    model->P2 = _K * model->P2;
    model->P3 = _K * model->P3;
  }

  double logalpha0() const
  {
    return _logalpha0;
  }

  double multError() const
  {
    return 1.0;
  }

  Mat3 normalizer1() const
  {
    return _Kinv;
  }

  Mat3 normalizer2() const
  {
    return Mat3::Identity();
  }

  double unormalizeError(double val) const
  {
    return std::sqrt(val) / _Kinv(0,0);
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
};

} // namespace sfm
} // namespace aliceVision
