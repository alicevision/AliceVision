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

namespace aliceVision {
namespace multiview {

/**
 * @brief Two view Kernel adapter for the A contrario (AC) model estimator.
 *        With unknown intrinsic.
 *
 * @ref [1] "Robust and accurate calibration of camera networks". PhD. Pierre MOULON
 *
 * @note Handle data normalization and compute the corresponding logalpha 0
 *       that depends of the error model (point to line, or point to point)
 *
 * @note This kernel adapter is working for affine, homography, fundamental matrix
 *       estimation.
 */
template<typename SolverT_, typename ErrorT_, typename UnnormalizerT_, typename ModelT_ = robustEstimation::Mat3Model, typename SolverLsT_ = robustEstimation::UndefinedSolver<ModelT_>>
class RelativePoseKernel
    : public robustEstimation::PointFittingRansacKernel<SolverT_, ErrorT_, ModelT_, SolverLsT_>
{
public:

  using PFRansacKernel = robustEstimation::PointFittingRansacKernel<SolverT_, ErrorT_, ModelT_, SolverLsT_>;

  RelativePoseKernel(const Mat& x1, int w1, int h1,
                     const Mat& x2, int w2, int h2,
                     bool pointToLine = true)
    : _x1n(x1.rows(), x1.cols())
    , _x2n(x2.rows(), x2.cols())
    , PFRansacKernel(_x1n, _x2n)  // provide a reference to the internal var members
    , _logalpha0(0.0)
    , _N1(3, 3)
    , _N2(3, 3)
    , _pointToLine(pointToLine)
  {
    ALICEVISION_LOG_TRACE("RelativePoseKernel: x1: " << x1.rows() << "x" << x1.cols() << ", x2: " << x2.rows() << "x" << x2.cols());
    assert(2 == x1.rows());
    assert(x1.rows() == x2.rows());
    assert(x1.cols() == x2.cols());

    robustEstimation::normalizePointsFromImageSize(x1, &_x1n, &_N1, w1, h1);
    robustEstimation::normalizePointsFromImageSize(x2, &_x2n, &_N2, w2, h2);

    // logAlpha0 is used to make error data scale invariant
    if(pointToLine)
    {
      // Ratio of containing diagonal image rectangle over image area
      const double D = sqrt(w2 * (double) w2 + h2 * (double) h2); // diameter
      const double A = w2 * (double) h2; // area
      _logalpha0 = log10(2.0 * D / A / _N2(0, 0));
    }
    else
    {
      // ratio of area : unit circle over image area
      _logalpha0 = log10(M_PI / (w2 * (double) h2) / (_N2(0, 0) * _N2(0, 0)));
    }
  }

  void unnormalize(ModelT_& model) const override
  {
    // Unnormalize model from the computed conditioning.
    Mat3 H = model.getMatrix();
    UnnormalizerT_::unnormalize(_N1, _N2, &H);
    model.setMatrix(H);
  }

  double logalpha0() const override {return _logalpha0; }
  double multError() const override {return (_pointToLine)? 0.5 : 1.0;}
  Mat3 normalizer1() const override {return _N1;}
  Mat3 normalizer2() const override {return _N2;}
  double unormalizeError(double val) const override {return sqrt(val) / _N2(0,0);}

protected:
  /// Normalized input data
  Mat _x1n, _x2n;
  /// Matrix used to normalize data
  Mat3 _N1, _N2;
  /// Alpha0 is used to make the error adaptive to the image size
  double _logalpha0;
  /// Store if error model is pointToLine or point to point
  bool _pointToLine;
};

/**
 * @brief Two view Kernel adapter for the A contrario (AC) model estimator.
 *        With known intrinsic.
 *
 * @ref [1] "Robust and accurate calibration of camera networks". PhD. Pierre MOULON
 *
 * @note Handle data normalization and compute the corresponding logalpha 0
 *       that depends of the error model (point to line, or point to point)
 *
 * @note This kernel adapter is working  only for essential matrix
 *       estimation.
 */
template<typename SolverT_, typename ErrorT_, typename ModelT_ = robustEstimation::Mat3Model, typename SolverLsT_ = robustEstimation::UndefinedSolver<ModelT_>>
class RelativePoseKernel_K
    : public robustEstimation::PointFittingRansacKernel<SolverT_, ErrorT_, ModelT_, SolverLsT_>
{
public:

  using PFRansacKernel = robustEstimation::PointFittingRansacKernel<SolverT_, ErrorT_, ModelT_, SolverLsT_>;

  RelativePoseKernel_K(const Mat& x1, int w1, int h1,
                       const Mat& x2, int w2, int h2,
                       const Mat3& K1, const Mat3& K2)
    : PFRansacKernel(x1, x2)
    , _N1(Mat3::Identity())
    , _N2(Mat3::Identity())
    , _logalpha0(0.0)
    , _K1(K1)
    , _K2(K2)
  {
    ALICEVISION_LOG_TRACE("RelativePoseKernel_K: x1: " << x1.rows() << "x" << x1.cols() << ", x2: " << x2.rows() << "x" << x2.cols());
    assert(2 == x1.rows());
    assert(x1.rows() == x2.rows());
    assert(x1.cols() == x2.cols());

    robustEstimation::applyTransformationToPoints(x1, _K1.inverse(), &_x1k);
    robustEstimation::applyTransformationToPoints(x2, _K2.inverse(), &_x2k);

    // point to line probability (line is the epipolar line)
    const double D = sqrt(w2 * static_cast<double>(w2) + h2 * static_cast<double>(h2)); // diameter
    const double A = w2 * static_cast<double>(h2); // area
    _logalpha0 = log10(2.0 * D / A * .5);
  }

  void fit(const std::vector<std::size_t>& samples, std::vector<ModelT_>& models) const override
  {
    const Mat x1 = ExtractColumns(_x1k, samples);
    const Mat x2 = ExtractColumns(_x2k, samples);

    PFRansacKernel::PFKernel::_kernelSolver.solve(x1, x2, models);
  }

  double error(std::size_t sample, const ModelT_& model) const override
  {
    Mat3 F;
    fundamentalFromEssential(model.getMatrix(), _K1, _K2, &F);
    const ModelT_ modelF(F);
    return _errorEstimator.error(modelF, PFRansacKernel::PFKernel::_x1.col(sample), PFRansacKernel::PFKernel::_x2.col(sample));
  }

  void unnormalize(ModelT_& model) const override
  {
    // do nothing, no normalization in this case
  }

  double logalpha0() const override {return _logalpha0; }
  double multError() const override {return 0.5;} // point to line error
  Mat3 normalizer1() const override {return _N1;}
  Mat3 normalizer2() const override {return _N2;}
  double unormalizeError(double val) const override { return val; }

private:

  Mat _x1k, _x2k;
  /// Matrix used to normalize data
  Mat3 _N1, _N2;
  /// Alpha0 is used to make the error adaptive to the image size
  double _logalpha0;
  /// Intrinsics camera parameter
  Mat3 _K1, _K2;
  /// solver error estimation
  const ErrorT_ _errorEstimator;
};

/**
 * Two view Kernel adapter in case of spherical camera for the A contrario model estimator
 * Handle data normalization and compute the corresponding logalpha 0
 * that depends of the error model (point to line, or point to point)
 * This kernel adapter is working for affine, homography, fundamental matrix estimation.
 */
template <typename SolverT_, typename ErrorT_, typename ModelT_ = robustEstimation::Mat3Model,
          typename SolverLsT_ = robustEstimation::UndefinedSolver<ModelT_>>
class RelativePoseSphericalKernel: public robustEstimation::PointFittingRansacKernel<SolverT_, ErrorT_, ModelT_, SolverLsT_>
{
public:
    using PFRansacKernel = robustEstimation::PointFittingRansacKernel<SolverT_, ErrorT_, ModelT_, SolverLsT_>;

    RelativePoseSphericalKernel(const Mat& x1, const Mat& x2)
        : PFRansacKernel(x1, x2)  // provide a reference to the input matrices
        , _logalpha0(M_PI)
    {
        ALICEVISION_LOG_TRACE("RelativePoseSphericalKernel: x1: " << x1.rows() << "x" << x1.cols() << ", x2: " << x2.rows() << "x" << x2.cols());
        assert(x1.rows() == 3);
        assert(x1.cols() > 0);
        assert(x1.rows() == x2.rows());
        assert(x1.cols() == x2.cols());
    }

    void unnormalize(ModelT_& model) const override
    {
        // do nothing, no normalization in this case
    }

    double logalpha0() const override { return _logalpha0; }
    double multError() const override { return 1.0; }

    Mat3 normalizer1() const { return Mat3::Identity(); }
    Mat3 normalizer2() const { return Mat3::Identity(); }
    double unormalizeError(double val) const override { return sqrt(val); }

protected:
    /// Alpha0 is used to make the error adaptive to the image size
    const double _logalpha0;
};


} // namespace multiview
} // namespace aliceVision
