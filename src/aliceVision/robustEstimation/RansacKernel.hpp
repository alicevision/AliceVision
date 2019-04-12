// This file is part of the AliceVision project.
// Copyright (c) 2019 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/config.hpp>
#include <aliceVision/numeric/numeric.hpp>
#include <aliceVision/multiview/ISolver.hpp>
#include <aliceVision/multiview/TwoViewKernel.hpp>
#include <aliceVision/multiview/conditioning.hpp>
#include <aliceVision/robustEstimation/IRansacKernel.hpp>

/**
 * @brief Collection of kernel.
 * @ref [1] "Robust and accurate calibration of camera networks". PhD. Pierre MOULON
 *
 * - RelativePoseKernel: relative pose two view estimation (affine, homography, fundamental) with unkwon intrinsic
 * - RelativePoseKernel_K: relative pose two view estimation for essential matrix with known intrinsics
 * - ResectionKernel: pose / resection estimation with unknown intrinsic
 * - ResectionKernel_K: pose / resection estimation with known intrinsic
 * - AngularRadianErrorKernel: essential matrix estimation between spherical camera
 *
 *  Mainly it add correct data normalization and define the function required
 *  by the generic ACRANSAC / LORANSAC routine.
 */

namespace aliceVision {
namespace robustEstimation{

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
template <typename SolverT_, typename ErrorT_, typename ModelT_, typename SolverLsT_ = multiview::UndefinedSolver<ModelT_>>
class RansacKernel
    : public IRansacKernel<ModelT_>
    , public multiview::TwoViewKernel<SolverT_, ErrorT_, ModelT_>
{
public:

  using KernelBase = multiview::TwoViewKernel<SolverT_, ErrorT_, ModelT_>;

  RansacKernel(const Mat& x1, const Mat& x2)
    : KernelBase(x1, x2)
  {}

  /**
   * @brief Return the minimum number of required samples for the solver
   * @return minimum number of required samples
   */
  virtual std::size_t getMinimumNbRequiredSamples() const override
  {
    return KernelBase::getMinimumNbRequiredSamples();
  }

  std::size_t getMinimumNbRequiredSamplesLS() const override
  {
    return _solverLs.getMinimumNbRequiredSamples();
  }

  /**
   * @brief Return the maximum number of models for the solver
   * @return maximum number of models
   */
  virtual std::size_t getMaximumNbModels() const override
  {
    return KernelBase::getMaximumNbModels();
  }

  /**
   * @brief This function is called to estimate the model from the minimum number
   * of sample \p minSample (i.e. minimal problem solver).
   * @param[in] samples A vector containing the indices of the data to be used for
   * the minimal estimation.
   * @param[out] models The model(s) estimated by the minimal solver.
   */
  virtual void fit(const std::vector<std::size_t>& samples, std::vector<ModelT_>& models) const override
  {
    KernelBase::fit(samples, models);
  }


  virtual void fitLS(const std::vector<std::size_t>& inliers, std::vector<ModelT_>& models, const std::vector<double>* weights = nullptr) const override
  {
    const Mat x1 = ExtractColumns(KernelBase::_x1, inliers);
    const Mat x2 = ExtractColumns(KernelBase::_x2, inliers);

    if(weights == nullptr)
      _solverLs.solve(x1, x2, models);
    else
      _solverLs.solve(x1, x2, models, *weights);
  }

  virtual void computeWeights(const ModelT_& model, const std::vector<std::size_t>& inliers,  std::vector<double>& weights, const double eps = 0.001) const override
  {
    const auto numInliers = inliers.size();
    weights.resize(numInliers);
    for(std::size_t sample = 0; sample < numInliers; ++sample)
    {
      const auto idx = inliers[sample];
      weights[sample] = KernelBase::_errorEstimator.error(model, KernelBase::_x1.col(idx), KernelBase::_x2.col(idx));
      // avoid division by zero
      weights[sample] = 1/std::pow(std::max(eps, weights[sample]), 2);
    }
  }

  /**
   * @brief Function that computes the estimation error for a given model and a given element.
   * @param[in] sample The index of the element for which the error is computed.
   * @param[in] model The model to consider.
   * @return The estimation error for the given element and the given model.
   */
  virtual double error(std::size_t sample, const ModelT_& model) const override
  {
    return KernelBase::error(sample, model);
  }

  /**
   * @brief Function that computes the estimation error for a given model and all the elements.
   * @param[in] model The model to consider.
   * @param[out] vec_errors The vector containing all the estimation errors for every element.
   */
  virtual void errors(const ModelT_& model, std::vector<double>& errors) const override
  {
    KernelBase::errors(model, errors);
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
  virtual std::size_t nbSamples() const override
  {
    return KernelBase::nbSamples();
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
  const SolverLsT_ _solverLs = SolverLsT_();
};


/**
 * @brief Two view Kernel adapter for the A contrario (AC) model estimator.
 *        With unknown intrinsic.
 *
 * @note Handle data normalization and compute the corresponding logalpha 0
 *       that depends of the error model (point to line, or point to point)
 *
 * @note This kernel adapter is working for affine, homography, fundamental matrix
 *       estimation.
 */
template<typename SolverT_, typename ErrorT_, typename UnnormalizerT_, typename ModelT_ = multiview::Mat3Model, typename SolverLsT_ = multiview::UndefinedSolver<ModelT_>>
class RelativePoseKernel
    : public RansacKernel<SolverT_, ErrorT_, ModelT_, SolverLsT_>
{
public:

  using KernelBase = RansacKernel<SolverT_, ErrorT_, ModelT_, SolverLsT_>;

  RelativePoseKernel(const Mat& x1, int w1, int h1,
                     const Mat& x2, int w2, int h2,
                     bool pointToLine = true)
    : _x1(x1.rows(), x1.cols())
    , _x2(x2.rows(), x2.cols())
    , KernelBase(_x1, _x2)
    , _logalpha0(0.0)
    , _N1(3, 3)
    , _N2(3, 3)
    , _pointToLine(pointToLine)
  {
    assert(2 == x1.rows());
    assert(x1.rows() == x2.rows());
    assert(x1.cols() == x2.cols());

    multiview::normalizePointsFromImageSize(x1, &_x1, &_N1, w1, h1);
    multiview::normalizePointsFromImageSize(x2, &_x2, &_N2, w2, h2);

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
  Mat _x1, _x2;
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
 * @note Handle data normalization and compute the corresponding logalpha 0
 *       that depends of the error model (point to line, or point to point)
 *
 * @note This kernel adapter is working  only for essential matrix
 *       estimation.
 */
template<typename SolverT_, typename ErrorT_, typename ModelT_ = multiview::Mat3Model, typename SolverLsT_ = multiview::UndefinedSolver<ModelT_>>
class RelativePoseKernel_K
    : public RansacKernel<SolverT_, ErrorT_, ModelT_, SolverLsT_>
{
public:

  using KernelBase = RansacKernel<SolverT_, ErrorT_, ModelT_, SolverLsT_>;

  RelativePoseKernel_K(const Mat& x1, int w1, int h1,
                       const Mat& x2, int w2, int h2,
                       const Mat3& K1, const Mat3& K2)
    : KernelBase(x1, x2)
    , _N1(Mat3::Identity())
    , _N2(Mat3::Identity())
    , _logalpha0(0.0)
    , _K1(K1)
    , _K2(K2)
  {
    assert(2 == x1.rows());
    assert(x1.rows() == x2.rows());
    assert(x1.cols() == x2.cols());

    multiview::applyTransformationToPoints(x1, _K1.inverse(), &_x1k);
    multiview::applyTransformationToPoints(x2, _K2.inverse(), &_x2k);

    // point to line probability (line is the epipolar line)
    const double D = sqrt(w2 * static_cast<double>(w2) + h2 * static_cast<double>(h2)); // diameter
    const double A = w2 * static_cast<double>(h2); // area
    _logalpha0 = log10(2.0 * D / A * .5);
  }

  void fit(const std::vector<std::size_t>& samples, std::vector<ModelT_>& models) const override
  {
    const Mat x1 = ExtractColumns(_x1k, samples);
    const Mat x2 = ExtractColumns(_x2k, samples);

    KernelBase::KernelBase::_kernelSolver.solve(x1, x2, models);
  }

  double error(std::size_t sample, const ModelT_& model) const override
  {
    Mat3 F;
    fundamentalFromEssential(model.getMatrix(), _K1, _K2, &F);
    const ModelT_ modelF(F);
    return _errorEstimator.error(modelF, KernelBase::_x1.col(sample), KernelBase::_x2.col(sample));
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
 * @brief Pose/Resection Kernel with unknown intrinsic for the A contrario (AC) model estimator.
 */
template<typename SolverT_, typename ErrorT_, typename UnnormalizerT_, typename ModelT_ = multiview::Mat34Model, typename SolverLsT_ = multiview::UndefinedSolver<ModelT_>>
class ResectionKernel
    : public RansacKernel<SolverT_, ErrorT_, ModelT_, SolverLsT_>
{
public:

  using KernelBase = RansacKernel<SolverT_, ErrorT_, ModelT_, SolverLsT_>;

  /**
   * @brief ResectionKernel constructor with unknown intrinsic
   * @param[in] x2d 2d points in the first image. One per column.
   * @param[in] x3d 3d points in the second image. One per column.
   * @param[in] K intrinsic camera parameter
   */
  ResectionKernel(const Mat& x2d, const Mat& x3d, int w, int h)
    : _x2d(x2d.rows(), x2d.cols())
    , KernelBase(_x2d, x3d)
    , _logalpha0(log10(M_PI))
    , _N1(3, 3)
  {
    assert(2 == x2d.rows());
    assert(3 == x3d.rows());
    assert(x2d.cols() == x3d.cols());

    multiview::normalizePointsFromImageSize(x2d, &_x2d, &_N1, w, h);
  }

  void unnormalize(ModelT_& model) const override
  {
    // unnormalize model from the computed conditioning.
    Mat34 P = model.getMatrix();
    UnnormalizerT_::unnormalize(_N1, Mat3::Identity(), &P);
    model.setMatrix(P);
  }

  double logalpha0() const override {return _logalpha0; }
  double multError() const override {return 1.0;} // point to point error
  Mat3 normalizer1() const override {return Mat3::Identity();}
  Mat3 normalizer2() const override {return _N1;}
  double unormalizeError(double val) const override {return sqrt(val) / _N1(0,0);}

protected:
  /// Normalized input data
  Mat _x2d;
  /// Matrix used to normalize data
  Mat3 _N1;
  /// Alpha0 is used to make the error adaptive to the image size
  double _logalpha0;
};


/**
 * @brief The kernel for the resection with known intrinsics (PnP) to be used with
 * the ACRANSAC / LORANSAC framework.
 *
 * @tparam SolverT The minimal solver able to find a solution from a minimum set of points, usually any PnP solver.
 * @tparam ErrorT The functor computing the error for each data sample with respect to the estimated model, usually a reprojection error functor.
 * @tparam UnnormalizerT The functor used to normalize the data before the estimation of the model, usually a functor that normalize the point in camera
 *         coordinates (ie multiply by the inverse of the calibration matrix).
 * @tparam SolverLsT = SolverT The least square solver that is used to find a solution from any set of data larger than the minimum required,
 *          usually the 6 point algorithm which solves the resection problem by means of LS.
 * @tparam ModelT = Mat34Model The type of the model to estimate, the projection matrix.
 */
template<typename SolverT_, typename ErrorT_, typename UnnormalizerT_, typename ModelT_ = multiview::Mat34Model, typename SolverLsT_ = multiview::UndefinedSolver<ModelT_>>
class ResectionKernel_K
    : public RansacKernel<SolverT_, ErrorT_, ModelT_, SolverLsT_>
{
public:

  using KernelBase = RansacKernel<SolverT_, ErrorT_, ModelT_, SolverLsT_>;

  /**
   * @brief ResectionKernel constructor with known intrinsic
   * @param[in] x2d 2d points in the first image. One per column.
   * @param[in] x3d 3d points in the second image. One per column.
   * @param[in] K intrinsic camera parameter
   */
  ResectionKernel_K(const Mat& x2d, const Mat& x3d, const Mat3& K)
    : _x2d(x2d.rows(), x2d.cols())
    , KernelBase(_x2d, x3d)
    , _logalpha0(log10(M_PI))
    , _N1(K.inverse())
    , _K(K)
  {
    assert(2 == x2d.rows());
    assert(3 == x3d.rows());
    assert(x2d.cols() == x3d.cols());

    // normalize points by inverse K
    multiview::applyTransformationToPoints(x2d, _N1, &_x2d);
  }

  void unnormalize(ModelT_& model) const override
  {
    // unnormalize model from the computed conditioning.
    model.setMatrix(_K * model.getMatrix());
  }

  double logalpha0() const override {return _logalpha0; }
  double multError() const override {return 1.0;} // point to point error
  Mat3 normalizer1() const override {return Mat3::Identity();}
  Mat3 normalizer2() const override {return _N1;}
  double unormalizeError(double val) const override {return sqrt(val) / _N1(0,0);}

protected:
  /// Normalized input data
  Mat _x2d;
  /// Matrix used to normalize data
  Mat3 _N1;
  /// Alpha0 is used to make the error adaptive to the image size
  double _logalpha0;
  /// intrinsic camera parameter
  Mat3 _K;
};

/**
 * @brief Two view Kernel adapter for the A contrario (AC) model estimator.
 *        Specialization to handle radian angular residual error.
 */
template<typename SolverT_, typename ErrorT_, typename ModelT_ = multiview::Mat3Model>
class AngularRadianErrorKernel
    : public RansacKernel<SolverT_, ErrorT_, ModelT_, multiview::UndefinedSolver<ModelT_>>
{
public:

  using KernelBase = RansacKernel<SolverT_, ErrorT_, ModelT_, multiview::UndefinedSolver<ModelT_>>;

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

} // namespace robustEstimation
} // namespace aliceVision
