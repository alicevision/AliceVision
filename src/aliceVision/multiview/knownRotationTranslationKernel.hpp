// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/multiview/relativePose/FundamentalError.hpp>
#include <aliceVision/numeric/projection.hpp>

namespace aliceVision {
namespace multiview {

/**
 * @brief Two-point translation estimation solver between two views from a known rotation
 *        Implementation based on [1] => 3.2 Selection of the right solution.
 *
 * @ref [1] "Finding the Exact Rotation Between Two Images Independently of the Translation."
 *          Authors: L.Kneip and R.Siegwart and M.Pollefeys
 *          Date: October 2012
 *          Conference: ECCV
 */
struct TwoPointTranslationSolver
{

  /**
   * @brief Return the minimum number of required samples
   * @return minimum number of required samples
   */
  inline std::size_t getMinimumNbRequiredSamples() const
  {
    return 2;
  }

  /**
   * @brief Return the maximum number of models
   * @return maximum number of models
   */
  inline std::size_t getMaximumNbModels() const
  {
    return 1;
  }

  /**
   * @brief Solve the problem of camera translation.
   * @param[in] xA
   * @param[in] xB
   * @param[in] R
   * @param[out] models t
   */
  void solve(const Mat& xA, const Mat& xB, const Mat3& R, std::vector<robustEstimation::MatrixModel<Vec3>>& models) const
  {
    const Mat3 Rt = R.transpose();

    // A side bearing vectors
    const Vec3 f1 = Vec3(xA.col(0)(0), xA.col(0)(1), 1.);
    const Vec3 f2 = Vec3(xA.col(1)(0), xA.col(1)(1), 1.);

    // B side bearing vectors
    const Vec3 f1prime = Rt * Vec3(xB.col(0)(0), xB.col(0)(1), 1.);
    const Vec3 f2prime = Rt * Vec3(xB.col(1)(0), xB.col(1)(1), 1.);

    // compute the translation of the camera
    const Vec3 c = ((f1.cross(f1prime)).cross(f2.cross(f2prime))).normalized();

    // ensure the translation make the points in front of the cameras
    const Vec3 opticalFlow = f1 - f1prime;
    Vec3 translation = c;
    if (opticalFlow.dot(translation) < 0)
      translation = -translation;
    models.emplace_back(-R * translation);
  }
};

/**
 * @brief Generic Solver to find the translation from a known Rotation.
 */
template<typename SolverT_ = TwoPointTranslationSolver, typename ErrorT_ = relativePose::FundamentalSampsonError, typename ModelT_ = robustEstimation::MatrixModel<Vec3>>
class TranslationFromKnowRotationKernel
{
public:

  using SolverT = SolverT_;
  using ErrorT = ErrorT_;
  using ModelT = ModelT_;

  /**
   * @brief TranslationFromKnowRotation constructor
   * @param[in] pt2DA 2D points [camera coordinates]
   * @param[in] pt2DB 2D points [camera coordinates]
   * @param[in] R rotation matrix
   */
  TranslationFromKnowRotationKernel(const Mat& x1, const Mat& x2, const Mat3& R)
    : _x1(x1)
    , _x2(x2)
    , _R(R)
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
  inline void fit(const std::vector<std::size_t>& samples, std::vector<ModelT>& models) const
  {
    assert(2 == _x1.rows());
    assert(2 == _x2.rows());

    const Mat x1 = ExtractColumns(_x1, samples);
    const Mat x2 = ExtractColumns(_x2, samples);

    _kernelSolver.solve(x1, x2, _R, models);
  }

  /**
   * @brief error : distance of the sample to the epipolar line
   * @param[in] sample
   * @param[in] model
   * @return error value
   */
  inline double error(std::size_t sample, const ModelT& model) const
  {
    Mat34 poseA, poseB;
    P_from_KRt(Mat3::Identity(), Mat3::Identity(), Vec3::Zero(), &poseA);
    P_from_KRt(Mat3::Identity(), _R, model.getMatrix(), &poseB);
    const robustEstimation::Mat3Model F(F_from_P(poseA, poseB));
    return _errorEstimator.error(F, _x1.col(sample), _x2.col(sample));
  }

  /**
   * @brief Return the errors associated to the model and  each sample point
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

  /// left corresponding point
  const Mat& _x1;
  /// right corresponding point
  const Mat& _x2;
  /// two view solver
  const SolverT _kernelSolver = SolverT();
  /// solver error estimation
  const ErrorT _errorEstimator = ErrorT();
  /// R rotation matrix
  const Mat3 _R;
};

}  // namespace multiview
}  // namespace aliceVision
