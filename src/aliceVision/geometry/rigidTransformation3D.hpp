// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/numeric/numeric.hpp>
#include <aliceVision/numeric/LMFunctor.hpp>
#include <aliceVision/robustEstimation/ISolver.hpp>
#include <random>

namespace aliceVision {
namespace geometry {

/**
 * @brief Compose a similarity matrix given a scale factor, a rotation matrix and 
 * a translation vector.
 * 
 * @param[in] S The scale factor.
 * @param[in] t The translation vector.
 * @param[in] R The rotation matrix.
 * @param[out] RTS The 4x4 similarity matrix [R*S | t; 0 0 0 1].
 */
inline void composeRTS(double &S, const Vec3 &t, const Mat3 &R, Mat4 &RTS)
{
  RTS.topLeftCorner(3, 3) = R*S;
  RTS.topRightCorner(3, 1) = t;
}

/**
 * @brief Decompose a similarity matrix into its scale, rotation and translation parts
 * 
 * @param[in] RTS The similarity matrix to decompose.
 * @param[out] S The scale factor.
 * @param[out] t The translation part.
 * @param[out] R The rotation part.
 * @return true if the input matrix is a similarity matrix.
 */
inline bool decomposeRTS(const Mat4 &RTS, double &S, Vec3 &t, Mat3 &R)
{
  // Check critical cases
  R = RTS.topLeftCorner<3, 3>();
  if (R.determinant() < 0)
    return false;
  S = pow(R.determinant(), 1.0 / 3.0);
  // Check for degenerate case (if all points have the same value...)
  if (S < std::numeric_limits<double>::epsilon())
    return false;

  // Extract transformation parameters
  R /= S;
  t = RTS.topRightCorner<3, 1>();
  return true;
}

/** 3D rigid transformation estimation (7 dof)
 * Compute a Scale Rotation and Translation rigid transformation.
 * This transformation provide a distortion-free transformation
 * using the following formulation Xb = S * R * Xa + t.
 * "Least-squares estimation of transformation parameters between two point patterns",
 * Shinji Umeyama, PAMI 1991, DOI: 10.1109/34.88573
 *
 * \param[in] x1 The first 3xN matrix of euclidean points
 * \param[in] x2 The second 3xN matrix of euclidean points
 * \param[out] S The scale factor
 * \param[out] t The 3x1 translation
 * \param[out] R The 3x3 rotation
 *
 * \return true if the transformation estimation has succeeded
 *
 * \note Need at least 3 points
 */

inline bool FindRTS(const Mat &x1,
  const Mat &x2,
  double &S,
  Vec3 &t,
  Mat3 &R)
{
  if (x1.cols() < 3 || x2.cols() < 3)
    return false;

  assert(3 == x1.rows());
  assert(3 <= x1.cols());
  assert(x1.rows() == x2.rows());
  assert(x1.cols() == x2.cols());

  // Get the transformation via Umeyama's least squares algorithm. This returns
  // a matrix of the form:
  // [ s * R t]
  // [ 0 1]
  // from which we can extract the scale, rotation, and translation.
  const Eigen::Matrix4d transform = Eigen::umeyama(x1, x2, true);

  return decomposeRTS(transform, S, t, R);
}

// Eigen LM functor to refine translation, Rotation and Scale parameter.
struct lm_SRTRefine_functor : LMFunctor<double>
{
  lm_SRTRefine_functor(int inputs, int values,
    const Mat &x1, const Mat &x2,
    const double &S, const Mat3 & R, const Vec &t): LMFunctor<double>(inputs,values),
    _x1(x1), _x2(x2), _t(t), _R(R), _S(S) { }

  int operator()(const Vec &x, Vec &fvec) const;

  Mat _x1, _x2;
  Vec3 _t;
  Mat3 _R;
  double _S;
};

// Eigen LM functor to refine Rotation.
struct lm_RRefine_functor : LMFunctor<double>
{
  lm_RRefine_functor(int inputs, int values,
    const Mat &x1, const Mat &x2,
    const double &S, const Mat3 & R, const Vec &t): LMFunctor<double>(inputs,values),
    _x1(x1), _x2(x2), _t(t), _R(R), _S(S) { }

  int operator()(const Vec &x, Vec &fvec) const;

  Mat _x1, _x2;
  Vec3 _t;
  Mat3 _R;
  double _S;
};

/** 3D rigid transformation refinement using LM
 * Refine the Scale, Rotation and translation rigid transformation
 * using a Levenberg-Marquardt opimization.
 *
 * \param[in] x1 The first 3xN matrix of euclidean points
 * \param[in] x2 The second 3xN matrix of euclidean points
 * \param[out] S The initial scale factor
 * \param[out] t The initial 3x1 translation
 * \param[out] R The initial 3x3 rotation
 *
 * \return none
 */
void Refine_RTS(const Mat &x1,
                const Mat &x2,
                double &S,
                Vec3 &t,
                Mat3 &R);

/**
 * @brief the Solver to use for ACRansac
 */
class RTSSolver : public robustEstimation::ISolver<robustEstimation::MatrixModel<Mat4>>
{
public:

  /**
   * @brief Return the minimum number of required samples
   * @return minimum number of required samples
   */
  inline std::size_t getMinimumNbRequiredSamples() const override
  {
    return 3;
  }

  /**
   * @brief Return the maximum number of models
   * @return maximum number of models
   */
  inline std::size_t getMaximumNbModels() const override
  {
    return 1;
  }

  // solve the RTS problem
  inline void solve(const Mat& pts1, const Mat& pts2, std::vector<robustEstimation::MatrixModel<Mat4>>& models) const
  {
    models.push_back( robustEstimation::MatrixModel<Mat4>(Eigen::umeyama(pts1, pts2, true)) );
  }

  void solve(const Mat& x1, const Mat& x2, std::vector<robustEstimation::MatrixModel<Mat4>>& models, const std::vector<double>& weights) const override
  {
     throw std::logic_error("RTSSolver does not support problem solving with weights.");
  }

  // compute the residual of the transformation
  inline double error(const robustEstimation::MatrixModel<Mat4>& RTS, const Vec3& pt1, const Vec3& pt2)
  {
    const Mat4& matrixRTS = RTS.getMatrix();
    const Mat3& RS = matrixRTS.topLeftCorner<3, 3>();
    const Vec3& t = matrixRTS.topRightCorner<3, 1>();
    return(pt2 - (RS*pt1 + t)).norm();
  }
};

/**
 * @brief A functor that computes the squared error between points transformed by
 * a similarity
 */
struct RTSSquaredResidualError 
{
  // return the squared error
  inline double error(const robustEstimation::MatrixModel<Mat4>& RTS, const Vec3& pt1, const Vec3& pt2) const
  {
    const Mat4& matrixRTS = RTS.getMatrix();
    const Mat3& RS = matrixRTS.topLeftCorner<3, 3>();
    const Vec3& t = matrixRTS.topRightCorner<3, 1>();
    return (pt2 - (RS*pt1 + t)).squaredNorm();
  }
};

/**
 * @brief The kernel to use for ACRansac
 */
template <typename SolverT_, typename ErrorT_, typename ModelT_ = robustEstimation::MatrixModel<Mat4>>
class ACKernelAdaptor_PointsRegistrationSRT
{
public:
  using SolverT = SolverT_;
  using ModelT = ModelT_;
  using ErrorT = ErrorT_;

  ACKernelAdaptor_PointsRegistrationSRT(const Mat& xA, const Mat& xB)
    : x1_(xA)
    , x2_(xB)
    , _logalpha0(log10(M_PI)) //@todo  WTF?
  {
    assert(3 == x1_.rows());
    assert(x1_.rows() == x2_.rows());
    assert(x1_.cols() == x2_.cols());
    
    // @todo normalize points?
  }

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

  void fit(const std::vector<std::size_t>& samples, std::vector<ModelT>& models) const
  {
    const Mat x1 = ExtractColumns(x1_, samples);
    const Mat x2 = ExtractColumns(x2_, samples);
    _kernelSolver.solve(x1, x2, models);
  }

  double error(std::size_t sample, const ModelT& model) const
  {
    return Square(_errorEstimator.error(model, x1_.col(sample), x2_.col(sample)));
  }

  void errors(const ModelT& model, std::vector<double> & vec_errors) const
  {
    vec_errors.resize(x1_.cols());
    for(std::size_t sample = 0; sample < x1_.cols(); ++sample)
      vec_errors[sample] = error(sample,model);
  }

  std::size_t nbSamples() const
  {
    return static_cast<std::size_t> (x1_.cols());
  }

  void unnormalize(ModelT& model) const { } //-- Do nothing, no normalization

  double logalpha0() const { return _logalpha0; }

  double multError() const { return 1.;}

  Mat3 normalizer1() const { return Mat3::Identity(); }

  Mat3 normalizer2() const { return Mat3::Identity(); }

  double unormalizeError(double val) const { return sqrt(val); }

private:
  Mat x1_, x2_;       // normalized input data
  double _logalpha0;  // alpha0 is used to make the error scale invariant

  SolverT _kernelSolver;
  ErrorT _errorEstimator;
};

/**
 * @brief Uses AC ransac to robustly estimate the similarity between two sets of points.
 * 
 * @param[in] x1 The first 3xN matrix of euclidean points.
 * @param[in] x2 The second 3xN matrix of euclidean points.
 * @param[in] randomNumberGenerator random number generator
 * @param[out] S The scale factor.
 * @param[out] t The 3x1 translation.
 * @param[out] R The 3x3 rotation.
 * @param[out] vec_inliers The vector containing the indices of inliers points.
 * @param[in] refine Enable/Disable refining of the found transformation.
 * @return true if the found transformation is a similarity
 * @see FindRTS()
 */
bool ACRansac_FindRTS(const Mat &x1,
                      const Mat &x2,
                      std::mt19937 &randomNumberGenerator,
                      double &S,
                      Vec3 &t,
                      Mat3 &R,
                      std::vector<std::size_t> &vec_inliers,
                      bool refine = false);

/**
 * @brief Uses AC ransac to robustly estimate the similarity between two sets of 
 * points. Just a wrapper that output the similarity in matrix form
 * @param[in] x1 The first 3xN matrix of euclidean points.
 * @param[in] x2 The second 3xN matrix of euclidean points.
 * @param[in] randomNumberGenerator random number generator
 * @param[out] RTS The 4x4 similarity matrix. 
 * @param[out] vec_inliers The inliers used to estimate the similarity.
 * @param  refine Enable/Disable refining of the found transformation.
 * @return true if the found transformation is a similarity
 * @see geometry::FindRTS()
 * @see geometry::ACRansac_FindRTS()
 */
inline bool ACRansac_FindRTS(const Mat &x1,
                             const Mat &x2, 
                             std::mt19937 &randomNumberGenerator,
                             Mat4 &RTS, 
                             std::vector<std::size_t> &vec_inliers,
                             bool refine = false)
{
  double S;
  Vec3 t; 
  Mat3 R; 
  const bool good = ACRansac_FindRTS(x1, x2, randomNumberGenerator ,S, t, R, vec_inliers, refine);
  if(good)
    composeRTS(S, t, R, RTS);
  
  return good;
}

} // namespace geometry
} // namespace aliceVision
