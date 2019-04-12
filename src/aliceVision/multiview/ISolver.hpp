// This file is part of the AliceVision project.
// Copyright (c) 2019 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/numeric/numeric.hpp>

namespace aliceVision {
namespace multiview {

/**
 * @brief Generic solver interface.
 */
template<typename ModelT_>
class ISolver
{
public:

  /**
   * @brief Return the minimum number of required samples
   * @return minimum number of required samples
   */
  virtual std::size_t getMinimumNbRequiredSamples() const = 0;

  /**
   * @brief Return the maximum number of models
   * @return maximum number of models
   */
  virtual std::size_t getMaximumNbModels() const = 0;

  /**
   * @brief Solve the problem.
   * @param[in]  x1  A 2xN matrix of column vectors.
   * @param[in]  x2  A 2xN (relative pose) or 3xN (resection) matrix of column vectors.
   * @param[out] models A vector into which the computed models are stored.
   */
  virtual void solve(const Mat& x1, const Mat& x2, std::vector<ModelT_>& models) const = 0;

  /**
   * @brief Solve the problem.
   * @param[in]  x1  A 2xN matrix of column vectors.
   * @param[in]  x2  A 2xN (relative pose) or 3xN (resection) matrix of column vectors.
   * @param[out] models A vector into which the computed models are stored.
   * @param[in]  weights.
   */
  virtual void solve(const Mat& x1, const Mat& x2, std::vector<ModelT_>& models, const std::vector<double>& weights) const = 0;
};


/**
 * @brief An Undefined Solver
 */
template<typename ModelT_>
class UndefinedSolver : public ISolver<ModelT_>
{
public:

  std::size_t getMinimumNbRequiredSamples() const override
  {
    throw std::runtime_error("Undefined solver used in kernel.");
    return 0;
  }

  std::size_t getMaximumNbModels() const override
  {
    throw std::runtime_error("Undefined solver used in kernel.");
    return 0;
  }

  void solve(const Mat& x1, const Mat& x2, std::vector<ModelT_>& models) const override
  {
    throw std::runtime_error("Undefined solver used in kernel.");
  }

  void solve(const Mat& x1, const Mat& x2, std::vector<ModelT_>& models, const std::vector<double>& weights) const override
  {
    throw std::runtime_error("Undefined solver used in kernel.");
  }
};


/**
 * @brief Generic matrix solver model.
 */
template<typename MatrixT>
struct MatrixModel
{
  MatrixModel()
  {}

  explicit MatrixModel(const MatrixT& matrix)
    : _matrix(matrix)
  {}

  inline const MatrixT& getMatrix() const
  {
    return _matrix;
  }

  inline MatrixT& getMatrix()
  {
    return _matrix;
  }

  inline void setMatrix(const MatrixT& matrix)
  {
    _matrix = matrix;
  }

private:
  MatrixT _matrix;
};

/**
 * @brief explicit typename for MatrixModel with Mat3 matrix (fundamental, essential, homography, ...)
 */
typedef MatrixModel<Mat3> Mat3Model;

/**
 * @brief explicit typename for MatrixModel with Mat34 matrix (projection, ...)
 */
typedef MatrixModel<Mat34> Mat34Model;

/**
 * @brief Relative pose solver error interface.
 */
template <typename ModelT>
struct ISolverErrorRelativePose
{
  virtual double error(const ModelT& model, const Vec2& x1, const Vec2& x2) const = 0;
};

/**
 * @brief Resection solver error interface.
 */
template <typename ModelT>
struct ISolverErrorResection
{
  virtual double error(const ModelT& model, const Vec2& x2d, const Vec3& x3d) const = 0;
};

/**
 * @brief Build a 9 x n matrix from point matches, where each row is equivalent to the
 * equation x'T*F*x = 0 for a single correspondence pair (x', x). The domain of
 * the matrix is a 9 element vector corresponding to F. In other words, set up
 * the linear system
 *
 *   Af = 0,
 *
 * where f is the F matrix as a 9-vector rather than a 3x3 matrix (row
 * major). If the points are well conditioned and there are 8 or more, then
 * the nullspace should be rank one. If the nullspace is two dimensional,
 * then the rank 2 constraint must be enforced to identify the appropriate F
 * matrix.
 *
 * @note that this does not resize the matrix A; it is expected to have the
 * appropriate size already.
 */
template<typename TMatX, typename TMatA>
inline void encodeEpipolarEquation(const TMatX& x1, const TMatX& x2, TMatA* A, const std::vector<double> *weights = nullptr)
{
  assert(x1.cols()==x2.cols());

  if(weights != nullptr)
  {
    assert(x1.cols()==weights->size());
  }

  for(typename TMatX::Index i = 0; i < x1.cols(); ++i)
  {
    const Vec2 xx1 = x1.col(i);
    const Vec2 xx2 = x2.col(i);

    A->row(i) <<
      xx2(0) * xx1(0),  // 0 represents x coords,
      xx2(0) * xx1(1),  // 1 represents y coords.
      xx2(0),
      xx2(1) * xx1(0),
      xx2(1) * xx1(1),
      xx2(1),
      xx1(0),
      xx1(1),
      1.0;

    if(weights != nullptr)
      A->row(i) *= (*weights)[i];
  }
}

} // namespace multiview
} // namespace aliceVision
