// This file is part of the AliceVision project.
// Copyright (c) 2019 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/numeric/numeric.hpp>

#include <vector>

namespace aliceVision {
namespace robustEstimation {

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
 * @brief Matrix based model to be used in a solver.
 */
template<typename MatrixT>
struct MatrixModel
{
  MatrixModel() = default;

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

protected:
  MatrixT _matrix{};
};

/**
 * @brief explicit typename for MatrixModel with Mat3 matrix (fundamental, essential, homography, ...)
 */
using Mat3Model = MatrixModel<Mat3>;

/**
 * @brief explicit typename for MatrixModel with Mat34 matrix (projection, ...)
 */
using Mat34Model = MatrixModel<Mat34>;


} // namespace robustEstimation
} // namespace aliceVision
