// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// Copyright (c) 2010 libmv contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/robustEstimation/ISolver.hpp>

namespace aliceVision {
namespace multiview {
namespace relativePose {

class Essential5PSolver : public robustEstimation::ISolver<robustEstimation::Mat3Model>
{
public:

  /**
   * @brief Return the minimum number of required samples
   * @return minimum number of required samples
   */
  inline std::size_t getMinimumNbRequiredSamples() const override
  {
    return 5;
  }

  /**
   * @brief Return the maximum number of models
   * @return maximum number of models
   */
  inline std::size_t getMaximumNbModels() const override
  {
    return 10;
  }

  /**
   * @brief Computes the relative pose of two calibrated cameras from 5 correspondences.
   * @param[in] x1 Points in the first image.  One per column.
   * @param[in] x2 Corresponding points in the second image. One per column.
   * @param[out] models  A list of at most 10 candidate essential matrix solutions.
   */
   void solve(const Mat& x1, const Mat& x2, std::vector<robustEstimation::Mat3Model>& models) const override;

   /**
    * @brief Solve the problem.
    * @param[in]  x1  A 2xN matrix of column vectors.
    * @param[in]  x2  A 2xN (relative pose) or 3xN (resection) matrix of column vectors.
    * @param[out] models A vector into which the computed models are stored.
    * @param[in]  weights.
    */
   void solve(const Mat& x1, const Mat& x2, std::vector<robustEstimation::Mat3Model>& models, const std::vector<double>& weights) const override
   {
      throw std::logic_error("Essential5PSolver does not support problem solving with weights.");
   }
};

/**
 * In the following code, polynomials are expressed as vectors containing
 * their coefficients in the basis of monomials:
 *
 *  [xxx xxy xyy yyy xxz xyz yyz xzz yzz zzz xx xy yy xz yz zz x y z 1]
 *
 * Note that there is an error in Stewenius' paper.  In equation (9) they
 * propose to use the basis:
 *
 *  [xxx xxy xxz xyy xyz xzz yyy yyz yzz zzz xx xy xz yy yz zz x y z 1]
 *
 * But this is not the basis used in the rest of the paper, neither in
 * the code they provide.  I (pau) have spend 4 hours debugging and
 * reverse engineering their code to find the problem. :(
 */
enum polynomialCoefficient {
  coef_xxx,
  coef_xxy,
  coef_xyy,
  coef_yyy,
  coef_xxz,
  coef_xyz,
  coef_yyz,
  coef_xzz,
  coef_yzz,
  coef_zzz,
  coef_xx,
  coef_xy,
  coef_yy,
  coef_xz,
  coef_yz,
  coef_zz,
  coef_x,
  coef_y,
  coef_z,
  coef_1
};

using Pc = polynomialCoefficient;

/**
 * @brief Multiply two polynomials of degree 1.
 */
Vec o1(const Vec& a, const Vec& b);


/**
 * @brief Multiply a polynomial of degree 2, a, by a polynomial of degree 1, b.
 */
Vec o2(const Vec& a, const Vec& b);

/**
 * @brief Compute the nullspace of the linear constraints given by the matches.
 */
Mat fivePointsNullspaceBasis(const Mat2X& x1, const Mat2X& x2);

}  // namespace relativePose
}  // namespace multiview
}  // namespace aliceVision
