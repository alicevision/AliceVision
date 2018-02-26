// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/numeric/numeric.hpp>

#include <iostream>

namespace aliceVision {
namespace resection {

/**
 * @brief The structure p4fSolution contain one output model
 */
struct p4fSolution
{
  p4fSolution(Mat R, Vec3 t, double f)
    : _R(R)
    , _t(t)
    , _f(f)
  {}

  Mat34 getP() const
  {
    Mat34 P;
    Mat K = Mat(3, 3);

    K << _f, 0, 0,
         0, _f, 0,
         0,  0, 1;

    P.block(0, 0, 3, 3) = K*_R;
    P.block(0, 3, 3, 1) = K*_t;

    return P;
  }

  Mat _R;
  Vec3 _t;
  double _f;
};

/**
 * @brief  Compute the absolute pose and focal length of a camera using three 3D-to-2D correspondences
 * @author Martin Bujnak, adapted to aliceVision by Michal Polic
 * @ref [1] A general solution to the p4p
 *          Bujnak, M., Kukelova, Z., and Pajdla T.
 *          CVPR 2008
 */
struct P4PfSolver
{
  enum
  {
    MINIMUM_SAMPLES = 4
  };

  enum
  {
    MAX_MODELS = 10
  };

  /**
   * @brief Solve the problem of camera pose.
   *
   * @param[in] pt2Dx featureVectors:
   * 2 x 4 matrix with feature vectors with subtracted principal point (each column is a vector)
   *
   * @param[in] pt3Dx worldPoints:
   * 3 x 4 matrix with corresponding 3D world points (each column is a point)
   *
   * @param[out] solutions
   * M x n vector that will contain the each solution in structure p4fSolution (rotation matrix p4fSolution._R,
   * translation vector p4fSolution._t, focal length p4fSolution._f). Following equation holds for each solution:
   * lambda*pt2D = diag([p4fSolution._f p4fSolution._f 1])*[p4fSolution._R p4fSolution._t] * pt3D
   */
  static void solve(const Mat &pt2Dx,
                    const Mat &pt3Dx,
                    std::vector<p4fSolution> *models);

  /**
   * @brief Compute the residual of the projection distance(pt2D, Project(P,pt3D)).
   * @param[in] solution
   * @param[in] pt2D feature vector
   * @param[in] pt3D coresponding 3D world point
   */
  static double error(const p4fSolution &model,
                      const Vec2 &pt2D,
                      const Vec3 &pt3D);
};

/**
 * @brief GJ elimination
 * @param[in,out] A
 * @param[in] rcnt
 * @param[in] ccnt
 * @param[in] tol
 */
void GJ(double *A, int rcnt, int ccnt, double tol);

/**
 * @brief Prepare polynomial coefficients.
 * @param[in] src1
 * @param[in] src2
 * @param[in] src3
 * @param[in] src4
 * @param[in] src5
 * @param[out] dst1
 */
void computeCoefficients(const double  *src1,
                         const double  *src2,
                         const double  *src3,
                         const double  *src4,
                         const double  *src5,
                         double *dst1);

/**
 * @brief Compute P4pf Poses
 * [glab, glac, glad, glbc, glbd, glcd], [a1; a2], [b1; b2], [c1; c2], [d1;d2]
 *
 * @param[in] glab - glXY - ||X-Y||^2 - quadratic distances between 3D points X and Y
 * @param[in] a1 (a2) = x (resp y) measurement of the first 2D point
 * @param[in] b1 (b2) = x (resp y) measurement of the second 2D point
 * @param[in] c1 (c2) = x (resp y) measurement of the third 2D point
 * @param[in] d1 (d2) = x (resp y) measurement of the fourth 2D point
 * @param[out] A 10 x 10 action matrix
 */
void computeP4pfPoses(const double *glab,
                      const double *a1,
                      const double *b1,
                      const double *c1,
                      const double *d1,
                      double *A);

/**
 * @brief isNan
 * @param[in] A matrix
 */
bool isNan(const Eigen::MatrixXcd &A);

/**
 * @brief validSol
 * @param[in] sol
 * @param[out] vSol
 */
bool validSol(const Eigen::MatrixXcd &sol,
              Mat &vSol);

/**
 * @brief Get the rigid transformation
 * @param[in] pp1
 * @param[in] pp2
 * @param[out] R
 * @param[out] t
 */
void getRigidTransform(const Mat &pp1,
                       const Mat &pp2,
                       Mat &R,
                       Vec3 &t);

} // namespace resection
} // namespace aliceVision
